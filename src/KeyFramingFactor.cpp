#include <KeyFramingFactor.hpp>

void
radarOdom::KeyFraming()
{
	static int cnt = 0;
	int num = window_list.size()-1;

	// KeyFrame factor
	FactorGeneration(0,num,del_list[num-1]);
	cout << "num : " << num << endl;
	/////////////////////////////////////////////////////////
	double d_yx = atan2(del_list[num-1][1],del_list[num-1][0]);
	double d_theta = del_list[num-1][2];
	norm_v[num-1]=sqrt(del_list[num-1][1]*del_list[num-1][1]+del_list[num-1][0]*del_list[num-1][0]);
	norm_w[num-1]=d_theta*180.0/M_PI;

	atv[num-1] = exp(-abs(d_yx + d_theta));

	if(norm_v[num-1] > 1.0){
		if(abs(del_list[num-1][1]) > 2.0)
			atv[num-1] = 0;
		if(abs(norm_w[num-1]) > 90.0)
			atv[num-1] = 0;
	}

    cout << "atv : ";
    for (int ii = 0; ii < num; ii++)
    	cout << atv[ii] << " ";
    cout << endl;

	/////////////////////////////////////////////////////////
    vector<int> y(num);
    size_t n(0);

    generate(std::begin(y), std::end(y), [&]{ return n++; });

    std::sort(  std::begin(y), 
                std::end(y),
                [&](int i1, int i2) { return atv[i1] < atv[i2]; } );

    int iter = 0;
    for (auto v : y) {
        //cout << "(" << v << ") " ;

        cost_idx[iter] = v;
        iter++;
    }
    //cout << endl;

	/////////////////////////////////////////////////////////	

    n = 0;

    generate(std::begin(y), std::end(y), [&]{ return n++; });

    std::sort(  std::begin(y), 
                std::end(y),
                [&](int i1, int i2) { return abs(norm_w[i1]) < abs(norm_w[i2]); } );

    iter = 0;
    for (auto v : y) {
        cost_iter[iter] = v;
        iter++;
    }

	/////////////////////////////////////////////////////////	


    cout << "cost_iter : ";
	for (int ii = 0; ii < num; ii++)
    	cout << cost_iter[ii] << " ";
    cout << endl;

    cout << "norm_v : ";
	for (int ii = 0; ii < num; ii++)
    	cout << norm_v[ii] << " ";
    cout << endl;

    cout << "norm_w : ";
	for (int ii = 0; ii < num; ii++)
    	cout << norm_w[ii] << " ";
    cout << endl;


	////////////////////////////////////////////////////

	if(num > 1) {
		// seq01_river : 0.99 0.90, seq02_dcc : 0.90 0.90, seq03_kaist : 0.8 0.8, seq04_sejong : 
		if( ((atv[num-1] < atv[num-2]) && (atv[num-1] < keyf_threshold_*atv[cost_idx[num-1]])) || num > 3 || norm_v[0] > 30.0) {
			int i = num-2;
			int p_ind = num-2;

			for(int ii = num-1; ii >= 0; ii--){
				if(atv[cost_iter[ii]] > 0.7*atv[cost_idx[num-1]]){
					i = cost_iter[ii];
					break;
				}
			}
			if(i == -1)
				i = cost_idx[num-1];

			p_ind = i;
			cout << p_ind << endl;

			/////////////////////////////////////////////////////////
			for(int ii = 0; ii < num; ii++) {
				if (p_ind == ii) {
					poseGraph->add(PharaoRotFactor(X(key_node), X(pose_count-num+p_ind+1), del_list[p_ind][2], rot_noise_model));
					cout << "prev_key:  " << key_node << "    present_key:   " << pose_count-num + p_ind+1 << endl;
					cout << "Connected Value:  " << del_list[p_ind][2] << endl;
					

				} else if(atv[ii] > keyf_threshold_*atv[cost_idx[num-1]]) {
					poseGraph->add(PharaoRotFactor(X(key_node), X(pose_count-num+ii+1), del_list[ii][2], odom_noise_model));
					cout << "prev_key:  " << key_node << "    present_good:   " << pose_count-num + ii+1 << endl;
					cout << "Connected Value:  " << del_list[ii][2] << endl;
				}
			}

			int p_size = pose_node_nums.size();
			if(p_size > 2) {
				isam2->update(*poseGraph, initial_values);
				isam2->update();
				odom_result = isam2->calculateEstimate();
				//GaussNewtonOptimizer optimizer(*poseGraph, initial_values, parameters);
				//odom_result = LevenbergMarquardtOptimizer(*poseGraph, initial_values).optimize();
				//odom_result = optimizer.optimize();
				poseGraph->resize(0);
    			initial_values.clear();
			
				cout << "p_size : " << p_size << endl;
				/////////////////////////////////////////////////////////

				
				//prev_pose = odom_result.at<Pose2>(X(pose_count));
				prev_pose = odom_result.at<Pose2>(X(key_node));
				cout << "Last Pose value:\n     x:" << prev_pose.translation().x() << "     y:"<< prev_pose.translation().y() << "       theta:"<< prev_pose.rotation().theta()<<endl;
				Eigen::AngleAxisd rollAngle(0.0, Eigen::Vector3d::UnitX());   //M_PI
				Eigen::AngleAxisd pitchAngle(0.0, Eigen::Vector3d::UnitY());
				Eigen::AngleAxisd yawAngle(prev_pose.rotation().theta(), Eigen::Vector3d::UnitZ());
				Eigen::Quaternion<double> gtsam_quat = yawAngle * pitchAngle * rollAngle;

				nav_msgs::Odometry opt_odom;
				opt_odom.header.frame_id = "odom";
				opt_odom.pose.pose.position.x = prev_pose.translation().x();
				opt_odom.pose.pose.position.y = prev_pose.translation().y();
				opt_odom.pose.pose.position.z = 0;
				opt_odom.pose.pose.orientation.w = gtsam_quat.w();
				opt_odom.pose.pose.orientation.x = gtsam_quat.x();
				opt_odom.pose.pose.orientation.y = gtsam_quat.y();
				opt_odom.pose.pose.orientation.z = gtsam_quat.z();
				pub_opt_odom_.publish(opt_odom);

				tf2::Quaternion mQ;
				mQ.setRPY( 0, 0, prev_pose.rotation().theta());
				writeFile.open("test.txt", ios::app);
					if(writeFile.is_open()) {
						writeFile << stamp << ' ' << prev_pose.translation().x() << ' ' << prev_pose.translation().y() << " 0 " << 
									mQ[0] << ' ' << mQ[1] << ' ' << mQ[2] << ' ' << mQ[3] << endl;
						writeFile.close();
					}
				
				key_node = pose_count - num + p_ind + 1;
				cout << "    updated_key:   " << key_node << endl;
				window_loop += num;
				prev_pose = odom_result.at<Pose2>(X(key_node));
				cout << "Covariance at keynode:\n" << isam2->marginalCovariance(X(key_node)) << endl;
				cout << "Factor Graph size:   " << odom_result.size() << endl;
			}

			cv::Mat last_p,last_c,last_cf;
			auto list_iter = window_list.begin() + 1 + p_ind;
			last_p = *list_iter;
			list_iter = window_list_cart.begin() + 1 + p_ind;
			last_c = *list_iter;
			list_iter = window_list_cart_f.begin() + 1 + p_ind;
			last_cf = *list_iter;

			keyf_list.push_back(last_p);
			keyf_list_cart.push_back(last_c);
			keyf_list_cart_f.push_back(last_cf);

			window_list.clear();
			window_list_cart.clear();
			window_list_cart_f.clear();

			window_list.push_back(last_p);
			window_list_cart.push_back(last_c);
			window_list_cart_f.push_back(last_cf);

			cnt++;
			
			exec = true;
		}
	}

}