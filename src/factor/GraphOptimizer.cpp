#include <factor/GraphOptimizer.hpp>

GraphOptimizer::GraphOptimizer(ros::NodeHandle nh, DataContainer* dc)
							: nh_(nh), dc_(dc), FactorConstructor(dc)
{
	nh_.getParam("odom_factor_cost_threshold", odom_threshold_);
	nh_.getParam("keyframe_factor_cost_threshold", keyf_threshold_);

	ISAM2Params parameters;
	parameters.relinearizeThreshold = 0.01;
	parameters.relinearizeSkip = 1;
	isam2 = new ISAM2(parameters);

	Eigen::AngleAxisd rollAngle(0.0, Eigen::Vector3d::UnitX());   //M_PI
	Eigen::AngleAxisd pitchAngle(0.0, Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd yawAngle(0, Eigen::Vector3d::UnitZ());
	Eigen::Quaternion<double> init_q = yawAngle * pitchAngle * rollAngle;

	key_rot = Rot2(0);
	orien = Rot2(0);
	trans_ = Vector2(0,0);
	now_pose = Vector3(0,0,0);
	Pose2 prior_pose = gtsam::Pose2(key_rot, gtsam::Point2(.0,.0));

	initial_values.insert(X(pose_count), prior_pose);

	prior_noise_model_ = noiseModel::Diagonal::Sigmas((Vector(3) << 0.01, 0.01, 0.001).finished());
	odom_noise_model_ = noiseModel::Diagonal::Sigmas((Vector(3) << 1, 1, 1e-1).finished());  // m, m, rad
	key_noise_model_ = noiseModel::Diagonal::Sigmas((Vector(3) << 1,1,1e-3).finished());
	auto rot_vector = (Vector(3) << 1000,1000,1e-2).finished();
	rot_noise_model_ = noiseModel::Diagonal::Sigmas(rot_vector);

	// Add prior factor to the graph.
	poseGraph->addPrior(X(pose_count), prior_pose, prior_noise_model_);
}

GraphOptimizer::~GraphOptimizer()
{

}

void
GraphOptimizer::optimize()
{
	// initialize
	if(dc_->initialized == false){
		dc_->del_list[0] = factorGeneration(0,1);

		dc_->initialized = true;
	}

	imshow("Coarse cart.",*(dc_->window_list_cart.end()-1));

	if(generateOdomFactor()) {
		// Keyframing
		generateKeyfFactor();
	}


}

bool
GraphOptimizer::generateOdomFactor()
{
	// Phase correlation (coarse to fine)
	int num = dc_->window_list.size() - 1;

	int index = 0;

	// Odom. Frame factor
	for(int ii = 1; ii <= num; ii++) {
		int begin = num-ii;
		int end = num;
		dc_->odom_list[num-1] = factorGeneration(begin, end);

		double o_yx = atan2(dc_->odom_list[num-1][1], dc_->odom_list[num-1][0]);
		double o_theta = dc_->odom_list[num-1][2];
		double cost = exp(-abs(o_yx + o_theta));
		double norm = sqrt(pow(dc_->odom_list[num-1][1],2) + pow(dc_->odom_list[num-1][0],2));
		cout << "filtering : " << cost << ", " << o_theta/M_PI*180.0 << ", " << o_yx << endl;

		if(norm < RESOL){ // There is no change.
			dc_->window_list.erase(dc_->window_list.end()-1);
			dc_->window_list_cart.erase(dc_->window_list_cart.end()-1);
			dc_->window_list_cart_f.erase(dc_->window_list_cart_f.end()-1);
			num--;
			dc_->stamp_list.erase(dc_->stamp_list.end()-1);

			return false;
		} else {
			if(cost > odom_threshold_) {

				Pose2 new_odom = gtsam::Pose2(dc_->odom_list[num-1][0], dc_->odom_list[num-1][1], dc_->odom_list[num-1][2]); // x,y,theta
				Vector3 egovec(dc_->odom_list[num-1][0], dc_->odom_list[num-1][1], dc_->odom_list[num-1][2]);
				pose_count ++;
				pose_node_nums.push_back(pose_count);
				if(pose_count-1 == pose_values.size()){
					pose_values.push_back(now_pose);
				}
				cout << "pocnt:  " << pose_count << "    ii:   " << ii << endl;
				cout << "poval:  "  << pose_values.size() << endl;
				cout << "pose matching node : " << pose_count-ii << " & " << pose_count <<endl;

				base_pose = pose_values.at(pose_count-ii);
				now_pose[0] = base_pose[0] * cos(base_pose[2]) - base_pose[1] * sin(base_pose[2]) + egovec[0];
				now_pose[1] = base_pose[0] * sin(base_pose[2]) + base_pose[1] * cos(base_pose[2]) + egovec[1];
				//now_pose = base_pose+egovec;
				Point2 prop_point = gtsam::Point2(now_pose[0],now_pose[1]);
				orien = Rot2(base_pose[2]);
				Pose2 prop_pose = gtsam::Pose2(orien, prop_point);
				poseGraph->add(BetweenFactor<Pose2>(X(pose_count-ii), X(pose_count), new_odom, odom_noise_model_));
				
				cout << "Connected Value:  " << dc_->odom_list[num-1][2] << endl;
				initial_values.insert(X(pose_count), prop_pose);

				// nav_msgs::Odometry odom;
				// odom.header.frame_id = "odom";
				// odom.pose.pose.position.x = new_odom.translation().x();
				// odom.pose.pose.position.y = new_odom.translation().y();
				// odom.pose.pose.position.z = 0;
				// odom.pose.pose.orientation.w = 1;
				// odom.pose.pose.orientation.x = 0;
				// odom.pose.pose.orientation.y = 0;
				// odom.pose.pose.orientation.z = 0;
				// pub_odom_.publish(odom);
				return true;
			}

		}
	}

	dc_->window_list.erase(dc_->window_list.end()-1);
	dc_->window_list_cart.erase(dc_->window_list_cart.end()-1);
	dc_->window_list_cart_f.erase(dc_->window_list_cart_f.end()-1);
	num--;
	dc_->stamp_list.erase(dc_->stamp_list.end()-1);

	return false;
}

void
GraphOptimizer::generateKeyfFactor()
{
	static int cnt = 0;
	int num = dc_->window_list.size()-1;

	// KeyFrame factor
	dc_->del_list[num-1] = factorGeneration(0,num);
	cout << "num : " << num << endl;
	/////////////////////////////////////////////////////////
	double d_yx = atan2(dc_->del_list[num-1][1],dc_->del_list[num-1][0]);
	double d_theta = dc_->del_list[num-1][2];
	norm_v[num-1]=sqrt(dc_->del_list[num-1][1]*dc_->del_list[num-1][1]+dc_->del_list[num-1][0]*dc_->del_list[num-1][0]);
	norm_w[num-1]=d_theta*180.0/M_PI;

	atv[num-1] = exp(-abs(d_yx + d_theta));

	if(norm_v[num-1] > 1.0){
		if(abs(dc_->del_list[num-1][1]) > 2.0)
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
					poseGraph->add(PharaoRotFactor(X(key_node), X(pose_count-num+p_ind+1), dc_->del_list[p_ind][2], rot_noise_model_));
					cout << "prev_key:  " << key_node << "    present_key:   " << pose_count-num + p_ind+1 << endl;
					cout << "Connected Value:  " << dc_->del_list[p_ind][2] << endl;
					

				} else if(atv[ii] > keyf_threshold_*atv[cost_idx[num-1]]) {
					poseGraph->add(PharaoRotFactor(X(key_node), X(pose_count-num+ii+1), dc_->del_list[ii][2], odom_noise_model_));
					cout << "prev_key:  " << key_node << "    present_good:   " << pose_count-num + ii+1 << endl;
					cout << "Connected Value:  " << dc_->del_list[ii][2] << endl;
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

				// nav_msgs::Odometry opt_odom;
				// opt_odom.header.frame_id = "odom";
				// opt_odom.pose.pose.position.x = prev_pose.translation().x();
				// opt_odom.pose.pose.position.y = prev_pose.translation().y();
				// opt_odom.pose.pose.position.z = 0;
				// opt_odom.pose.pose.orientation.w = gtsam_quat.w();
				// opt_odom.pose.pose.orientation.x = gtsam_quat.x();
				// opt_odom.pose.pose.orientation.y = gtsam_quat.y();
				// opt_odom.pose.pose.orientation.z = gtsam_quat.z();
				// pub_opt_odom_.publish(opt_odom);

				key_node = pose_count - num + p_ind + 1;
				cout << "    updated_key:   " << key_node << endl;
				window_loop += num;
				prev_pose = odom_result.at<Pose2>(X(key_node));
				cout << "Covariance at keynode:\n" << isam2->marginalCovariance(X(key_node)) << endl;
				cout << "Factor Graph size:   " << odom_result.size() << endl;
			}

			cv::Mat last_p,last_c,last_cf;
			auto list_iter = dc_->window_list.begin() + 1 + p_ind;
			last_p = *list_iter;
			list_iter = dc_->window_list_cart.begin() + 1 + p_ind;
			last_c = *list_iter;
			list_iter = dc_->window_list_cart_f.begin() + 1 + p_ind;
			last_cf = *list_iter;

			dc_->keyf_list.push_back(last_p);
			dc_->keyf_list_cart.push_back(last_c);
			dc_->keyf_list_cart_f.push_back(last_cf);

			dc_->window_list.clear();
			dc_->window_list_cart.clear();
			dc_->window_list_cart_f.clear();

			dc_->window_list.push_back(last_p);
			dc_->window_list_cart.push_back(last_c);
			dc_->window_list_cart_f.push_back(last_cf);

			cnt++;
		}
	}

}