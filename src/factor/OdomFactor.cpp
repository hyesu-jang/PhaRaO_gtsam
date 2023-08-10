#include <factor/OdomFactor.hpp>

OdomFactor::OdomFactor()
{

}

OdomFactor::~OdomFactor()
{

}

bool
OdomFactor::factorGeneration()
{
	// Phase correlation (coarse to fine)
	int num = window_list.size() - 1;

	int index = 0;

	// Odom. Frame factor
	for(int ii = 1; ii <= num; ii++) {
		int begin = num-ii;
		int end = num;
		FactorGeneration(begin, end, odom_list[num-1]);

		double o_yx = atan2(odom_list[num-1][1], odom_list[num-1][0]);
		double o_theta = odom_list[num-1][2];
		double cost = exp(-abs(o_yx + o_theta));
		double norm = sqrt(pow(odom_list[num-1][1],2) + pow(odom_list[num-1][0],2));
		cout << "filtering : " << cost << ", " << o_theta/M_PI*180.0 << ", " << o_yx << endl;

		if(norm < RESOL){		// Too close to caluculate odom
			window_list.erase(window_list.end()-1);
			window_list_cart.erase(window_list_cart.end()-1);
			window_list_cart_f.erase(window_list_cart_f.end()-1);
			num--;
			stamp_list.erase(stamp_list.end()-1);

			return false;
		} else {
			if(cost > odom_threshold_) {

				Pose2 new_odom = gtsam::Pose2(odom_list[num-1][0], odom_list[num-1][1], odom_list[num-1][2]); // x,y,theta
				Vector3 egovec(odom_list[num-1][0], odom_list[num-1][1], odom_list[num-1][2]);
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
				poseGraph->add(BetweenFactor<Pose2>(X(pose_count-ii), X(pose_count), new_odom, odom_noise_model));
				
				cout << "Connected Value:  " << odom_list[num-1][2] << endl;
				initial_values.insert(X(pose_count), prop_pose);

				nav_msgs::Odometry odom;
				odom.header.frame_id = "odom";
				odom.pose.pose.position.x = new_odom.translation().x();
				odom.pose.pose.position.y = new_odom.translation().y();
				odom.pose.pose.position.z = 0;
				odom.pose.pose.orientation.w = 1;
				odom.pose.pose.orientation.x = 0;
				odom.pose.pose.orientation.y = 0;
				odom.pose.pose.orientation.z = 0;
				pub_odom_.publish(odom);
				return true;
			}

		}
	}

	window_list.erase(window_list.end()-1);
	window_list_cart.erase(window_list_cart.end()-1);
	window_list_cart_f.erase(window_list_cart_f.end()-1);
	num--;
	stamp_list.erase(stamp_list.end()-1);

	return false;
}
