#include <factor/OdomFactor.hpp>

OdomFactor::OdomFactor(DataContainer* dc, double threshold)
					: dc_(dc), threshold_(threshold), AbstractFactor(dc)
{
	
}

OdomFactor::~OdomFactor()
{

}

std::tuple<bool, std::array<double, 3>, 
OdomFactor::calcOdom(std::array<double, 3> )
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
			dc_->stamp_list.erase(stamp_list.end()-1);

			return false;
		} else {
			if(cost > threshold_) {

				Pose2 new_odom = gtsam::Pose2(dc_->odom_list[num-1][0], dc_->odom_list[num-1][1], dc_->odom_list[num-1][2]); // x,y,theta
				poseGraph->add(BetweenFactor<Pose2>(X(pose_count-ii), X(pose_count), new_odom, odom_noise_model_));

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
