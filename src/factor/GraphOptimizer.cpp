#include <factor/GraphOptimizer.hpp>

GraphOptimizer::GraphOptimizer()
{
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

	auto prior_noise_model = noiseModel::Diagonal::Sigmas((Vector(3) << 0.01, 0.01, 0.001).finished());
	auto odom_noise_model = noiseModel::Diagonal::Sigmas((Vector(3) << 1, 1, 1e-1).finished());  // m, m, rad
	auto odom_noise_model_second = noiseModel::Diagonal::Sigmas((Vector(3) << 1, 1, 1e-1).finished());  // m, m, rad
	auto odom_noise_model_third = noiseModel::Diagonal::Sigmas((Vector(3) << 1, 1, 1e-1).finished());  // m, m, rad
	auto key_noise_model = noiseModel::Diagonal::Sigmas((Vector(3) << 1,1,1e-3).finished());
	auto rot_vector = (Vector(3) << 1000,1000,1e-2).finished();
	auto rot_noise_model = noiseModel::Diagonal::Sigmas(rot_vector);

	// Add prior factor to the graph.
	poseGraph->addPrior(X(pose_count), prior_pose, prior_noise_model);
}

GraphOptimizer::~GraphOptimizer()
{
    
}

void
GraphOptimizer::setInputLists(std::vector<cv::Mat>* ptr_window_list,
                                std::vector<cv::Mat>* ptr_window_list_cart,
                                std::vector<cv::Mat>* ptr_window_list_cart_f,
                                std::vector<cv::Mat>* ptr_keyf_list,
                                std::vector<cv::Mat>* ptr_keyf_list_cart,
                                std::vector<cv::Mat>* ptr_keyf_list_cart_f)
{
    ptr_window_list_ = ptr_window_list;
    ptr_window_list_cart_ = ptr_window_list_cart;
    ptr_window_list_cart_f_ = ptr_window_list_cart_f;
    ptr_keyf_list_ = ptr_keyf_list;
    ptr_keyf_list_cart_ = ptr_keyf_list_cart;
    ptr_keyf_list_cart_f_ = ptr_keyf_list_cart_f;
}

void
GraphOptimizer::optimize()
{
    AbstractFactor ft(ptr_window_list_, ptr_window_list_cart_, ptr_window_list_cart_f_,
                ptr_keyf_list_, ptr_keyf_list_cart_, ptr_keyf_list_cart_f_);
	// // initialize
	// if(initialized == false){
	// 	FactorGeneration(0,1, del_list[0]);

	// 	initialized = true;
	// }

	// imshow("Coarse cart.",*(window_list_cart.end()-1));

	// bool onoff = OdomFactor();

	// if(onoff) {
	// 	// Keyframing
	// 	KeyFraming();
	// }


}