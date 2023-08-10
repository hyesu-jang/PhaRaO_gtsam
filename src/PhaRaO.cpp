#include <PhaRaO.hpp>

PhaRaO::PhaRaO(ros::NodeHandle nh) : nh_(nh)
{
	nh_.getParam("is_polar_img", param_isPolarImg_);
	nh_.getParam("radar_range_bin", param_range_bin_);
	nh_.getParam("radar_angular_bin", param_ang_bin_);

	nh_.getParam("coarse_scale_factor", param_scale_);
	nh_.getParam("sub_img_size", param_sub_);
	nh_.getParam("odom_factor_cost_threshold", odom_threshold_);
	nh_.getParam("keyframe_factor_cost_threshold", keyf_threshold_);

	width_ 		= floor((double) param_range_bin_ / (double) param_scale_);
	height_ 	= width_;
	p_width_ 	= param_range_bin_;
	p_height_ 	= param_ang_bin_;

	initialized = false;
	pub_opt_odom_ 	= nh.advertise<nav_msgs::Odometry>("/opt_odom", 1000);
	pub_odom_ 		= nh.advertise<nav_msgs::Odometry>("/odom", 1000);

	//////////////////GTSAM////////////////////
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

	///////////////////////////////////////////
}

void
PhaRaO::callback(const sensor_msgs::ImageConstPtr& msg)
{
	static int cnt = 0;

	stamp = msg->header.stamp;
	stamp_list.push_back(stamp);

	cv::Mat img;

	// Image preprocessing for phase correlation
	polar_mutex.lock();
		img = cv_bridge::toCvShare(msg, "mono8")->image;
		
		// Convert to polar if input is cartesian
		if(!param_isPolarImg_)
			img = convertToPolar(img);

		img = img.t();
	polar_mutex.unlock();

	boost::thread* thread_pc = new boost::thread(boost::bind(&radarOdom::preprocess_coarse, this, _1), img);
	boost::thread* thread_pcf = new boost::thread(boost::bind(&radarOdom::preprocess_fine, this, _1), img);

	thread_pc->join();
	thread_pcf->join();
	delete thread_pc;
	delete thread_pcf;

	// initialize
	if(initialized == false){
		FactorGeneration(0,1, del_list[0]);

		initialized = true;
	}

	imshow("Coarse cart.",*(window_list_cart.end()-1));

	bool onoff = OdomFactor();

	if(onoff) {
		// Keyframing
		KeyFraming();
	}

    waitKey(1);
    cnt++;
}

void
PhaRaO::preprocess_coarse(cv::Mat img)
{
	cv::Mat radar_image_polar;
	cv::Mat radar_image_cart;

	// Image Downsampling and Polar to Cartesian Module
	polar_mutex.lock();
		img.convertTo(radar_image_polar, CV_32FC1, 1.0/255.0);
	polar_mutex.unlock();

	cv::Mat polar;
	cv::resize(radar_image_polar, polar, Size(width_, p_height_), 0, 0, CV_INTER_NN);	

	cv::Mat resize_cart;
	itf.warpPolar(polar, resize_cart, Size( width_*2,height_*2 ),
				Point2f( width_,height_ ), width_, CV_INTER_AREA | CV_WARP_INVERSE_MAP);

	////////////////////////////////////////////////////////////////////////////
	// FFT Module (OpenCV)

	cv::Mat padded;
	int m = getOptimalDFTSize( resize_cart.rows );
	int n = getOptimalDFTSize( resize_cart.cols );
	copyMakeBorder(resize_cart, padded, 0, m-resize_cart.rows, 0, n-resize_cart.cols, BORDER_CONSTANT, Scalar::all(0));
	Mat planes[] = {Mat_<float>(padded), Mat::zeros(padded.size(), CV_32F)};
	Mat complexI;
	merge(planes, 2, complexI);         // Add to the expanded another plane with zeros

	dft(complexI, complexI);            // this way the result may fit in the source matrix

	// compute the magnitude and switch to logarithmic scale
	// => log(1 + sqrt(Re(DFT(I))^2 + Im(DFT(I))^2))
	split(complexI, planes);                   // planes[0] = Re(DFT(I)), planes[1] = Im(DFT(I))

	magnitude(planes[0], planes[1], planes[0]);// planes[0] = magnitude
	Mat magI = planes[0];

	log(magI, magI);

	itf.fftShift(magI);

	////////////////////////////////////////////////////////////////////////////

	// HPF Module
	ArrayXXf filter = itf.highpassfilt(magI.size(), initialized);
	MatrixXf filter_mt = filter.matrix();
	cv::Mat filter_cv;
	eigen2cv(filter_mt, filter_cv);

	cv::Mat filt_FFT = filter_cv.mul(magI);

	// Log-Polar Module
	cv::Mat resize_polar = log_polar(filt_FFT);	

	if(initialized == false) {
		window_list.push_back(resize_polar);
		window_list_cart.push_back(resize_cart);
		keyf_list.push_back(resize_polar);
		keyf_list_cart.push_back(resize_cart);
	}

	////////////////////////////////////////////////////////////////////////////

	window_list.push_back(resize_polar);
	window_list_cart.push_back(resize_cart);
}

void
PhaRaO::preprocess_fine(cv::Mat img)
{
	cv::Mat radar_image_polar;
	cv::Mat radar_image_cart;

	int length = param_sub_;

	// Image Downsampling and Polar to Cartesian Module
	polar_mutex.lock();
		img(cv::Rect(0, 0, length, p_height_)).convertTo(radar_image_polar, CV_32FC1, 1.0/255.0);
	polar_mutex.unlock();

	cv::Mat resize_cart;
	itf_f.warpPolar(radar_image_polar, resize_cart, Size( length*2,length*2 ),
				Point2f( length,length ), length, CV_INTER_AREA | CV_WARP_INVERSE_MAP);

	if(initialized == false) {
		window_list_cart_f.push_back(resize_cart);
		keyf_list_cart_f.push_back(resize_cart);
	}

	window_list_cart_f.push_back(resize_cart);
}