#include <PhaRaO.hpp>

PhaRaO::PhaRaO(ros::NodeHandle nh) : nh_(nh)
{
	nh_.getParam("is_polar_img", param_isPolarImg_);
	nh_.getParam("radar_range_bin", param_range_bin_);
	nh_.getParam("radar_angular_bin", param_ang_bin_);

	nh_.getParam("coarse_scale_factor", param_scale_);
	nh_.getParam("sub_img_size", param_sub_);

	width_ 		= floor((double) param_range_bin_ / (double) param_scale_);
	height_ 	= width_;
	p_width_ 	= param_range_bin_;
	p_height_ 	= param_ang_bin_;

	ddc_.initialized = false;
	dc_ = &ddc_;
	go_ = new GraphOptimizer(nh_, dc_);
}

PhaRaO::~PhaRaO()
{

}

void
PhaRaO::callback(const sensor_msgs::ImageConstPtr& msg)
{
	ros::Time stamp = msg->header.stamp;
	dc_->stamp_list.push_back(stamp);

	// Image preprocessing for phase correlation
	cv::Mat img = cv_bridge::toCvShare(msg, "mono8")->image;
	
	// Convert to polar if input is cartesian
	if(!param_isPolarImg_)
		img = convertToPolar(img);

	img = img.t();

	boost::thread* thread_pc = new boost::thread(&PhaRaO::preprocess_coarse, this, img);
	boost::thread* thread_pcf = new boost::thread(&PhaRaO::preprocess_fine, this, img);

	thread_pc->join();
	thread_pcf->join();
	delete thread_pc;
	delete thread_pcf;

	go_->optimize();

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

	waitKey(1);

}

void
PhaRaO::preprocess_coarse(cv::Mat img)
{
	cv::Mat radar_image_polar;
	cv::Mat radar_image_cart;

	// Image Downsampling and Polar to Cartesian Module
	img.convertTo(radar_image_polar, CV_32FC1, 1.0/255.0);

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
	ArrayXXf filter = itf.highpassfilt(magI.size(), dc_->initialized);
	MatrixXf filter_mt = filter.matrix();
	cv::Mat filter_cv;
	eigen2cv(filter_mt, filter_cv);

	cv::Mat filt_FFT = filter_cv.mul(magI);

	// Log-Polar Module
	cv::Mat resize_polar = log_polar(filt_FFT);	

	// Save preprocessed images
	dc_->window_list.push_back(resize_polar);
	dc_->window_list_cart.push_back(resize_cart);

	if(dc_->initialized == false) {
		dc_->keyf_list.push_back(resize_polar);
		dc_->keyf_list_cart.push_back(resize_cart);
	}

	////////////////////////////////////////////////////////////////////////////
}

void
PhaRaO::preprocess_fine(cv::Mat img)
{
	cv::Mat radar_image_polar;
	cv::Mat radar_image_cart;

	int length = param_sub_;

	// Image Downsampling and Polar to Cartesian Module
	img(cv::Rect(0, 0, length, p_height_)).convertTo(radar_image_polar, CV_32FC1, 1.0/255.0);

	cv::Mat resize_cart;
	itf_f.warpPolar(radar_image_polar, resize_cart, Size( length*2,length*2 ),
				Point2f( length,length ), length, CV_INTER_AREA | CV_WARP_INVERSE_MAP);

	// Save preprocessed images
	dc_->window_list_cart_f.push_back(resize_cart);
	
	if(dc_->initialized == false) {
		dc_->keyf_list_cart_f.push_back(resize_cart);
	}

}