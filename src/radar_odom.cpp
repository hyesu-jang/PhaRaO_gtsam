#include <math.h>
#include <ros/ros.h>
#include "radar_odom.h"

radarOdom::radarOdom(ros::NodeHandle nh) : nh_(nh)
{
	nh_.getParam("is_polar_img", param_isPolarImg_);
	nh_.getParam("radar_range_bin", param_range_bin_);
	nh_.getParam("radar_angular_bin", param_ang_bin_);
	nh_.getParam("coarse_scale_factor", param_scale_);
	nh_.getParam("odom_factor_cost_threshold", odom_threshold_);
	nh_.getParam("keyframe_factor_cost_threshold", keyf_threshold_);

	width_ 		= floor((double) param_range_bin_ / (double) param_scale_);
	height_ 	= width_;
	p_width_ 	= param_range_bin_;
	p_height_ 	= param_ang_bin_;

    initialized = false;
    pub_opt_odom_ = nh.advertise<nav_msgs::Odometry>("/opt_odom", 1000);

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
	//initial_values.insert(K(pose_count), key_rot);
	//initial_values.insert(K(pose_count), prior_pose);
	auto prior_noise_model = noiseModel::Diagonal::Sigmas((Vector(3) << 0.01,0.01,0.001).finished());
    auto odom_noise_model = noiseModel::Diagonal::Sigmas(
        (Vector(3) << 1, 1, 1e-1)
            .finished());  // m, m, rad
	auto odom_noise_model_second = noiseModel::Diagonal::Sigmas((Vector(3) << 1, 1, 1e-1).finished());  // m, m, rad
	auto odom_noise_model_third = noiseModel::Diagonal::Sigmas((Vector(3) << 1, 1, 1e-1).finished());  // m, m, rad
	auto key_noise_model = noiseModel::Diagonal::Sigmas((Vector(3) << 1,1,1e-3).finished());
	auto rot_vector = (Vector(3) << 1000,1000,1e-2).finished();
    auto rot_noise_model = noiseModel::Diagonal::Sigmas(rot_vector);
    // Add prior factor to the graph.
    poseGraph->addPrior(X(pose_count), prior_pose, prior_noise_model);

	///////////////////////////////////////////
}



void radarOdom::callback(const sensor_msgs::ImageConstPtr& msg)
{
	static int cnt = 0;

	stamp = msg->header.stamp;
	stamp_list.push_back(stamp);

	ros::Time lasttime1=ros::Time::now();

	// Image preprocessing for phase correlation
	polar_mutex.lock();
		img = cv_bridge::toCvShare(msg, "mono8")->image;
    	// Convert to polar if input is cartesian
		//cv::Point2f center(img.cols / 2.0F, img.rows / 2.0F);
		//double maxRadius = cv::norm(cv::Point2f(img.cols - center.x, img.rows - center.y));
    	//cv::linearPolar(img, img, center, maxRadius, cv::WARP_FILL_OUTLIERS);
		// Extract nearest polar points
		//img = img.t();
		//img = polarToNearPol(img);
		//imshow("polar.",img);
		//toPointCloud(img, output_pcd, RESOL);
		img = img.t(); // for original radar
		//imshow("pol.",img);
	polar_mutex.unlock();

	boost::thread* thread_pc = new boost::thread(boost::bind(&radarOdom::phase_corr, this));
	boost::thread* thread_pcf = new boost::thread(boost::bind(&radarOdom::phase_corr_fine, this));

	thread_pc->join();
	thread_pcf->join();
	delete thread_pc;
	delete thread_pcf;

	ros::Time currtime1=ros::Time::now();

	// initialize
	if(initialized == false){
		// auto it = del_list.begin();
		FactorGeneration(0,1, del_list[0]);
		// FactorGeneration(0,1, *it);

		initialized = true;
	}

	imshow("Coarse cart.",*(window_list_cart.end()-1));


	bool onoff = OdomFactor();

	if(onoff) {
		// Keyframing
		KeyFraming();

		opt_odom.header.frame_id = "odom";
		pcd_radar.header.frame_id = "odom";
		pcd_radar.header.stamp = ros::Time::now();
		pub_opt_odom_.publish(opt_odom);
		//cout << pcd_radar << endl;
		// Graph optimization
		//GraphOptimize();
	}

    waitKey(1);
    cnt++;
}

inline cv::Mat radarOdom::log_polar(const cv::Mat img)
{
	cv::Mat log_polar_img;

	cv::Point2f center((float)img.cols/2, (float)img.rows/2);

	double radius = (double)img.rows / 2;

	double M = (double)img.cols / log(radius);

	cv::logPolar(img, log_polar_img, center, M, cv::INTER_LINEAR + cv::WARP_FILL_OUTLIERS);

	return log_polar_img;
}

cv::Mat radarOdom::polarToNearPol(const cv::Mat& polar_img) {
	
    int range = polar_img.rows;
    int theta = polar_img.cols;

    cv::Mat polar_ref = cv::Mat::zeros(range, theta, polar_img.type());
    polar_ref(cv::Range(0, 120), cv::Range::all()).copyTo(polar_img(cv::Range(0, 120), cv::Range::all()));

    for (int th = 0; th < theta; ++th) {
        int non_zero_count = 0;
        for (int r = 0; r < range; ++r) {
            if (polar_img.at<uchar>(r, th) != 0) {
                polar_ref.at<uchar>(r, th) = polar_img.at<uchar>(r, th);
                ++non_zero_count;
                if (non_zero_count == 100) {
                    break;
                }
            }
        }
    }
    return polar_ref;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr radarOdom::toPointCloud(const cv::Mat& radar_img, const double &range_resolution)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  const int nb_ranges = radar_img.cols;
  const int nb_azimuths = radar_img.rows;
  for(int azimuth_nb = 0; azimuth_nb < nb_azimuths; azimuth_nb++)
  {
    const double theta = ((double)(azimuth_nb + 1) / radar_img.rows) * 2. * M_PI;
    for(int range_bin = 0; range_bin < nb_ranges; range_bin++)
    {
      const double range = range_resolution * double(range_bin);
      const double intensity = (double)(radar_img.at<uchar>(azimuth_nb, range_bin));
      if(intensity > 0.)
      {
        pcl::PointXYZ p;
        p.x = range * std::cos(theta);
        p.y = range * std::sin(theta);
        //p.intensity = intensity;
        p.z = 0;
        cloud->push_back(p);
      }
    }
  }
  return cloud;
}


void radarOdom::phase_corr()
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

	// End Module
	//if(window_list.size() > NUM){
	//	window_list.erase(window_list.begin());
	//	window_list_cart.erase(window_list_cart.begin());
	//}

	window_list.push_back(resize_polar);
	window_list_cart.push_back(resize_cart);
}

void radarOdom::phase_corr_fine()
{
	cv::Mat radar_image_polar;
	cv::Mat radar_image_cart;

	int length = 500;

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

	////////////////////////////////////////////////////////////////////////////

	// End Module
	//if(window_list_cart_f.size() > NUM){
	//	window_list_cart_f.erase(window_list_cart_f.begin());
	//}

	window_list_cart_f.push_back(resize_cart);
}

array<double, 3>
radarOdom::PhaseCorr2D(cv::Mat r_src1, cv::Mat r_src2, cv::Mat src1, cv::Mat src2,
						 bool flag, array<double, 3> state)
{
	// flag == true : coarse, flag == false : fine
	double x,y,theta;
	double rotation;
	double phaseCorr;
	double factor;

	cv::Point2d peakLoc_r;

	if(flag == true) {
		peakLoc_r = cv::phaseCorrelate(r_src1, r_src2, cv::noArray(), &phaseCorr);
		rotation = (peakLoc_r.y - init_val[2])*(360.0/(r_src1.rows));
		theta = rotation*(M_PI/180.0);
	} else {
		theta = state.at(2);
		rotation = theta*180.0/M_PI;
	}

	cv::Point2f center(src2.cols/2.0, src2.rows/2.0);
	cv::Mat rot = cv::getRotationMatrix2D(center, rotation, 1.0);
	cv::Mat derot_cart;
	cv::warpAffine(src2, derot_cart, rot, src2.size());

	cv::Point2d peakLoc;
	if(flag == true) {
		peakLoc = cv::phaseCorrelate(src1, derot_cart, cv::noArray(), &phaseCorr);
	
		factor = ratio;
		x = (init_val[0]-peakLoc.x)*RESOL*factor;
		y = (init_val[1]-peakLoc.y)*RESOL*factor;
	} else {
    	int pixel_x = round(state.at(0)/(ratio * RESOL));
    	int pixel_y = round(state.at(1)/(ratio * RESOL));

    	if( abs(pixel_x) > 500 || abs(pixel_y) > 500){
			x = state.at(0);
			y = state.at(1);
		} else {
			peakLoc = itf_f.phaseCorrelateWindow(src1, derot_cart, cv::noArray(), &phaseCorr, state);

			factor = 1.0;
			x = state.at(0) + (init_val_f[0]-peakLoc.x)*RESOL*factor;
			y = state.at(1) + (init_val_f[1]-peakLoc.y)*RESOL*factor;
		}
	}

	if(initialized == false) {
		if(flag == true) {
			init_val[0] = peakLoc.x;
			init_val[1] = peakLoc.y;
			init_val[2] = peakLoc_r.y;
		} else {
			init_val_f[0] = peakLoc.x;
			init_val_f[1] = peakLoc.y;
			init_val_f[2] = peakLoc_r.y;
		}

		theta = 0;
		x = 0;
		y = 0;
	}

	array<double, 3> d_state = {x, y, theta};
	return d_state;
}

void
radarOdom::FactorGeneration(int src1, int src2, array<double, 3>& out_state)
{
	// Coarse Phase Correlation Module
	auto begin_iter = window_list.begin();
	auto begin_iter_cart = window_list_cart.begin();

	array<double, 3> state = {0,0,0};
	array<double, 3> cd_state = PhaseCorr2D(*(begin_iter+src1), *(begin_iter+src2),
									*(begin_iter_cart+src1), *(begin_iter_cart+src2), true, state);

	// Fine Phase Correlation Module
	begin_iter_cart = window_list_cart_f.begin();

	array<double, 3> fd_state = PhaseCorr2D(*(begin_iter+src1), *(begin_iter+src2),
									*(begin_iter_cart+src1), *(begin_iter_cart+src2), false, cd_state);

	polar_mutex.lock();
		std::copy(fd_state.begin(), fd_state.end(), out_state.begin());
	polar_mutex.unlock();
}


bool
radarOdom::OdomFactor()
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

		if(norm < RESOL){
			window_list.erase(window_list.end()-1);
			window_list_cart.erase(window_list_cart.end()-1);
			window_list_cart_f.erase(window_list_cart_f.end()-1);
			num--;
			stamp_list.erase(stamp_list.end()-1);

			return false;
		} else {
			if(cost > 0.85) {

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

void
radarOdom::KeyFraming()
{
	static int cnt = 0;
	int num = window_list.size()-1;

	// KeyFrame factor
	//for (int numnum = 1; numnum <= num;numnum++)
	//	FactorGeneration(0,numnum,del_list[numnum-1]);
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
    //cout << endl;

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
		if( ((atv[num-1] < atv[num-2]) && (atv[num-1] < 0.9*atv[cost_idx[num-1]])) || num > 3 || norm_v[0] > 30.0) {
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
					//poseGraph->add(PharaoRotFactor(X(key_node), X(key_node+p_ind+1), -del_list[p_ind][2], rot_noise_model));
					
					poseGraph->add(PharaoRotFactor(X(key_node), X(pose_count-num+p_ind+1), del_list[p_ind][2], rot_noise_model));
					//Pose2 new_odom = gtsam::Pose2(del_list[p_ind][0], del_list[p_ind][1], del_list[p_ind][2]);
					//poseGraph->add(BetweenFactor<Pose2>(X(key_node), X(pose_count-num+p_ind+1), new_odom, odom_noise_model));
					cout << "prev_key:  " << key_node << "    present_key:   " << pose_count-num + p_ind+1 << endl;
					cout << "Connected Value:  " << del_list[p_ind][2] << endl;
					

				} else if(atv[ii] > 0.9*atv[cost_idx[num-1]]) {
					//Eigen::Vector1d rel_theta(-del_list[ii][2]);
					//poseGraph->add(PharaoRotFactor(X(key_node), X(key_node+ii+1), -del_list[ii][2], rot_noise_model));
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
			
				//cout << "idx : " << idx << "," << "p_size : " << p_size << endl;
				cout << "p_size : " << p_size << endl;
				/////////////////////////////////////////////////////////

				
				//prev_pose = odom_result.at<Pose2>(X(pose_count));
				prev_pose = odom_result.at<Pose2>(X(key_node));
				cout << "Last Pose value:\n     x:" << prev_pose.translation().x() << "     y:"<< prev_pose.translation().y() << "       theta:"<< prev_pose.rotation().theta()<<endl;
				Eigen::AngleAxisd rollAngle(0.0, Eigen::Vector3d::UnitX());   //M_PI
				Eigen::AngleAxisd pitchAngle(0.0, Eigen::Vector3d::UnitY());
				Eigen::AngleAxisd yawAngle(prev_pose.rotation().theta(), Eigen::Vector3d::UnitZ());
				Eigen::Quaternion<double> gtsam_quat = yawAngle * pitchAngle * rollAngle;

				
				opt_odom.pose.pose.position.x = prev_pose.translation().x();
				opt_odom.pose.pose.position.y = prev_pose.translation().y();
				opt_odom.pose.pose.position.z = 0;
				opt_odom.pose.pose.orientation.w = gtsam_quat.w();
				opt_odom.pose.pose.orientation.x = gtsam_quat.x();
				opt_odom.pose.pose.orientation.y = gtsam_quat.y();
				opt_odom.pose.pose.orientation.z = gtsam_quat.z();

				pcl::PointCloud<pcl::PointXYZ>::Ptr output_pcd = toPointCloud(img, RESOL);

				Eigen::Matrix3f R = createRotationMatrix(prev_pose.translation().x(), prev_pose.translation().y(), prev_pose.rotation().theta());

        		transformPointCloud(R, output_pcd);


				tf2::Quaternion mQ;
				mQ.setRPY( 0, 0, prev_pose.rotation().theta());
				writeFile.open("test.txt", ios::app);
					if(writeFile.is_open()) {
						writeFile << stamp << ' ' << prev_pose.translation().x() << ' ' << prev_pose.translation().y() << " 0 " << 
									mQ[0] << ' ' << mQ[1] << ' ' << mQ[2] << ' ' << mQ[3] << endl;
						writeFile.close();
					}
				
				
				//key_node = window_loop + p_ind +1;
				key_node = pose_count - num + p_ind + 1;
				cout << "    updated_key:   " << key_node << endl;
				window_loop += num;
				prev_pose = odom_result.at<Pose2>(X(key_node));
				//poseGraph->add(BetweenFactor<Pose2>(X(0), X(key_node), prev_pose, prior_noise_model));
				//poseGraph->addPrior(X(key_node), prev_pose, prior_noise_model);
				//Marginals marginals(*poseGraph, odom_result);
				//Matrix marginalss = isam2->marginalCovariance(X(key_node));
				cout << "Covariance at keynode:\n" << isam2->marginalCovariance(X(key_node)) << endl;
				cout << "Factor Graph size:   " << odom_result.size() << endl;
				//poseGraph->addPrior(K(key_node), Rot2(prev_pose.rotation().theta()), rot_noise_model);
				//poseGraph->add(BetweenFactor<Pose2>(X(key_node),K(key_node), Pose2(0.0,0.0,0.0), odom_noise_model));
				//poseGraph->addPrior(X(key_node), prev_pose, odom_noise_model);
			}
			/*
			for(int ii = 0; ii < p_ind+1; ii++) {
				pose_node_nums.erase(pose_node_nums.begin());
			}
			for(int ii = p_ind+2; ii < p_size; ii++) {
				pose_node_nums.erase(pose_node_nums.end()-1);
			}
			//prev_pose = odom_result.at<Pose2>(X(key_node));
			
			poseGraph->resize(0);
			initial_values.clear();
			pose_count = 1;
			poseGraph->addPrior(X(*(pose_node_nums.begin())), prev_pose, odom_noise_model);
			*/
			//poseGraph->addPrior(X(key_node), prev_pose, prior_noise_model);

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

void
radarOdom::GraphOptimize()
{
	/*
	static int cnt = 0;
	int num = keyf_list.size()-1;

	vector<double> atv(num);
	vector<int> cost_idx(num);
	double norm_v[num];
	double norm_w[num];

	cout << "del atv : ";
	for(int ii = 0; ii < num; ii++) {
		double d_yx = atan2(var_list[ii][1],var_list[ii][0]);
		double d_theta = var_list[ii][2];
		norm_v[ii]=sqrt(var_list[ii][1]*var_list[ii][1]+var_list[ii][0]*var_list[ii][0]);
		norm_w[ii]=d_theta*180.0/M_PI;

	    atv[ii] = exp(-abs(d_yx + d_theta));

	    if(norm_v[0] > 1.0){
	        if(abs(var_list[ii][1]) > 1.5)
	        	atv[ii] = 0;
	    	if(abs(norm_w[ii]) > 30.0)
	    		atv[ii] = 0;
		}
		cout << atv[ii] << " ";
	}
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
    cout << endl;
    /////////////////////////////////////////////////////////



	Pose2d_Node* new_pose_node = new Pose2d_Node();
	slam.add_node(new_pose_node);
	pose_nodes.push_back(new_pose_node);

	if(pose_nodes.size() > NUM){
		pose_nodes.erase(pose_nodes.begin());
		slam.remove_node(*(pose_nodes.begin()));

		Pose2d_Node* b_pose = *(pose_nodes.begin());
		Vector3d new_pose = b_pose->value().vector();

		Pose2d new_origin(new_pose(0), new_pose(1), new_pose(2));
		Pose2d_Factor* prior = new Pose2d_Factor(b_pose, new_origin, p3);
		// add it to the graph
		slam.add_factor(prior);
	}

	// connect to previous with odometry measurement
	auto end_iter = pose_nodes.end() - 1;

	for(int ii = 0; ii < num; ii++) {
		//if(atv[ii] > 0.9*atv[cost_idx[num-1]] || ii == 0) {
		if(ii == 0){
			Pose2d odometry(var_list[ii][0], var_list[ii][1], var_list[ii][2]); // x,y,theta
			Pose2d_Pose2d_Factor* constraint = new Pose2d_Pose2d_Factor(*(end_iter-1-ii), *end_iter, odometry, noise3);
			slam.add_factor(constraint);
		} else if (atv[ii] > 0.9) {
			Pose2d odometry(var_list[ii][0], var_list[ii][1], var_list[ii][2]); // x,y,theta
			Pose2d_Pose2d_Factor* constraint = new Pose2d_Pose2d_Factor(*(end_iter-1-ii), *end_iter, odometry, f3);
			slam.add_factor(constraint);
		}
	}

	//if(cnt % NUM == 0) {
		slam.update();
		slam.batch_optimization();
	//}

	Pose2d_Node* f_pose = *(pose_nodes.end()-1);
	//Pose2d_Node* f_pose = *(pose_nodes.begin());
	auto pose = f_pose->value().vector();
	tf2::Quaternion mQ;
	mQ.setRPY( 0, 0, pose(2) );

	writeFile.open("test.txt", ios::app);
	if(writeFile.is_open()) {
		writeFile << stamp << ' ' << pose(0) << ' ' << pose(1) << " 0 " << 
					mQ[0] << ' ' << mQ[1] << ' ' << mQ[2] << ' ' << mQ[3] << endl;
		writeFile.close();
	}

	exec = false;
	cnt++;
	*/
}



void radarOdom::fftshift(const Mat& inputImg, Mat& outputImg)
{
    outputImg = inputImg.clone();
    int cx = outputImg.cols / 2;
    int cy = outputImg.rows / 2;
    Mat q0(outputImg, Rect(0, 0, cx, cy));
    Mat q1(outputImg, Rect(cx, 0, cx, cy));
    Mat q2(outputImg, Rect(0, cy, cx, cy));
    Mat q3(outputImg, Rect(cx, cy, cx, cy));
    Mat tmp;
    q0.copyTo(tmp);
    q3.copyTo(q0);
    tmp.copyTo(q3);
    q1.copyTo(tmp);
    q2.copyTo(q1);
    tmp.copyTo(q2);
}

Eigen::Matrix3f radarOdom::createRotationMatrix(double x, double y, double ez)
{
    Eigen::Matrix3f R;

    float cos_theta = std::cos(ez);
    float sin_theta = std::sin(ez);

    R << cos_theta, -sin_theta, x,
         sin_theta,  cos_theta, y,
         0,          0,         1;

    return R;
}


void radarOdom::transformPointCloud(const Eigen::Matrix3f& R, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    for (auto& point : cloud->points)
    {
        Eigen::Vector3f p(point.x, point.y, 1);
        Eigen::Vector3f transformed_point = R * p;

        point.x = transformed_point.x();
        point.y = transformed_point.y();
    }
}

void radarOdom::converToPolar()
{
	// Convert to polar if input is cartesian
	cv::Point2f center(img.cols / 2.0F, img.rows / 2.0F);
	double maxRadius = cv::norm(cv::Point2f(img.cols - center.x, img.rows - center.y));
	cv::linearPolar(img, img, center, maxRadius, cv::WARP_FILL_OUTLIERS);
	// Extract nearest polar points
	img = img.t();
	img = polarToNearPol(img);
	imshow("polar.",img);
}