#pragma once

#include <vector>
#include <array>
#include <string>
#include <algorithm>
#include <numeric>

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>

#include <eigen_conversions/eigen_msg.h>

#include <ros/ros.h>

#include <tf2/LinearMath/Quaternion.h>
#include <cv_bridge/cv_bridge.h>

#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

#include <factor/GraphOptimizer.hpp>

using namespace std;
using namespace Eigen;



class PhaRaO
{
	public:

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		PhaRaO(ros::NodeHandle nh);
		~PhaRaO();
		void callback(const sensor_msgs::ImageConstPtr&);

	private:
		void preprocess_coarse(cv::Mat img);
		void preprocess_fine(cv::Mat img);

	private:

		ros::NodeHandle nh_;

		bool param_isPolarImg_ = true;
		int param_range_bin_ = 3360;
		int param_ang_bin_ = 400;
		int param_scale_ = 10;
		int param_sub_ = 500;

		int width_, height_;
		int p_width_, p_height_;

		DataContainer ddc_;
		DataContainer* dc_;
		GraphOptimizer* go_;
		ImageTF itf, itf_f;
				
		int length;
		
		fftModule fftM;
		fftModule fftM_f;

		double prev_v = 0.0;
		double prev_w = 0.0;
		bool exec = false;

};
