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
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

#include <factor/GraphOptimizer.hpp>

using namespace std;
using namespace Eigen;

#define NUM 5

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
		ros::Publisher pub_opt_odom_;
		ros::Publisher pub_odom_;

		bool param_isPolarImg_;
		int param_range_bin_;
		int param_ang_bin_;
		int param_scale_;
		int param_sub_;

		double odom_threshold_;
		double keyf_threshold_;



		int width_, height_;
		int p_width_, p_height_;


		GraphOptimizer go_;
		ImageTF itf_temp;
		
		sensor_msgs::PointCloud2 pcd_radar;
		geometry_msgs::Pose2D radar_ego;
		

		boost::mutex polar_mutex;




		int length;

		

		bool initialized;

		ofstream writeFile;
		fftModule fftM;
		fftModule fftM_f;

		vector<cv::Mat> window_list_;
		vector<cv::Mat> window_list_cart_;
		vector<cv::Mat> window_list_cart_f_;

		vector<cv::Mat> keyf_list_;
		vector<cv::Mat> keyf_list_cart_;
		vector<cv::Mat> keyf_list_cart_f_;

		vector<ros::Time> stamp_list;
		vector<ros::Time> sstamp_list;
		vector<ros::Time> kstamp_list;

		array<array<double, 3>, NUM> var_list;
		array<array<double, 3>, NUM> odom_list;

		array<int, NUM> cost_idx;
		array<int, NUM> cost_iter;
		array<double, NUM> atv;
		double norm_v[NUM];
		double norm_w[NUM];



		ros::Time stamp;


		double prev_v = 0.0;
		double prev_w = 0.0;
		bool exec = false;

};
