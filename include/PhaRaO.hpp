#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>

#include <vector>
#include <array>
#include <string>
#include <algorithm>
#include <numeric>

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <eigen_conversions/eigen_msg.h>

#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>

#include "ImageTF.hpp"
#include "fftModule.hpp"
#include <hs_rotation_factor.h>

#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

using namespace std;
using namespace Eigen;
using namespace gtsam;

using symbol_shorthand::X; // Pose2 (x,y,theta)
using symbol_shorthand::K; // Rot2 (theta)

#define NUM 5

class radarOdom
{
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    radarOdom(ros::NodeHandle);

    void callback(const sensor_msgs::ImageConstPtr&);

private:
	
    inline cv::Mat log_polar(const cv::Mat);
	cv::Mat polarToNearPol(const cv::Mat&);

	void phase_corr(cv::Mat img);
	void phase_corr_fine(cv::Mat img);
	array<double, 3> PhaseCorr2D(cv::Mat r_src1, cv::Mat r_src2, cv::Mat src1,
								 cv::Mat src2, bool flag, array<double, 3> state);
	void FactorGeneration(int src1, int src2, array<double, 3>& out_state);
	bool OdomFactor();
	void KeyFraming();
	void GraphOptimize();

	void fftshift(const Mat& inputImg, Mat& outputImg);
	Eigen::Matrix3f createRotationMatrix(double x, double y, double ez);
	void transformPointCloud(const Eigen::Matrix3f& R, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
	cv::Mat convertToPolar(cv::Mat img);

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

	
	sensor_msgs::PointCloud2 pcd_radar;
	geometry_msgs::Pose2D radar_ego;
	Rot2 key_rot, orien;

	boost::mutex polar_mutex;

	const double RESOL = 0.059612233;
	//const double RESOL = 2733/1024;


	int length;

    

    bool initialized;

	ofstream writeFile;
	ImageTF itf;
	ImageTF itf_f;
	fftModule fftM;
	fftModule fftM_f;

	double init_val[3] = {0,};
	double init_val_f[3] = {0,};

	vector<cv::Mat> window_list;
	vector<cv::Mat> window_list_cart;
	vector<cv::Mat> window_list_cart_f;

	vector<cv::Mat> keyf_list;
	vector<cv::Mat> keyf_list_cart;
	vector<cv::Mat> keyf_list_cart_f;

	vector<ros::Time> stamp_list;
	vector<ros::Time> sstamp_list;
	vector<ros::Time> kstamp_list;

	array<array<double, 3>, NUM> del_list;
	array<array<double, 3>, NUM> var_list;
	array<array<double, 3>, NUM> odom_list;

	array<int, NUM> cost_idx;
	array<int, NUM> cost_iter;
	array<double, NUM> atv;
	double norm_v[NUM];
	double norm_w[NUM];

	ISAM2 *isam2;
	//ISAM2Params parameters;
	GaussNewtonParams parameters;
	Values initial_values;
	Values odom_result;
	Vector2 trans_ ;
    NonlinearFactorGraph* poseGraph = new NonlinearFactorGraph();
    Pose2 prev_pose, prop_pose;
	noiseModel::Diagonal::shared_ptr prior_noise_model;
	noiseModel::Diagonal::shared_ptr odom_noise_model;
	noiseModel::Diagonal::shared_ptr odom_noise_model_second;
	noiseModel::Diagonal::shared_ptr odom_noise_model_third;
	noiseModel::Diagonal::shared_ptr key_noise_model;
	noiseModel::Diagonal::shared_ptr key_noise_model2;
	noiseModel::Diagonal::shared_ptr rot_noise_model;
	int pose_count = 0;
	int frame_step = 3;
	bool correlation_found = false;
	int key_node = 0;
	int window_loop =0;
	vector<int> pose_node_nums;
	Vector3 now_pose;
	Vector3 base_pose;
	vector<Vector3> pose_values;
	//vector<Pose2d_Node*> pose_nodes;
	ISAM2 *g_isam;
	//Slam* g_slam;
	//vector<Pose2d_Node*> g_pose_nodes;
	Matrix3d m = (Eigen::Matrix3d() << pow(1.0,2), 0, 0, 0, pow(1.0,2), 0, 0, 0, pow(0.2,2)).finished();
	Matrix3d n = (Eigen::Matrix3d() << pow(1.0,2), 0, 0, 0, pow(1.0,2), 0, 0, 0, pow(1.0,2)).finished();
/*
	Noise f3 = Information(pow(1.0,2) * m);
	Noise noise3 = Information(pow(1.0,2) * n);
	Noise p3 = Information(pow(10,12) * eye(3));
	Noise kf1 = Information(pow(5.0,2)*eye(1));
	Noise f1 = Information(pow(2.5,2)*eye(1));
*/
	ImageTF itf_temp;
	ros::Time stamp;

	double ratio = 10.0;	// seq03_kaist : 14.0, others : 10.0
	double prev_v = 0.0;
	double prev_w = 0.0;
	bool exec = false;

};
