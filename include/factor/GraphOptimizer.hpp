#pragma once

#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>

#include <factor/FactorConstructor.hpp>
#include <gtsam_custom/hs_rotation_factor.h>

using namespace gtsam;
using symbol_shorthand::X; // Pose2 (x,y,theta)
using symbol_shorthand::K; // Rot2 (theta)

class GraphOptimizer : public FactorConstructor
{
	public:
		GraphOptimizer(ros::NodeHandle nh, DataContainer* dc);
		~GraphOptimizer();
		bool generateOdomFactor();
		void regenerateOdomFactor();
		void generateKeyfFactor();
		void optimize();

	protected:
		void publishOdom(Pose2 pose, Eigen::Quaterniond quat);
		void publishOptOdom(Pose2 pose, Eigen::Quaterniond quat);

		ros::Publisher pub_opt_odom_;
		ros::Publisher pub_odom_;

		ros::NodeHandle nh_;
		DataContainer* dc_;

		double odom_threshold_ = 0.85;
		double keyf_threshold_ = 0.9;
		double vel_threshold_ = 20.0;
		double angvel_threshold_ = 7.0;

		std::array<int, NUM> cost_idx;
		std::array<int, NUM> cost_iter;
		std::array<double, NUM> atv;
		double norm_v[NUM];
		double norm_w[NUM];

		// GTSAM
		ISAM2 *isam2;
		Values initial_values;
		Vector2 trans_ ;
		NonlinearFactorGraph* poseGraph = new NonlinearFactorGraph();
		Pose2 prev_pose, prop_pose;

		noiseModel::Diagonal::shared_ptr prior_noise_model_;
		noiseModel::Diagonal::shared_ptr loose_prior_noise_model_;
		noiseModel::Diagonal::shared_ptr odom_noise_model_;
		noiseModel::Diagonal::shared_ptr key_noise_model_;
		noiseModel::Diagonal::shared_ptr rot_noise_model_;

		Vector3 current_pose;
		Vector3 base_pose;

		int pose_count = 0;
		bool correlation_found = false;
		int key_node = 0;
		int window_loop =0;
		std::vector<int> pose_node_nums;
		std::vector<Vector3> pose_values;

		Eigen::Matrix3d m = (Eigen::Matrix3d() << pow(1.0,2), 0, 0, 0, pow(1.0,2), 0, 0, 0, pow(0.2,2)).finished();
		Eigen::Matrix3d n = (Eigen::Matrix3d() << pow(1.0,2), 0, 0, 0, pow(1.0,2), 0, 0, 0, pow(1.0,2)).finished();

};