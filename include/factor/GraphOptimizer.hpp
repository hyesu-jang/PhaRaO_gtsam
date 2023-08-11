#pragma once

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

#include <factor/AbstractFactor.hpp>
#include <gtsam_custom/hs_rotation_factor.h>

using namespace gtsam;
using symbol_shorthand::X; // Pose2 (x,y,theta)
using symbol_shorthand::K; // Rot2 (theta)

class GraphOptimizer
{
	public:
		GraphOptimizer();
		~GraphOptimizer();
		void setInputLists(std::vector<cv::Mat>* ptr_window_list,
							std::vector<cv::Mat>* ptr_window_list_cart,
							std::vector<cv::Mat>* ptr_window_list_cart_f,
							std::vector<cv::Mat>* ptr_keyf_list,
							std::vector<cv::Mat>* ptr_keyf_list_cart,
							std::vector<cv::Mat>* ptr_keyf_list_cart_f);
		void optimize();

	protected:

		std::vector<cv::Mat>* ptr_window_list_;
		std::vector<cv::Mat>* ptr_window_list_cart_;
		std::vector<cv::Mat>* ptr_window_list_cart_f_;

		std::vector<cv::Mat>* ptr_keyf_list_;
		std::vector<cv::Mat>* ptr_keyf_list_cart_;
		std::vector<cv::Mat>* ptr_keyf_list_cart_f_;

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
		std::vector<int> pose_node_nums;
		Vector3 now_pose;
		Vector3 base_pose;
		std::vector<Vector3> pose_values;
		//vector<Pose2d_Node*> pose_nodes;
		ISAM2 *g_isam;
		//Slam* g_slam;
		//vector<Pose2d_Node*> g_pose_nodes;
		Eigen::Matrix3d m = (Eigen::Matrix3d() << pow(1.0,2), 0, 0, 0, pow(1.0,2), 0, 0, 0, pow(0.2,2)).finished();
		Eigen::Matrix3d n = (Eigen::Matrix3d() << pow(1.0,2), 0, 0, 0, pow(1.0,2), 0, 0, 0, pow(1.0,2)).finished();

		// Noise f3 = Information(pow(1.0,2) * m);
		// Noise noise3 = Information(pow(1.0,2) * n);
		// Noise p3 = Information(pow(10,12) * eye(3));
		// Noise kf1 = Information(pow(5.0,2)*eye(1));
		// Noise f1 = Information(pow(2.5,2)*eye(1));

		Rot2 key_rot, orien;

};