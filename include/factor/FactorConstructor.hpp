#pragma once

#include <array>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>

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

#include <imgProcess/ImageTF.hpp>
#include <imgProcess/fftModule.hpp>
#include <DataContainer.hpp>
#include <util.hpp>

using namespace gtsam;

class FactorConstructor
{
	public:
		FactorConstructor(DataContainer* dc);
		~FactorConstructor();

		std::array<double, 3> factorGeneration(int src1, int src2);
		std::array<double, 3> phaseCorr2D(cv::Mat r_src1, cv::Mat r_src2, cv::Mat src1,
											cv::Mat src2, bool flag, std::array<double, 3> state);
		
	protected:
		DataContainer* dc_;

		double init_val[3] = {0,};
		double init_val_f[3] = {0,};

		double ratio = 10.0;	// seq03_kaist : 14.0, others : 10.0
		const double RESOL = 0.059612233;
		//const double RESOL = 2733/1024;

		ImageTF itf;
};