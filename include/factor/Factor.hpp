#pragma once

#include <array>

#include <opencv2/core.hpp>

class Factor
{
	public:
		Factor();
		~Factor();

		std::array<double, 3> phaseCorr2D(cv::Mat r_src1, cv::Mat r_src2, cv::Mat src1,
											cv::Mat src2, bool flag, std::array<double, 3> state);
		void factorGeneration(int src1, int src2, std::array<double, 3>& out_state);
};