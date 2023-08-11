#pragma once

#include <array>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>

#include <imgProcess/ImageTF.hpp>
#include <imgProcess/fftModule.hpp>
#include <util.hpp>

class AbstractFactor
{
	public:
		AbstractFactor(std::vector<cv::Mat>* ptr_window_list,
				std::vector<cv::Mat>* ptr_window_list_cart,
				std::vector<cv::Mat>* ptr_window_list_cart_f,
				std::vector<cv::Mat>* ptr_keyf_list,
				std::vector<cv::Mat>* ptr_keyf_list_cart,
				std::vector<cv::Mat>* ptr_keyf_list_cart_f);
		~AbstractFactor();

		std::array<double, 3> factorGeneration(int src1, int src2);
		std::array<double, 3> phaseCorr2D(cv::Mat r_src1, cv::Mat r_src2, cv::Mat src1,
											cv::Mat src2, bool flag, std::array<double, 3> state);
		
	protected:
		std::vector<cv::Mat>* ptr_window_list_;
		std::vector<cv::Mat>* ptr_window_list_cart_;
		std::vector<cv::Mat>* ptr_window_list_cart_f_;

		std::vector<cv::Mat>* ptr_keyf_list_;
		std::vector<cv::Mat>* ptr_keyf_list_cart_;
		std::vector<cv::Mat>* ptr_keyf_list_cart_f_;

		double init_val[3] = {0,};
		double init_val_f[3] = {0,};

		double ratio = 10.0;	// seq03_kaist : 14.0, others : 10.0
		const double RESOL = 0.059612233;
		//const double RESOL = 2733/1024;

		ImageTF itf;
		ImageTF itf_f;
};