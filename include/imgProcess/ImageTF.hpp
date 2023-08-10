#pragma once

#include <Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/eigen.hpp>

#define WARP_POLAR_LINEAR 0
#define WARP_POLAR_LOG 256 

using namespace cv;
using namespace std;
using namespace Eigen;

class ImageTF
{
	public:
		ImageTF (void);
		~ImageTF (void);

		void warpPolar (InputArray _src, OutputArray _dst, Size dsize,
						Point2f center, double maxRadius, int flags);

		void magSpectrums( InputArray _src, OutputArray _dst);
		void divSpectrums( InputArray _srcA, InputArray _srcB, OutputArray _dst, int flags, bool conjB);
		void fftShift(InputOutputArray _out);
		Point2d weightedCentroid(InputArray _src, cv::Point peakLocation, cv::Size weightBoxSize, double* response);
		Point2d phaseCorrelateWindow(InputArray _src1, InputArray _src2, InputArray _window,
									 double* response, array<double, 3> state);
		void createHanningWindow(OutputArray _dst, cv::Size winSize, int type);

		ArrayXXf highpassfilt(Size size, bool init);

	private:
		Mat mapx, mapy;

		bool preCalc = 0;
		double RESOL = 0.059612233;

		ArrayXXf preCalc_filter;
};
