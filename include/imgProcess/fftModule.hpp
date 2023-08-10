#pragma once

#include <Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>

#include <fftw3.h>

using namespace cv;
using namespace std;
using namespace Eigen;

class fftModule
{
	public:
		fftModule (void);
		~fftModule (void);

		void dft(Mat img, Mat* real_out, Mat* imag_out);
        Mat idft(Mat img);

	private:

};
