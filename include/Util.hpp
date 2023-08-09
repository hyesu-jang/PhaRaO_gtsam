inline cv::Mat
convertToPolar(cv::Mat img)
{
	cv::Point2f center(img.cols / 2.0F, img.rows / 2.0F);
	double maxRadius = cv::norm(cv::Point2f(img.cols - center.x, img.rows - center.y));
	cv::linearPolar(img, img, center, maxRadius, cv::WARP_FILL_OUTLIERS);
	
	// Extract nearest polar points
	img = img.t();
	img = polarToNearPol(img);
	imshow("polar.",img);
	
	return img;
}

inline cv::Mat 
log_polar(const cv::Mat img)
{
	cv::Mat log_polar_img;

	cv::Point2f center((float)img.cols/2, (float)img.rows/2);

	double radius = (double)img.rows / 2;

	double M = (double)img.cols / log(radius);

	cv::logPolar(img, log_polar_img, center, M, cv::INTER_LINEAR + cv::WARP_FILL_OUTLIERS);

	return log_polar_img;
}

inline cv::Mat 
polarToNearPol(const cv::Mat& polar_img) {
	
    int range = polar_img.rows;
    int theta = polar_img.cols;

    cv::Mat polar_ref = cv::Mat::zeros(range, theta, polar_img.type());
    polar_ref(cv::Range(0, 120), cv::Range::all()).copyTo(polar_img(cv::Range(0, 120), cv::Range::all()));

    for (int th = 0; th < theta; ++th) {
        int non_zero_count = 0;
        for (int r = 0; r < range; ++r) {
            if (polar_img.at<uchar>(r, th) != 0) {
                polar_ref.at<uchar>(r, th) = polar_img.at<uchar>(r, th);
                ++non_zero_count;
                if (non_zero_count == 100) {
                    break;
                }
            }
        }
    }
    return polar_ref;
}

inline void
fftshift(const Mat& inputImg, Mat& outputImg)
{
    outputImg = inputImg.clone();
    int cx = outputImg.cols / 2;
    int cy = outputImg.rows / 2;
    Mat q0(outputImg, Rect(0, 0, cx, cy));
    Mat q1(outputImg, Rect(cx, 0, cx, cy));
    Mat q2(outputImg, Rect(0, cy, cx, cy));
    Mat q3(outputImg, Rect(cx, cy, cx, cy));
    Mat tmp;
    q0.copyTo(tmp);
    q3.copyTo(q0);
    tmp.copyTo(q3);
    q1.copyTo(tmp);
    q2.copyTo(q1);
    tmp.copyTo(q2);
}