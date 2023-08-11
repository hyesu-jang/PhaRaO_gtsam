#include <factor/AbstractFactor.hpp>

AbstractFactor::AbstractFactor(std::vector<cv::Mat>* ptr_window_list,
				std::vector<cv::Mat>* ptr_window_list_cart,
				std::vector<cv::Mat>* ptr_window_list_cart_f,
				std::vector<cv::Mat>* ptr_keyf_list,
				std::vector<cv::Mat>* ptr_keyf_list_cart,
				std::vector<cv::Mat>* ptr_keyf_list_cart_f)
				: ptr_window_list_(ptr_window_list),
				ptr_window_list_cart_(ptr_window_list_cart),
				ptr_window_list_cart_f_(ptr_window_list_cart_f),
				ptr_keyf_list_(ptr_keyf_list),
				ptr_keyf_list_cart_(ptr_keyf_list_cart),
				ptr_keyf_list_cart_f_(ptr_keyf_list_cart_f)
{

}

AbstractFactor::~AbstractFactor()
{

}

std::array<double, 3>
AbstractFactor::factorGeneration(int src1, int src2)
{
    // Coarse Phase Correlation Module
    auto begin_iter = ptr_window_list_->begin();
    auto begin_iter_cart = ptr_window_list_cart_->begin();

    std::array<double, 3> state = {0,0,0};
    std::array<double, 3> cd_state = phaseCorr2D(*(begin_iter+src1), *(begin_iter+src2),
                                    *(begin_iter_cart+src1), *(begin_iter_cart+src2), true, state);

    // Fine Phase Correlation Module
    begin_iter_cart = ptr_window_list_cart_f_->begin();

    std::array<double, 3> fd_state = phaseCorr2D(*(begin_iter+src1), *(begin_iter+src2),
                                    *(begin_iter_cart+src1), *(begin_iter_cart+src2), false, cd_state);

	std::array<double, 3> out_state;
    std::copy(fd_state.begin(), fd_state.end(), out_state.begin());

	return out_state;
}


std::array<double, 3>
AbstractFactor::phaseCorr2D(cv::Mat r_src1, cv::Mat r_src2, cv::Mat src1, cv::Mat src2,
					bool flag, std::array<double, 3> state)
{
	// flag == true : coarse, flag == false : fine
	double x,y,theta;
	double rotation;
	double phaseCorr;
	double factor;

	cv::Point2d peakLoc_r;

	if(flag == true) {
		peakLoc_r = cv::phaseCorrelate(r_src1, r_src2, cv::noArray(), &phaseCorr);
		rotation = (peakLoc_r.y - init_val[2])*(360.0/(r_src1.rows));
		theta = rotation*(M_PI/180.0);
	} else {
		theta = state.at(2);
		rotation = theta*180.0/M_PI;
	}

	cv::Point2f center(src2.cols/2.0, src2.rows/2.0);
	cv::Mat rot = cv::getRotationMatrix2D(center, rotation, 1.0);
	cv::Mat derot_cart;
	cv::warpAffine(src2, derot_cart, rot, src2.size());

	cv::Point2d peakLoc;
	if(flag == true) {
		peakLoc = cv::phaseCorrelate(src1, derot_cart, cv::noArray(), &phaseCorr);
	
		factor = ratio;
		x = (init_val[0]-peakLoc.x)*RESOL*factor;
		y = (init_val[1]-peakLoc.y)*RESOL*factor;
	} else {
    	int pixel_x = round(state.at(0)/(ratio * RESOL));
    	int pixel_y = round(state.at(1)/(ratio * RESOL));

    	if( abs(pixel_x) > 500 || abs(pixel_y) > 500){
			x = state.at(0);
			y = state.at(1);
		} else {
			peakLoc = itf_f.phaseCorrelateWindow(src1, derot_cart, cv::noArray(), &phaseCorr, state);

			factor = 1.0;
			x = state.at(0) + (init_val_f[0]-peakLoc.x)*RESOL*factor;
			y = state.at(1) + (init_val_f[1]-peakLoc.y)*RESOL*factor;
		}
	}

	if(initialized == false) {
		if(flag == true) {
			init_val[0] = peakLoc.x;
			init_val[1] = peakLoc.y;
			init_val[2] = peakLoc_r.y;
		} else {
			init_val_f[0] = peakLoc.x;
			init_val_f[1] = peakLoc.y;
			init_val_f[2] = peakLoc_r.y;
		}

		theta = 0;
		x = 0;
		y = 0;
	}

	std::array<double, 3> d_state = {x, y, theta};
	return d_state;
}