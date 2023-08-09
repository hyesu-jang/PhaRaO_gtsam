#include <imgProcess/fftModule.hpp>

fftModule::fftModule(void)
{
    fftwf_init_threads();
    fftwf_plan_with_nthreads(4);
}

fftModule::~fftModule(void)
{

}

void 
fftModule::dft(Mat img, Mat* real_out, Mat* imag_out)
{
    ArrayXXcf x = ArrayXXcf::Zero(img.cols, img.rows);
    x.real() = Map<ArrayXXf>(&img.at<float>(0,0), img.cols, img.rows);

    ArrayXXcf xf = ArrayXXcf(x.rows(), x.cols());

    fftwf_plan fft_plan = fftwf_plan_dft_2d(x.cols(), x.rows(), (float(*)[2])(x.data()), 
        (float(*)[2])(xf.data()), FFTW_FORWARD, FFTW_ESTIMATE); // reverse order for column major
    
    //polar_mutex.lock();
    fftwf_execute(fft_plan);
    //polar_mutex.unlock();

    MatrixXf xf_real_mt = xf.real().matrix();
    MatrixXf xf_imag_mt = xf.imag().matrix();

    eigen2cv(xf_real_mt, *real_out);
    eigen2cv(xf_imag_mt, *imag_out);

    //fftwf_cleanup_threads();
    fftwf_destroy_plan(fft_plan);
}

Mat
fftModule::idft(Mat img)
{
    ArrayXXcf xf;
    ArrayXXf x = ArrayXXf((xf.rows()-1)*2, xf.cols());
    
    ArrayXXcf cxf;

    
    fftwf_plan fft_plan = fftwf_plan_dft_c2r_2d(xf.cols(), (xf.rows()-1)*2, (float(*)[2])(cxf.data()),
        (float(*))(x.data()), FFTW_ESTIMATE);
    
    fftwf_execute(fft_plan);
    
    return img;
}
