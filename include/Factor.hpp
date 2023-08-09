#include <Factor.hpp>

Factor::Factor()
{

}

Factor::~Factor()
{

}

void
radarOdom::FactorGeneration(int src1, int src2, array<double, 3>& out_state)
{
	// Coarse Phase Correlation Module
	auto begin_iter = window_list.begin();
	auto begin_iter_cart = window_list_cart.begin();

	array<double, 3> state = {0,0,0};
	array<double, 3> cd_state = PhaseCorr2D(*(begin_iter+src1), *(begin_iter+src2),
									*(begin_iter_cart+src1), *(begin_iter_cart+src2), true, state);

	// Fine Phase Correlation Module
	begin_iter_cart = window_list_cart_f.begin();

	array<double, 3> fd_state = PhaseCorr2D(*(begin_iter+src1), *(begin_iter+src2),
									*(begin_iter_cart+src1), *(begin_iter_cart+src2), false, cd_state);

	polar_mutex.lock();
		std::copy(fd_state.begin(), fd_state.end(), out_state.begin());
	polar_mutex.unlock();
}
