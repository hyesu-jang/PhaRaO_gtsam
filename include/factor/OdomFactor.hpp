#pragma once

#include <factor/AbstractFactor.hpp>

class OdomFactor : public AbstractFactor
{
	public:
		OdomFactor(DataContainer* dc);
		~OdomFactor();
		bool calcOdom();

	protected:
		DataContainer* dc_;
		double threshold_;


};