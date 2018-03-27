#include "LoboIsotropicHyperelasticWithCompressionResistance.h"
#include <math.h>
#include <iostream>
LoboIsotropicHyperelasticWithCompressionResistance::LoboIsotropicHyperelasticWithCompressionResistance(bool enableCompressionResistance_ /*= 0*/) :enableCompressionResistance(enableCompressionResistance_)
{
	
}

LoboIsotropicHyperelasticWithCompressionResistance::~LoboIsotropicHyperelasticWithCompressionResistance()
{

}

double LoboIsotropicHyperelasticWithCompressionResistance::GetCompressionResistanceFactor(int elementIndex)
{
	return 1.0;
}

void LoboIsotropicHyperelasticWithCompressionResistance::AddCompressionResistanceEnergy(int elementIndex, double * invariants, double * energy)
{
	if (enableCompressionResistance)
	{
		double IIIC = invariants[2];
		double J = sqrt(IIIC);

		if (J < 1)
		{
			double compressionResistanceFactor = GetCompressionResistanceFactor(elementIndex);
			*energy += -compressionResistanceFactor * (J - 1.0) * (J - 1.0) * (J - 1.0) / 2592.0;
		}
	}
}

void LoboIsotropicHyperelasticWithCompressionResistance::AddCompressionResistanceGradient(int elementIndex, double * invariants, double * gradient)
{
	if (enableCompressionResistance)
	{
		double IIIC = invariants[2];
		double J = sqrt(IIIC);

		if (J < 1)
		{
			double compressionResistanceFactor = GetCompressionResistanceFactor(elementIndex);

			gradient[2] += -compressionResistanceFactor * (J - 1.0) * (J - 1.0) / (1728.0 * J);
		}
	}
}

void LoboIsotropicHyperelasticWithCompressionResistance::AddCompressionResistanceHessian(int elementIndex, double * invariants, double * hessian)
{
	if (enableCompressionResistance)
	{
		double IIIC = invariants[2];
		double J = sqrt(IIIC);

		if (J < 1.0)
		{
			double compressionResistanceFactor = GetCompressionResistanceFactor(elementIndex);
			hessian[5] += compressionResistanceFactor * (1.0 - J) * (1.0 + J) / (3456.0 * J * J * J);
		}
	}
}
