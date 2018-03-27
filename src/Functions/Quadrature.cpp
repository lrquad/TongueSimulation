#include "Quadrature.h"
int Quadrature(int quadrature_order, int dimension, std::vector<double> &weights, std::vector<double> &quadratures)
{
	int numQuadratures = std::pow(quadrature_order,dimension);

	weights.resize(numQuadratures);
	quadratures.resize(numQuadratures*dimension);
	VectorXd r1pt(quadrature_order);
	VectorXd r1wt(quadrature_order);

	switch (quadrature_order)
	{
	case 1:
		r1pt.data()[0] = 0.000000000000000;
		r1wt.data()[0] = 2.000000000000000;
		break;
	case 2:
		r1pt.data()[0] = 0.577350269189626;
		r1pt.data()[1] = -0.577350269189626;

		r1wt.data()[0] = 1.000000000000000;
		r1wt.data()[1] = 1.000000000000000;
		break;
	case 4:
		r1pt.data()[0] = 0.861134311594053;
		r1pt.data()[1] = -0.861134311594053;
		r1pt.data()[2] = 0.339981043584856;
		r1pt.data()[3] = -0.339981043584856;

		r1wt.data()[0] = 0.347854845137454;
		r1wt.data()[1] = 0.347854845137454;
		r1wt.data()[2] = 0.652145154862546;
		r1wt.data()[3] = 0.652145154862546;

	default:
		break;
	}

	int n = 0;

	if (dimension == 2)
	{
		for (int i = 0; i < quadrature_order; i++)
		{
			for (int j = 0; j < quadrature_order; j++)
			{
				weights.data()[n] = r1wt.data()[i] * r1wt.data()[j];

				quadratures.data()[n] = r1pt.data()[i];
				quadratures.data()[n + numQuadratures] = r1pt.data()[j];
				n++;
			}
		}
	}

	return numQuadratures;
}