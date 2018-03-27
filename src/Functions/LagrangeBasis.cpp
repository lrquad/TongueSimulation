#include "LagrangeBasis.h"
void lagrange_basis(VectorXd&N, MatrixXd& dNdxi, Vector2d coord)
{
	double xi = coord.data()[0];
	double eta = coord.data()[1];
	N.resize(4);
	N.data()[0] = (1 - xi)*(1 - eta);
	N.data()[1] = (1 + xi)*(1 - eta);
	N.data()[2] = (1 + xi)*(1 + eta);
	N.data()[3] = (1 - xi)*(1 + eta);
	N *= 1. / 4.;

	dNdxi.resize(4, 2);
	dNdxi.data()[0] = -(1 - eta);
	dNdxi.data()[1] = 1 - eta;
	dNdxi.data()[2] = 1 + eta;
	dNdxi.data()[3] = -(1 + eta);

	dNdxi.data()[4] = -(1 - xi);
	dNdxi.data()[5] = -(1 + xi);
	dNdxi.data()[6] = 1 + xi;
	dNdxi.data()[7] = 1 - xi;
	dNdxi *= 1. / 4.;
}