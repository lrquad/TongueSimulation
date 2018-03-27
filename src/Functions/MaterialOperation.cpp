#include "MaterialOperation.h"

void computeStrain(Matrix3d &inputF, Matrix3d &outputE)
{
	outputE = 0.5*(inputF.transpose()*inputF - Matrix3d().Identity());
}

void computePiolaStress(Matrix3d &E, Matrix3d &F, Matrix3d &P, double &mu_, double &lambda_)
{
	P = F*(2.0 * mu_*E + lambda_*E.trace()*Matrix3d().Identity());
}