#include "IsotropicHyperlasticForceModel.h"
#include "Simulator/IsotropicHyperelasticFEM/isotropicHyperelasticModel.h"
#include <fstream>

IsotropicHyperlasticForceModel::IsotropicHyperlasticForceModel(IsotropicHyperelasticModel* isotropicHyperelasticModel_)
{
	this->isotropicHyperelasticModel = isotropicHyperelasticModel_;
	LoboForceModel::r = isotropicHyperelasticModel->getR();
}

IsotropicHyperlasticForceModel::~IsotropicHyperlasticForceModel()
{
}

void IsotropicHyperlasticForceModel::updateMaterial()
{
	isotropicHyperelasticModel->updateMaterial();
}

void IsotropicHyperlasticForceModel::getInternalForce(double *u, double* internalForces, bool linear /*= true*/, bool quadratic /*= true*/, bool cubic /*= true*/)
{
	isotropicHyperelasticModel->computeForce(u, internalForces, NULL);
}

void IsotropicHyperlasticForceModel::getTangentStiffnessMatrix(double *u, SparseMatrix<double> * tangentStiffnessMatrix, bool linear /*= true*/, bool quadratic /*= true*/, bool cubic /*= true*/)
{
	isotropicHyperelasticModel->computeStiffnessMatrix(u, tangentStiffnessMatrix, true, NULL);
}

void IsotropicHyperlasticForceModel::getTangentStiffnessMatrixTopology(SparseMatrix<double> * tangentStiffnessMatrix)
{
	isotropicHyperelasticModel->getSparseMatrixTopology(*tangentStiffnessMatrix);
}

void IsotropicHyperlasticForceModel::getForceAndMatrix(double * u, double * internalForce, SparseMatrix<double> * tangentStiffnessMatrix)
{
	isotropicHyperelasticModel->computeForceAndMatrix(u, internalForce, tangentStiffnessMatrix, NULL);
}

double IsotropicHyperlasticForceModel::computeElementEnergy(int eleid)
{
	return isotropicHyperelasticModel->computeLinearElasticityStress(eleid);
	//return isotropicHyperelasticModel->computeCorotatedLinearStress(eleid);
	//return  isotropicHyperelasticModel->computeElementStress(eleid);
}
