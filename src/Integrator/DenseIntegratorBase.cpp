#include "DenseIntegratorBase.h"


DenseIntegratorBase::DenseIntegratorBase(int r, double timestep, MatrixXd *massMatrix, ReducedForceModel* reducedForceModel, int numConstrainedDOFs, int *constrainedDOFs_, double dampingMassCoef, double dampingStiffnessCoef) :LoboIntegrator(r, timestep, dampingMassCoef, dampingStiffnessCoef)
{
	this->reducedforcemodel = reducedForceModel;
	this->massMatrix = massMatrix;
	this->numConstrainedDOFs = numConstrainedDOFs;

	this->constrainedDOFs = (int*)malloc(sizeof(int) * numConstrainedDOFs);
	memcpy(this->constrainedDOFs, constrainedDOFs_, sizeof(int) * numConstrainedDOFs);

	dampingMatrix = new MatrixXd(r, r);
	dampingMatrix->setZero();

}


DenseIntegratorBase::~DenseIntegratorBase()
{
	free(constrainedDOFs);
	delete dampingMatrix;
}

void DenseIntegratorBase::setMassMatrix(MatrixXd *massMatrix_)
{
	this->massMatrix = massMatrix_;
}

