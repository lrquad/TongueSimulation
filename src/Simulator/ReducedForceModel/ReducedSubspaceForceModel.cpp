#include "ReducedSubspaceForceModel.h"
#include "Simulator/ReducedSubspaceModel/ReducedSubspaceModel.h"


ReducedSubspaceForceModel::ReducedSubspaceForceModel(ReducedSubspaceModel* simulatormodel)
{
	this->simulatormodel = simulatormodel;
	this-> r = simulatormodel->getR();

}

ReducedSubspaceForceModel::~ReducedSubspaceForceModel()
{

}

void ReducedSubspaceForceModel::getInternalForce(double* u, double * internalForce)
{
	simulatormodel->computeForcebyfull(u, internalForce);
}

void ReducedSubspaceForceModel::getInternalForce(double* u, double* internalForce, std::vector<int>* indices)
{
	simulatormodel->computeForce(u, internalForce,indices);
}

void ReducedSubspaceForceModel::getTangentStiffnessMatrix(double*u, MatrixXd* stiffnessmatrix)
{
	simulatormodel->computeStiffnessMatrixbyfull(u, stiffnessmatrix);
}

void ReducedSubspaceForceModel::getForceAndMatrix(double*u, double* internalForce, MatrixXd* stiffnessmatrix)
{
	simulatormodel->computeForceAndMatrix(u, internalForce, stiffnessmatrix);
}

void ReducedSubspaceForceModel::resetModel()
{

}
