#include "ReducedSTVKForceModel.h"
#include "Simulator/ReducedSTVK/ReducedSTVKModel.h"

ReducedSTVKForceModel::ReducedSTVKForceModel(ReducedSTVKModel* simulatormodel)
{
	this->simulatormodel = simulatormodel;
}

ReducedSTVKForceModel::~ReducedSTVKForceModel()
{
}

void ReducedSTVKForceModel::getInternalForce(double* u, double * internalForce)
{
	simulatormodel->computeForce(u, internalForce);
}

void ReducedSTVKForceModel::getInternalForce(double* u, double* internalForce, std::vector<int>* indices)
{
	simulatormodel->computeForce(u, internalForce, indices);
}

void ReducedSTVKForceModel::getTangentStiffnessMatrix(double* u, MatrixXd * stiffnessmatrix)
{
	simulatormodel->computeStiffnessMatrix(u, stiffnessmatrix);
}

void ReducedSTVKForceModel::resetModel()
{

}

