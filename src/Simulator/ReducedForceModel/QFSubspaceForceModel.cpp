#include "QFSubspaceForceModel.h"
#include "Simulator/FrameBased/QuadraticFrameSubspaceModel.h"

QFSubspaceForceModel::QFSubspaceForceModel(QuadraticFrameSubspaceModel* qfsubspacemodel)
	:FrameBasedForceModel(qfsubspacemodel)
{
	this->qfsubspacemodel = qfsubspacemodel;
}


QFSubspaceForceModel::~QFSubspaceForceModel()
{
}

void QFSubspaceForceModel::getInternalForce(double* u, double* internalForce, std::vector<int>* ele_indices, std::vector<int>* node_indices)
{
	qfsubspacemodel->computeSubForce(u, internalForce, ele_indices, node_indices);
}

void QFSubspaceForceModel::getInternalForce(double* u, double* internalForce, std::vector<int>* ele_indices)
{
	qfsubspacemodel->computeForce(u, internalForce, ele_indices);
} 
