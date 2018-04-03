#include "LoboReducedForceModel.h"


LoboReducedForceModel::LoboReducedForceModel()
{
}


LoboReducedForceModel::~LoboReducedForceModel()
{
}

void LoboReducedForceModel::getForceAndMatrix(double * u, double * internalForce, MatrixXd * tangentStiffnessMatrix)
{
	this->getInternalForce(u, internalForce);
	this->getTangentStiffnessMatrix(u, tangentStiffnessMatrix);
}