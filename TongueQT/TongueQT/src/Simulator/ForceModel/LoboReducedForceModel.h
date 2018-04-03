#pragma once
#include <Eigen/Dense>
#include <Eigen/Sparse>
using namespace Eigen;
class LoboReducedForceModel
{
public:
	LoboReducedForceModel();
	~LoboReducedForceModel();

	virtual void getInternalForce(double *u, double* internalForces) = 0;
	virtual void getTangentStiffnessMatrix(double *u, MatrixXd * tangentStiffnessMatrix) = 0;

	virtual void ResetToZero(){};
	virtual void Reset(double *q){};

	virtual void getForceAndMatrix(double * u, double * internalForce, MatrixXd * tangentStiffnessMatrix);

protected:
	int r;
};

