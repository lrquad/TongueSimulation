#pragma once
#include <Eigen/Dense>
#include <Eigen/Sparse>
using namespace Eigen;

class ReducedForceModel
{
public:
	ReducedForceModel();
	~ReducedForceModel();

	inline int getR(){ return r; }

	virtual void getInternalForce(double* u, double * internalForce) = 0;
	virtual void getTangentStiffnessMatrix(double* u, MatrixXd * stiffnessmatrix) = 0;

	virtual void getInternalForce(double* u, double* internalForce, std::vector<int>* indices) = 0;

	virtual void resetModel() = 0;
	
	virtual void getForceAndMatrix(double* u, double *internalForce, MatrixXd* stiffnessmatrix);

	void TestStiffnessMatrix(int numTrials, double qMagnitude = 1.0);

protected:
	int r;
};

