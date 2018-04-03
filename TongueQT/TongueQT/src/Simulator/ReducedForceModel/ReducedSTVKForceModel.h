#pragma once
#include "ReducedForceModel.h"
class ReducedSTVKModel;

class ReducedSTVKForceModel:public ReducedForceModel
{
public:
	ReducedSTVKForceModel(ReducedSTVKModel* simulatormodel);
	~ReducedSTVKForceModel();

	virtual void getInternalForce(double* u, double * internalForce);
	virtual void getInternalForce(double* u, double* internalForce, std::vector<int>* indices);


	virtual void getTangentStiffnessMatrix(double* u, MatrixXd * stiffnessmatrix);


	virtual void resetModel();

protected:
	ReducedSTVKModel* simulatormodel;
};

