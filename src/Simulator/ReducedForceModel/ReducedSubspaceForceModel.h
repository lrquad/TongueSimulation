#pragma once
#include "ReducedForceModel.h"
class ReducedSubspaceModel;

class ReducedSubspaceForceModel:public ReducedForceModel
{
public:
	ReducedSubspaceForceModel(ReducedSubspaceModel* simulatormodel);
	~ReducedSubspaceForceModel();
	
	virtual void getInternalForce(double* u, double * internalForce);
	virtual void getInternalForce(double* u, double* internalForce, std::vector<int>* indices);

	virtual void getTangentStiffnessMatrix(double*u, MatrixXd* stiffnessmatrix);

	virtual void getForceAndMatrix(double*u, double* internalForce, MatrixXd* stiffnessmatrix);

	virtual void resetModel();



protected:



	ReducedSubspaceModel* simulatormodel;

};

