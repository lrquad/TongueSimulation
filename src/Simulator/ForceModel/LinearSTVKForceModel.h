#pragma once
#include "LoboForceModel.h"

#include "Simulator/ForceModel/STVKForceModel.h"

class STVKModel;

class LinearSTVkForceModel :public STVKForceModel
{
public:
	LinearSTVkForceModel(STVKModel* stvkmodel_);
	~LinearSTVkForceModel();

	virtual void updateMaterial();
	virtual void computeLinearStiffnessMatrix(double *u, SparseMatrix<double> * tangentStiffnessMatrix);
	
	virtual void getInternalForce(double *u, double* internalForces, bool linear = true, bool quadratic = true, bool cubic = true);

	virtual void getTangentStiffnessMatrix(double *u, SparseMatrix<double> * tangentStiffnessMatrix, bool linear = true, bool quadratic = true, bool cubic = true);

	virtual void getTangentStiffnessMatrixTopology(SparseMatrix<double> * tangentStiffnessMatrix);

};

