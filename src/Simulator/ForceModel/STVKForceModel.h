#pragma once
#include "LoboForceModel.h"
class STVKModel;
class STVKForceModel:public LoboForceModel
{
public:
	STVKForceModel(STVKModel* stvkmodel_);
	~STVKForceModel();

	virtual void getInternalForce(double *u, double* internalForces, bool linear = true, bool quadratic = true, bool cubic = true);

	virtual void getTangentStiffnessMatrix(double *u, SparseMatrix<double> * tangentStiffnessMatrix, bool linear = true, bool quadratic = true, bool cubic = true);

	virtual void getTangentStiffnessMatrixTopology(SparseMatrix<double> * tangentStiffnessMatrix);

protected:
	STVKModel* stvkmodel;

};

