#pragma once
#include "LoboForceModel.h"
class IsotropicHyperelasticModel;

class IsotropicHyperlasticForceModel :public LoboForceModel
{
public:
	IsotropicHyperlasticForceModel(IsotropicHyperelasticModel* isotropicHyperelasticModel_);
	~IsotropicHyperlasticForceModel();

	virtual void updateMaterial();

	virtual void getInternalForce(double *u, double* internalForces, bool linear = true, bool quadratic = true, bool cubic = true);

	virtual void getTangentStiffnessMatrix(double *u, SparseMatrix<double> * tangentStiffnessMatrix, bool linear = true, bool quadratic = true, bool cubic = true);

	virtual void getTangentStiffnessMatrixTopology(SparseMatrix<double> * tangentStiffnessMatrix);

	virtual void getForceAndMatrix(double * u, double * internalForce, SparseMatrix<double> * tangentStiffnessMatrix);

	virtual double computeElementEnergy(int eleid);

protected:
	IsotropicHyperelasticModel* isotropicHyperelasticModel;
};

