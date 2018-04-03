#pragma once
#include "SparseIntegratorBase.h"

class ImplicitBackwardEulerSparseIntegrator:public SparseIntegratorBase
{
public:
	ImplicitBackwardEulerSparseIntegrator(int r,double timestep,SparseMatrix<double>* massMatrix, LoboForceModel* forcemodel,int positiveDefiniteSolver_,int numConstrainedDOFs = 0,int * constrainedDOFs = NULL,double dampingMaxxCoef = 0.0, double dampingStiffnessCoef = 0.0,int maxIterations = 1);

	~ImplicitBackwardEulerSparseIntegrator();

	virtual int setState(double *q, double *qvel = NULL);
	virtual int doTimeStep();


protected:

	SparseMatrix<double> * rayleighDampingMatrix;
	SparseMatrix<double> * tangentStiffnessMatrix;
	SparseMatrix<double> * systemMatrix;
	SparseMatrix<double> * subSystemMatrix;
	SparseMatrix<double> * constrainedSystemMatrix;
	std::vector<int> mapOldNew;
	std::vector<int> mapSparseMatrixEntryOldNew;


	int maxIterations;
};

