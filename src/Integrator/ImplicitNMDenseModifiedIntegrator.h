#pragma once
#include "Integrator/ImpicitNewMarkDenseIntegrator.h"

class ImplicitNMDenseModifiedIntegrator :public ImpicitNewMarkDenseIntegrator
{
public:
	ImplicitNMDenseModifiedIntegrator(int r, double timestep, MatrixXd * massmatrix, ReducedForceModel* reducedforcemodel, int numConstrainedDOFs = 0, int *constrainedDOFs = NULL, double dampingMassCoef = 0.0, double dampingStiffnessCoef = 0.0, int maxIterations = 1, double epsilon = 1e-6, double NewmarkBeta = 0.25, double NewmarkGama = 0.5, bool usestatic = false);
	~ImplicitNMDenseModifiedIntegrator();

	virtual void updateSubspace(MatrixXd* UUT);
	virtual bool updateSubspaceLeastSquare(MatrixXd* W,MatrixXd* U);


protected:



};

