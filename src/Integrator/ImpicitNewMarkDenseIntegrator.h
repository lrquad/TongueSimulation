#pragma once
#include "DenseIntegratorBase.h"

class ImpicitNewMarkDenseIntegrator: public DenseIntegratorBase
{
public:
	ImpicitNewMarkDenseIntegrator(int r, double timestep, MatrixXd * massmatrix, ReducedForceModel* reducedforcemodel, int numConstrainedDOFs = 0, int *constrainedDOFs = NULL, double dampingMassCoef = 0.0, double dampingStiffnessCoef = 0.0, int maxIterations = 1, double epsilon = 1e-6, double NewmarkBeta = 0.25, double NewmarkGama = 0.5, bool usestatic = false);
	~ImpicitNewMarkDenseIntegrator();

	virtual void updateMaterial();

	virtual bool computeStaticDisplacement(VectorXd &externalForce, bool checkwolfecondition = false);

	inline virtual void setTimeStep(double timestep){ this->timestep = timestep; updateAlphas(); }

	virtual int setState(double *q, double *qvel = NULL);

	virtual int doTimeStep();
	virtual int doTimeStepWithLagrangeMulti();
	virtual int doTimeStepWithPanelty();

	void assginConstrainedSystem(VectorXd &systemF);


	virtual void setConstrainByLagrangeMulti(MatrixXd &reducedConstrainedMatrix, VectorXd &reducedConstrainVector);

	bool getUseLagrangeMulti() const { return useLagrangeMulti; }
	void setUseLagrangeMulti(bool val) { useLagrangeMulti = val; }
protected:

	void dotimestepwithConstrainedDOFs(VectorXd &systemF);
	void dotimestepwithoutConstrainedDOFs(VectorXd &systemF);

	MatrixXd * tangentStiffnessMatrix;
	MatrixXd * systemMatrix;
	MatrixXd * subSystemMatrix;

	MatrixXd reducedConstrainedMatrix;
	VectorXd reducedConstrainVector;

	MatrixXd lagrangesystemMatrix;
	VectorXd lagrangesystemVector;

	std::vector<int> mapOldNew;
	void updateAlphas();


	double NewmarkBeta, NewmarkGamma;
	double alpha1, alpha2, alpha3, alpha4, alpha5, alpha6;
	double epsilon;
	int maxIterations;

	bool useLagrangeMulti;

};

