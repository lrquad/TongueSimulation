#pragma once
#include "SparseIntegratorBase.h"
class Vegatest;
class ImplicitNewMatkSparseIntegrator:public SparseIntegratorBase
{
public:
	ImplicitNewMatkSparseIntegrator(int r, double timestep, SparseMatrix<double> *massMatrix, LoboForceModel* forcemodel, int positiveDefiniteSolver_, int numConstrainedDOFs = 0, int *constrainedDOFs = NULL, double dampingMassCoef = 0.0, double dampingStiffnessCoef = 0.0, int maxIterations = 1, double epsilon = 1e-6, double NewmarkBeta = 0.25, double NewmarkGama = 0.5, bool usestatic = false);
	
	~ImplicitNewMatkSparseIntegrator();

	virtual void updateMaterial();

	virtual void setDampingMatrix(SparseMatrix<double> *dampingMatrix);
	inline virtual void setTimeStep(double timestep){ this->timestep = timestep; updateAlphas(); }

	//sets q, qvel
	virtual int setState(double *q, double *qvel =NULL);

	virtual void changeCurrentState(VectorXd &q_,VectorXd &q_vel_,VectorXd &q_accel_);
	virtual void changeCurrentState(VectorXd &q_1_, VectorXd &q_, VectorXd &q_vel_, VectorXd &q_accel_);

	virtual int doTimeStep();

	virtual Eigen::VectorXd getInteranlForce(VectorXd q_);


	virtual bool computeStaticDisplacement(VectorXd &externalForce,bool checkwolfecondition = false);
	virtual void testDisplacementOutputInternalforce(VectorXd &q_input, const char* filename);

	inline void setNewmarkBeta(double NewmarkBeta){ this->NewmarkBeta = NewmarkBeta; updateAlphas(); }
	inline void setNewmarkGamma(double NewmarkGamma){ this->NewmarkGamma = NewmarkGamma; updateAlphas(); }

	virtual void useStaticSolver(bool b);

	virtual void setConstrainByLagrangeMulti(SparseMatrix<double>* constrainMatrix);
	virtual void setConstrainByLagrangeMulti(SparseMatrix<double>* constrainMatrix, VectorXd constrainTarget);
	
	 void setMaxInteration(int maxiteration);
	 int getMaxInteration(){ return maxIterations; }

	 SparseMatrix<double> * getTangentStiffnessMatrix(){ return tangentStiffnessMatrix;};

	 bool getUse_lagrange_multi() const { return use_lagrange_multi; }
	 void setUse_lagrange_multi(bool val) { use_lagrange_multi = val; }
	 bool getStoreLagrangeMultipliers() const { return storeLagrangeMultipliers; }
	 void setStoreLagrangeMultipliers(bool val) { storeLagrangeMultipliers = val; }
	 Eigen::VectorXd getStoredLagrangeMultipliers() const { return storedLagrangeMultipliers; }
	 void setStoredLagrangeMultipliers(Eigen::VectorXd val) { storedLagrangeMultipliers = val; }
protected:

	virtual int doTimeStep2();

	virtual double wolfeconditionFunction(VectorXd &x, VectorXd &eForce);
	virtual double wolfeconditionGradient(VectorXd &x, VectorXd &p, VectorXd &eForce);

	SparseMatrix<double> * rayleighDampingMatrix;
	SparseMatrix<double> * tangentStiffnessMatrix;
	SparseMatrix<double> * systemMatrix;
	SparseMatrix<double> * subSystemMatrix;
	SparseMatrix<double> * constrainedSystemMatrix;
	std::vector<int> mapOldNew;
	std::vector<int> mapSparseMatrixEntryOldNew;

	void updateAlphas();
	void staticTimeStep();
	void staticTimeStepLagrange();
	void staticTimeStepLagrangeV2();

	double NewmarkBeta, NewmarkGamma;
	double alpha1, alpha2, alpha3, alpha4, alpha5, alpha6;
	double epsilon;
	int maxIterations;

	bool use_static_solver;
	bool use_lagrange_multi;
	bool use_lagrange_multi2;

	int numlagrangeConstraint;
	SparseMatrix<double>* lagrangeConstrainMatrix;
	VectorXd constrainTarget;
	VectorXd storedLagrangeMultipliers;
	bool storeLagrangeMultipliers;
	int positiveDefiniteSolver;

	Vegatest* vegatest_;
};


