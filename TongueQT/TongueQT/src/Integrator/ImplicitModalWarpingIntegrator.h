#pragma once
#include "ImplicitNewMatkSparseIntegrator.h"


class ModalRotationMatrix;
class ImplicitModalWarpingIntegrator :public ImplicitNewMatkSparseIntegrator
{
public:
	ImplicitModalWarpingIntegrator(ModalRotationMatrix* modalrotationMatrix_, SparseMatrix<double>* modalRotationSparseMatrix_, int r, double timestep, SparseMatrix<double> *massMatrix, LoboForceModel* forcemodel, int positiveDefiniteSolver_, int numConstrainedDOFs = 0, int *constrainedDOFs = NULL, double dampingMassCoef = 0.0, double dampingStiffnessCoef = 0.0, int maxIterations = 1, double epsilon = 1e-6, double NewmarkBeta = 0.25, double NewmarkGama = 0.5, bool usestatic = false);

	~ImplicitModalWarpingIntegrator();

	virtual void updateMaterial();


	virtual Eigen::VectorXd getInteranlForce(VectorXd q_);

	virtual bool computeStaticDisplacement(VectorXd &externalForce, bool checkwolfecondition = false);


	virtual void predictQdot(VectorXd &q_, VectorXd &q_1_, VectorXd &q_dot_1, VectorXd &q_ac_1, VectorXd &result);
	virtual void predictQacc(VectorXd &q_, VectorXd &q_1_, VectorXd &q_dot_1, VectorXd &q_ac_1, VectorXd &result);


protected:

	virtual int doTimeStep2();
	virtual void newmarkLinear();

	bool computedMatrix_b;

	SimplicialLDLT<SparseMatrix<double>> LDLT;
	//SparseLU<SparseMatrix<double>> LDLT;

#ifdef EIGEN_USE_MKL_ALL
	//PardisoLDLT<SparseMatrix<double>> LDLT;
#else
	
#endif

	SparseMatrix<double>* modalRotationSparseMatrix;
	SparseMatrix<double>* localOrientationMatrixR;
	ModalRotationMatrix* modalrotationMatrix;
};

