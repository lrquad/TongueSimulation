#pragma once
#include "ImplicitNewMatkSparseIntegrator.h"

class LoboNeuralNetwork;
class ModalRotationMatrix;
class ImplicitNewMarkDNNIntegrator:public ImplicitNewMatkSparseIntegrator
{
public:
	ImplicitNewMarkDNNIntegrator(ModalRotationMatrix* modalrotationMatrix_, SparseMatrix<double>* modalRotationSparseMatrix_,const char* DNNPath, int r, double timestep, SparseMatrix<double> *massMatrix, LoboForceModel* forcemodel, int positiveDefiniteSolver_, int numConstrainedDOFs = 0, int *constrainedDOFs = NULL, double dampingMassCoef = 0.0, double dampingStiffnessCoef = 0.0, int maxIterations = 1, double epsilon = 1e-6, double NewmarkBeta = 0.25, double NewmarkGama = 0.5, bool usestatic = false);
	~ImplicitNewMarkDNNIntegrator();



protected:

	virtual int doTimeStep2();

	virtual void mapQ(VectorXd &q_);

	LoboNeuralNetwork* loboNeuralNetwork;

	ModalRotationMatrix* modalrotationMatrix;
	SparseMatrix<double>* modalRotationSparseMatrix;


};

