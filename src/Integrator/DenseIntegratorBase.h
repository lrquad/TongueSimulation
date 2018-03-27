#pragma once
#include "LoboIntegrator.h"
#include "Simulator\ReducedForceModel\ReducedForceModel.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "Functions/LoboDefine.h"
//#define EIGEN_USE_MKL_ALL

using namespace Eigen;

#pragma once
class DenseIntegratorBase: public LoboIntegrator
{
public:
	DenseIntegratorBase(int r, double timestep, MatrixXd *massMatrix, ReducedForceModel* reducedForceModel, int numConstrainedDOFs = 0 , int *constrainedDOFs_ = NULL,double dampingMassCoef = 0.0, double dampingStiffnessCoef = 0.0);
	~DenseIntegratorBase();

	virtual int doTimeStep() = 0;
	void setMassMatrix(MatrixXd *massMatrix_);


protected:

	ReducedForceModel* reducedforcemodel;
	MatrixXd *massMatrix;
	MatrixXd *dampingMatrix;
	
	int numConstrainedDOFs;
	int * constrainedDOFs;



};

