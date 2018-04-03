#pragma once
#include "LoboIntegrator.h"
#include "Simulator/ForceModel/LoboForceModel.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/PardisoSupport>
#include "Functions/LoboDefine.h"
//#define EIGEN_USE_MKL_ALL

using namespace Eigen;
class SparseIntegratorBase :public LoboIntegrator
{
public:
	SparseIntegratorBase(int r, double timestep, SparseMatrix<double> *massMatrix, LoboForceModel* simulator, int numConstraintedDOFs = 0, int *constraintedDOFs = NULL, double dampingMassCoef = 0.0, double dampingStiffnessCoef = 0.0);
	~SparseIntegratorBase();

	inline virtual void setForceModel(LoboForceModel* forcemodel){ this->forcemodel = forcemodel; };



	virtual void setDampingMatrix(SparseMatrix<double>* dampingMatirx);

	virtual void setTangentStiffnessMatrixOffset(SparseMatrix<double>* tangentStiffnessMatrixOffset,int reuseTopology = 1);

	virtual void AddTangentStiffnessMatrixOffset(SparseMatrix<double>* tangentStiffnessMatrixOffset);

	virtual void clearTangentStiffnessMatrixOffset();

	virtual int doTimeStep() = 0;

	inline virtual double getSystemSolveTime(){ return systemSolveTime; }
	inline virtual double getForceAssemblyTime(){ return forceAssemblyTime; }

protected:
	SparseMatrix<double>* massMatrix;
	LoboForceModel *forcemodel;
	SparseMatrix<double>* dampingMatrix;
	SparseMatrix<double>* tangentStiffnessMatrixOffset;

	int numConstrainedDOFs;
	int * constrainedDOFs;

	double systemSolveTime;
	double forceAssemblyTime;

};

