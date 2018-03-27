#include "SparseIntegratorBase.h"


SparseIntegratorBase::SparseIntegratorBase(int r, double timestep, SparseMatrix<double> *massMatrix, LoboForceModel* forcemodel_, int numConstrainedDOFs, int *constrainedDOFs_, double dampingMassCoef, double dampingStiffnessCoef) :
LoboIntegrator(r,timestep,dampingMassCoef,dampingStiffnessCoef)
{
	this->forcemodel = forcemodel_;
	this->massMatrix = massMatrix;
	this->numConstrainedDOFs = numConstrainedDOFs;

	systemSolveTime = 0.0;
	forceAssemblyTime = 0.0;

	this->constrainedDOFs = (int*)malloc(sizeof(int) * numConstrainedDOFs);
	memcpy(this->constrainedDOFs, constrainedDOFs_, sizeof(int) * numConstrainedDOFs);

	dampingMatrix = new SparseMatrix<double>(r,r);
	tangentStiffnessMatrixOffset = NULL;

}


SparseIntegratorBase::~SparseIntegratorBase()
{
	free(constrainedDOFs);

	delete dampingMatrix;
	delete tangentStiffnessMatrixOffset;
}



void SparseIntegratorBase::setDampingMatrix(SparseMatrix<double> * dampingMatrix_)
{
	delete dampingMatrix;
	dampingMatrix = dampingMatrix_;
}

void SparseIntegratorBase::setTangentStiffnessMatrixOffset(SparseMatrix<double>* tangentStiffnessMatrixOffset_, int reuseTopology)
{
	if (reuseTopology && (tangentStiffnessMatrixOffset != NULL))
		*tangentStiffnessMatrixOffset = *tangentStiffnessMatrixOffset_;
	else
	{
		delete(tangentStiffnessMatrixOffset);
		tangentStiffnessMatrixOffset = new SparseMatrix<double>(*tangentStiffnessMatrixOffset_);
	}
}

void SparseIntegratorBase::AddTangentStiffnessMatrixOffset(SparseMatrix<double>* tangentStiffnessMatrixOffset_)
{
	*tangentStiffnessMatrixOffset += *tangentStiffnessMatrixOffset_;
}

void SparseIntegratorBase::clearTangentStiffnessMatrixOffset()
{
	delete tangentStiffnessMatrixOffset;
	tangentStiffnessMatrixOffset = NULL;
}