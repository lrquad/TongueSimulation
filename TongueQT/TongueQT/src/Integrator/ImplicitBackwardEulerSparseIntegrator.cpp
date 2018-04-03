#include "ImplicitBackwardEulerSparseIntegrator.h"
#include "SparseMatrix/SparseMatrixRemoveRows.h"


ImplicitBackwardEulerSparseIntegrator::ImplicitBackwardEulerSparseIntegrator(int r, double timestep, SparseMatrix<double>* massMatrix, LoboForceModel* forcemodel, int positiveDefiniteSolver_, int numConstrainedDOFs /*= 0*/, int * constrainedDOFs /*= NULL*/, double dampingMaxxCoef /*= 0.0*/, double dampingStiffnessCoef /*= 0.0*/, int maxIterations /*= 1*/) :SparseIntegratorBase(r,timestep,massMatrix,forcemodel,numConstrainedDOFs,constrainedDOFs,dampingMaxxCoef,dampingStiffnessCoef)
{
	this->maxIterations = maxIterations;
	tangentStiffnessMatrix = new SparseMatrix<double>();
	systemMatrix = new SparseMatrix<double>(r, r);
	forcemodel->getTangentStiffnessMatrixTopology(tangentStiffnessMatrix);
	*systemMatrix = *tangentStiffnessMatrix + *massMatrix;
	subSystemMatrix = new SparseMatrix<double>();

	createMapByConstrains(mapOldNew, r, numConstrainedDOFs, constrainedDOFs);
	createSparseMapbyTopology(systemMatrix, subSystemMatrix, mapSparseMatrixEntryOldNew, mapOldNew, r, numConstrainedDOFs, constrainedDOFs);
}

ImplicitBackwardEulerSparseIntegrator::~ImplicitBackwardEulerSparseIntegrator()
{

}

int ImplicitBackwardEulerSparseIntegrator::setState(double *q, double *qvel /*= NULL*/)
{
	return 0;
}

int ImplicitBackwardEulerSparseIntegrator::doTimeStep()
{
	q_vel = q_vel_1;
	forcemodel->getForceAndMatrix(q.data(), internalForces.data(), tangentStiffnessMatrix);

	internalForces *= internalForceScalingFactor;
	*dampingMatrix = dampingMassCoef* (*massMatrix) + dampingStiffnessCoef*(*tangentStiffnessMatrix);

	*systemMatrix = (*massMatrix + timestep*(*dampingMatrix) + timestep*timestep*(*tangentStiffnessMatrix));
	VectorXd systemF =timestep*(externalForces - internalForces - (timestep*(*tangentStiffnessMatrix) + (*dampingMatrix))*q_vel);
	
	VectorXd subsystemF(r - numConstrainedDOFs);

	SparseMatrixRemoveRows(systemMatrix, subSystemMatrix, mapSparseMatrixEntryOldNew, r, numConstrainedDOFs, constrainedDOFs);
	VectorRemoveRows(mapOldNew, systemF, subsystemF, numConstrainedDOFs, constrainedDOFs);

#ifdef EIGEN_USE_MKL_ALL
	PardisoLDLT<SparseMatrix<double>> LDLT(*subSystemMatrix);
#else
	SimplicialLDLT<SparseMatrix<double>> LDLT(*subSystemMatrix);
#endif
	VectorXd result_ = LDLT.solve(subsystemF);
	this->qdelta.setZero();
	VectorInsertRows(mapOldNew, result_, qdelta, numConstrainedDOFs, constrainedDOFs);

	q_vel_1 = q_vel + qdelta;
	q += q_vel_1*timestep;

	return 0;
}
