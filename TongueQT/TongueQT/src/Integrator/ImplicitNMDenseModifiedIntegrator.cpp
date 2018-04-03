#include "ImplicitNMDenseModifiedIntegrator.h"
#include "SparseMatrix/SparseMatrixRemoveRows.h"
#include <QElapsedTimer>

ImplicitNMDenseModifiedIntegrator::ImplicitNMDenseModifiedIntegrator(int r, double timestep, MatrixXd * massmatrix, ReducedForceModel* reducedforcemodel, int numConstrainedDOFs /*= 0*/, int *constrainedDOFs /*= NULL*/, double dampingMassCoef /*= 0.0*/, double dampingStiffnessCoef /*= 0.0*/, int maxIterations /*= 1*/, double epsilon /*= 1e-6*/, double NewmarkBeta /*= 0.25*/, double NewmarkGama /*= 0.5*/, bool usestatic /*= false*/) :ImpicitNewMarkDenseIntegrator(r,timestep,massmatrix,reducedforcemodel,numConstrainedDOFs,constrainedDOFs,dampingMassCoef,dampingStiffnessCoef,maxIterations,epsilon,NewmarkBeta,NewmarkGama,usestatic)
{
	
}

ImplicitNMDenseModifiedIntegrator::~ImplicitNMDenseModifiedIntegrator()
{
}

void ImplicitNMDenseModifiedIntegrator::updateSubspace(MatrixXd* UUT)
{

	int newr = UUT->rows();
	q = (*UUT)*q;
	q_vel = (*UUT)*q_vel;
	q_accel = (*UUT)*q_accel;

	q_1.resize(newr);
	q_vel_1.resize(newr);
	q_accel_1.resize(newr);

	internalForces.resize(newr);
	externalForces.resize(newr);

	q_residual.resize(newr);
	qdelta.resize(newr);

	r = newr;
	systemMatrix->resize(r, r);
	tangentStiffnessMatrix->resize(r, r);

	createMapByConstrains(mapOldNew, r, numConstrainedDOFs, constrainedDOFs);
}

bool ImplicitNMDenseModifiedIntegrator::updateSubspaceLeastSquare(MatrixXd* W, MatrixXd* U)
{
	QElapsedTimer timer;
	timer.start();
	std::cout << "compute least square" << std::endl;

	int newr = W->cols();
	MatrixXd WWT = (*W).transpose()*(*W);
	Eigen::LDLT<MatrixXd> solver(WWT);
	MatrixXd WU = (*W).transpose()*(*U);
	VectorXd righhand;
	righhand = WU*q;
	//VectorXd q_temp = (*U)*q;
	VectorXd q_temp2 = solver.solve(righhand);
	//double error = ((*W)*q_temp2 - q_temp).norm() / q_temp.norm();
	
	q = q_temp2;
	righhand = WU*q_vel;
	q_vel = solver.solve(righhand);
	righhand = WU*q_accel;
	q_accel = solver.solve(righhand);
	std::cout << "finished least square"<<timer.elapsed() << std::endl;

	q_1.resize(newr);
	q_vel_1.resize(newr);
	q_accel_1.resize(newr);

	internalForces.resize(newr);
	internalForces.setZero();

	externalForces.resize(newr);
	externalForces.setZero();

	q_residual.resize(newr);
	q_residual.setZero();

	qdelta.resize(newr);
	qdelta.setZero();

	r = newr;
	systemMatrix->resize(r, r);
	tangentStiffnessMatrix->resize(r, r);

	std::cout << "start create map by constrains" << timer.elapsed() << std::endl;

	createMapByConstrains(mapOldNew, r, numConstrainedDOFs, constrainedDOFs);

	std::cout << "resize matrix" << timer.elapsed() << std::endl;
	return true;
}
