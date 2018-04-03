#include "ImpicitNewMarkDenseIntegrator.h"
#include "SparseMatrix/SparseMatrixRemoveRows.h"
#include <fstream>
#include <QElapsedTimer>

ImpicitNewMarkDenseIntegrator::ImpicitNewMarkDenseIntegrator(int r, double timestep, MatrixXd * massmatrix, ReducedForceModel* reducedforcemodel, int numConstrainedDOFs, int *constrainedDOFs, double dampingMassCoef, double dampingStiffnessCoef, int maxIterations, double epsilon, double NewmarkBeta, double NewmarkGamma, bool usestatic) : DenseIntegratorBase(r, timestep, massmatrix, reducedforcemodel, numConstrainedDOFs, constrainedDOFs, dampingMassCoef, dampingStiffnessCoef)
{
	this->maxIterations = maxIterations;
	this->epsilon = epsilon;
	
	this->NewmarkBeta = NewmarkBeta;
	this->NewmarkGamma = NewmarkGamma;

	systemMatrix = new MatrixXd(r, r);
	tangentStiffnessMatrix = new MatrixXd(r, r);

	setUseLagrangeMulti(false);
	
	createMapByConstrains(mapOldNew, r, numConstrainedDOFs, constrainedDOFs);
	updateAlphas();

}


ImpicitNewMarkDenseIntegrator::~ImpicitNewMarkDenseIntegrator()
{

}

void ImpicitNewMarkDenseIntegrator::updateMaterial()
{

}

bool ImpicitNewMarkDenseIntegrator::computeStaticDisplacement(VectorXd &externalForce, bool checkwolfecondition /*= false*/)
{
	//wolfe conditions
	double c1 = 1e-4;
	double c2 = 0.9;


	int numIter = 0;

	double error0 = 0; // error after the first step
	double errorQuotient;
	double preerror = DBL_MAX;
	double error = 0;
	q_1 = q;

	//std::cout << std::endl;
	do{
		int i;
		reducedforcemodel->getForceAndMatrix(q.data(), internalForces.data(), tangentStiffnessMatrix);
		//internalForces *= internalForceScalingFactor;
		*systemMatrix = (*tangentStiffnessMatrix);

		VectorXd systemF =
			this->internalForces
			- externalForce
			;

		systemF *= -1;
		error = 0;
		error = systemF.norm()*systemF.norm();

		//std::cout << error << std::endl;

		if (numIter == 0)
		{
			error0 = error;
			errorQuotient = 1;
		}
		else
		{
			errorQuotient = error / error0;
		}


		if (std::abs(error - preerror) / std::abs(preerror) < 1e-8)
		{
			break;
		}

		/*if (error > preerror)
		{
		break;
		}*/

		preerror = error;

		if (errorQuotient < epsilon*epsilon)
		{
			break;
		}

		if (error < 1e-15)
		{
			break;
		}

		if (error != error)
		{
			return false;
			break;
		}

		qdelta = systemMatrix->selfadjointView<Upper>().ldlt().solve(systemF);

		
		double alpha = qdelta.norm();
		VectorXd direction = qdelta.normalized();

		int numtry = 0;
		
		q += alpha*direction;

		numIter++;
		//std::cout << "\r";
		//std::cout << numIter;
	} while (numIter < maxIterations);
	//std::cout << std::endl;
	q_vel.setZero();
	q_accel.setZero();


	if (numIter >= maxIterations)
	{
		return false;
	}

	return true;
}

void ImpicitNewMarkDenseIntegrator::updateAlphas()
{
	alpha1 = 1.0 / (NewmarkBeta * timestep * timestep);
	alpha2 = 1.0 / (NewmarkBeta * timestep);
	alpha3 = (1.0 - 2.0 * NewmarkBeta) / (2.0 * NewmarkBeta);
	alpha4 = NewmarkGamma / (NewmarkBeta * timestep);
	alpha5 = 1 - NewmarkGamma / NewmarkBeta;
	alpha6 = (1.0 - NewmarkGamma / (2.0 * NewmarkBeta)) * timestep;
}

int ImpicitNewMarkDenseIntegrator::setState(double *q, double *qvel)
{
	memcpy(this->q.data(), q, r*sizeof(double));
	memcpy(this->q_vel.data(), qvel, r*sizeof(double));

	return 0;
}

int ImpicitNewMarkDenseIntegrator::doTimeStep()
{
	if (getUseLoadedStepResults())
	{
		if (step_*skipSteps<sequence_q.size())
			q = sequence_q[step_ * skipSteps];
		step_++;
		return 1;
	}

	if (getUseLagrangeMulti())
	{
		doTimeStepWithLagrangeMulti();
		return 1;
	}

	this->q_1 = q;

	double error;
	double error0;
	double errorQuotient;
	double preerror = DBL_MAX;
	q_1 = q;
	q_vel_1 = q_vel;
	q_accel_1 = q_accel;

	q_accel = alpha1*(q - q_1) - alpha2*q_vel_1 - alpha3*q_accel_1;
	q_vel = alpha4*(q - q_1) + alpha5*q_vel_1 + alpha6 * q_accel_1;
	int numIter = 0;
	for (int i = 0; i < maxIterations; i++)
	{
		reducedforcemodel->getForceAndMatrix(q.data(), internalForces.data(), tangentStiffnessMatrix);
	/*	if (step_ >= 390)
		{
			internalForces *= std::rand() % 200;
		}*/

		internalForces *= internalForceScalingFactor;

		*dampingMatrix = dampingMassCoef*(*massMatrix) + dampingStiffnessCoef*(*tangentStiffnessMatrix);

		*systemMatrix = alpha1*(*massMatrix) + alpha4*(*dampingMatrix) + (*tangentStiffnessMatrix);

		VectorXd systemF = (*massMatrix)*(q_accel)
			+(*dampingMatrix)*(q_vel)
			+ this->internalForces
			- this->externalForces
			;

		systemF *= -1;

		error = systemF.norm()*systemF.norm();
		if (i == 0)
		{
			error0 = error;
			errorQuotient = 1;
		}
		else
		{
			errorQuotient = error / error0;
		}

		if (errorQuotient < epsilon*epsilon)
		{
			break;
		}

		if (std::abs(error - preerror) / std::abs(error) < 1e-4)
		{
			break;
		}

		preerror = error;


		if (numConstrainedDOFs != 0)
		{
			dotimestepwithConstrainedDOFs(systemF);
		}
		else
		{
			dotimestepwithoutConstrainedDOFs(systemF);
		}
		numIter++;
	}

	if (getRecordLoadedStepResults())
	{
		sequence_q.push_back(q);
	}

	step_++;
	return 1;
}

int ImpicitNewMarkDenseIntegrator::doTimeStepWithLagrangeMulti()
{
	this->q_1 = q;

	double error;
	double error0;
	double errorQuotient;
	q_1 = q;
	q_vel_1 = q_vel;
	q_accel_1 = q_accel;

	q_accel = alpha1*(q - q_1) - alpha2*q_vel_1 - alpha3*q_accel_1;
	q_vel = alpha4*(q - q_1) + alpha5*q_vel_1 + alpha6 * q_accel_1;
	double preerror = DBL_MAX;
	for (int i = 0; i < maxIterations; i++)
	{
		reducedforcemodel->getForceAndMatrix(q.data(), internalForces.data(), tangentStiffnessMatrix);
		internalForces *= internalForceScalingFactor;
		*dampingMatrix = dampingMassCoef*(*massMatrix) + dampingStiffnessCoef*(*tangentStiffnessMatrix);

		*systemMatrix = alpha1*(*massMatrix) + alpha4*(*dampingMatrix) + (*tangentStiffnessMatrix);

		VectorXd systemF = (*massMatrix)*q_accel
			+ (*dampingMatrix)*q_vel
			+ this->internalForces
			- this->externalForces
			;

		systemF *= -1;

		assginConstrainedSystem(systemF);

		error = lagrangesystemVector.norm()*lagrangesystemVector.norm();

		if (i == 0)
		{
			error0 = error;
			errorQuotient = 1;
		}
		else
		{
			errorQuotient = error / error0;
		}
		if (std::abs(error - preerror) / std::abs(error) < 1e-4)
		{
			break;
		}
		preerror = error;

		if (errorQuotient < epsilon*epsilon)
		{
			break;
		}

		VectorXd result;
#ifdef EIGEN_USE_MKL_ALL
		Eigen::LDLT	<MatrixXd> solver(lagrangesystemMatrix);
		result = solver.solve(lagrangesystemVector);
#else
		Eigen::LDLT<MatrixXd> solver(lagrangesystemMatrix);
		if (solver.info() != Success)
		{
			std::cout << "failed" << std::endl;
		}
		result = solver.solve(lagrangesystemVector);
#endif

		for (int i = 0; i < systemMatrix->rows(); i++)
		{
			qdelta.data()[i] = result.data()[i];
		}

		VectorXd lagrange(reducedConstrainVector.rows());
		for (int i = 0; i < lagrange.rows(); i++)
		{
			lagrange.data()[i] = result.data()[i + systemMatrix->rows()];
			reducedConstrainVector[i] -= lagrange[i];
		}
		
		q += qdelta;
		q_accel = alpha1*(q - q_1) - alpha2*q_vel_1 - alpha3*q_accel_1;
		q_vel = alpha4*(q - q_1) + alpha5*q_vel_1 + alpha6*q_accel_1;
	}

	if (getRecordLoadedStepResults())
	{
		sequence_q.push_back(q);
	}

	step_++;
	return 1;
}

int ImpicitNewMarkDenseIntegrator::doTimeStepWithPanelty()
{
	this->q_1 = q;

	double error;
	double error0;
	double errorQuotient;

	for (int i = 0; i < maxIterations; i++)
	{

		reducedforcemodel->getForceAndMatrix(q.data(), internalForces.data(), tangentStiffnessMatrix);
		internalForces *= internalForceScalingFactor;
		*dampingMatrix = dampingMassCoef*(*massMatrix) + dampingStiffnessCoef*(*tangentStiffnessMatrix);

		*systemMatrix = alpha1*(*massMatrix) + alpha4*(*dampingMatrix) + (*tangentStiffnessMatrix);

		VectorXd systemF = (*massMatrix)*(alpha1*(q_1 - q) - alpha2*q_vel - alpha3*q_accel)
			+ (*dampingMatrix)*(alpha4*(q_1 - q) + alpha5*q_vel + alpha6*q_accel)
			+ this->internalForces
			- this->externalForces
			;

		systemF *= -1;

		error = systemF.norm()*systemF.norm();

		if (i == 0)
		{
			error0 = error;
			errorQuotient = 1;
		}
		else
		{
			errorQuotient = error / error0;
		}

		if (errorQuotient < epsilon*epsilon)
		{
			break;
		}

		//assginConstrainedSystem(systemF);

		lagrangesystemMatrix = systemMatrix->transpose()*(*systemMatrix)
			+ 10000 * reducedConstrainedMatrix.transpose()*reducedConstrainedMatrix;

		lagrangesystemVector = systemMatrix->transpose()*systemF
			+ 10000 * reducedConstrainedMatrix.transpose()*reducedConstrainVector;

		VectorXd result;
#ifdef EIGEN_USE_MKL_ALL
		Eigen::LDLT<MatrixXd> solver(lagrangesystemMatrix);
		result = solver.solve(lagrangesystemVector);
#else
		Eigen::LDLT<MatrixXd> solver(lagrangesystemMatrix);
		if (solver.info() != Success)
		{
			std::cout << "failed" << std::endl;
		}
		result = solver.solve(lagrangesystemVector);
#endif
		for (int i = 0; i < systemMatrix->rows(); i++)
		{
			qdelta.data()[i] = result.data()[i];
		}

		/*VectorXd lagrange(reducedConstrainVector.rows());
		for (int i = 0; i < lagrange.rows(); i++)
		{
			lagrange.data()[i] = result.data()[i + systemMatrix->rows()];
		}*/

		q_1 += qdelta;
	}

	q_vel_1 = alpha4*(q_1 - q) + alpha5*q_vel + alpha6*q_accel;
	q_accel_1 = alpha1*(q_1 - q) - alpha2*q_vel - alpha3*q_accel;

	q_vel = q_vel_1;
	q_accel = q_accel_1;
	q = q_1;

	step_++;
	return 1;
}

void ImpicitNewMarkDenseIntegrator::assginConstrainedSystem(VectorXd &systemF)
{
	int newr = systemMatrix->rows() + reducedConstrainedMatrix.rows();
	lagrangesystemMatrix.resize(newr, newr);
	lagrangesystemVector.resize(newr, newr);
	lagrangesystemMatrix.setZero();
	lagrangesystemVector.setZero();

	lagrangesystemMatrix.block(0, 0, systemMatrix->rows(), systemMatrix->rows())
		= *systemMatrix;
	lagrangesystemMatrix.block(systemMatrix->rows(), 0, reducedConstrainedMatrix.rows(), reducedConstrainedMatrix.cols())
		= reducedConstrainedMatrix;
	lagrangesystemMatrix.block(0, systemMatrix->rows(), reducedConstrainedMatrix.cols(), reducedConstrainedMatrix.rows())
		= reducedConstrainedMatrix.transpose();

	for (int i = 0; i < systemMatrix->rows(); i++)
	{
		lagrangesystemVector.data()[i] = systemF.data()[i];
	}

	for (int i = 0; i < reducedConstrainVector.rows(); i++)
	{
		lagrangesystemVector.data()[i + systemMatrix->rows()] = reducedConstrainVector.data()[i];
	}
}

void ImpicitNewMarkDenseIntegrator::setConstrainByLagrangeMulti(MatrixXd &reducedConstrainedMatrix, VectorXd &reducedConstrainVector)
{
	if (reducedConstrainedMatrix.rows() == 0)
	{
		setUseLagrangeMulti(false);
		return;
	}

	setUseLagrangeMulti(true);

	this->reducedConstrainedMatrix = reducedConstrainedMatrix;
	this->reducedConstrainVector = reducedConstrainVector;

}

void ImpicitNewMarkDenseIntegrator::dotimestepwithConstrainedDOFs(VectorXd &systemF)
{
	VectorXd subsystemF(r - numConstrainedDOFs);
	VectorRemoveRows(mapOldNew, systemF, subsystemF, numConstrainedDOFs, constrainedDOFs);
	MatrixRemoveDofs(mapOldNew, *systemMatrix, *subSystemMatrix);

#ifdef EIGEN_USE_MKL_ALL
	VectorXd result_ = subSystemMatrix->selfadjointView<Upper>().ldlt().solve(subsystemF);
#else
	Eigen::LDLT<MatrixXd> solver(*subSystemMatrix);
	if (solver.info() != Success)
	{
		std::cout << "failed" << std::endl;
	}
	VectorXd result_ = solver.solve(subsystemF);
#endif
	this->qdelta.setZero();
	VectorInsertRows(mapOldNew, result_, qdelta, numConstrainedDOFs, constrainedDOFs);

	q += qdelta;
	q_accel = alpha1*(q - q_1) - alpha2*q_vel_1 - alpha3*q_accel_1;
	q_vel = alpha4*(q - q_1) + alpha5*q_vel_1 + alpha6*q_accel_1;

}

void ImpicitNewMarkDenseIntegrator::dotimestepwithoutConstrainedDOFs(VectorXd &systemF)
{
#ifdef EIGEN_USE_MKL_ALL
	qdelta = systemMatrix->selfadjointView<Upper>().ldlt().solve(systemF);
#else
	Eigen::LDLT<MatrixXd> solver(*systemMatrix);
	if (solver.info() != Success)
	{
		std::cout << "failed" << std::endl;
	}
	qdelta = solver.solve(systemF);
#endif

	q += qdelta;
	q_accel = alpha1*(q - q_1) - alpha2*q_vel_1 - alpha3*q_accel_1;
	q_vel = alpha4*(q - q_1) + alpha5*q_vel_1 + alpha6*q_accel_1;

}
