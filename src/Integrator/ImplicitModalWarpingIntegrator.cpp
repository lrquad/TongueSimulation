#include "ImplicitModalWarpingIntegrator.h"
#include "SparseMatrix/SparseMatrixRemoveRows.h"
#include <fstream>
#include <iostream>
#include "vegatest.h"
#include <iomanip> 
#include "Simulator/DeepWarp/ModalRotationMatrix.h"
#include <QElapsedTimer>
ImplicitModalWarpingIntegrator::ImplicitModalWarpingIntegrator(ModalRotationMatrix* modalrotationMatrix_, SparseMatrix<double>* modalRotationSparseMatrix_, int r, double timestep, SparseMatrix<double> *massMatrix, LoboForceModel* forcemodel, int positiveDefiniteSolver_, int numConstrainedDOFs /*= 0*/, int *constrainedDOFs /*= NULL*/, double dampingMassCoef /*= 0.0*/, double dampingStiffnessCoef /*= 0.0*/, int maxIterations /*= 1*/, double epsilon /*= 1e-6*/, double NewmarkBeta /*= 0.25*/, double NewmarkGama /*= 0.5*/, bool usestatic /*= false*/) :ImplicitNewMatkSparseIntegrator(r, timestep, massMatrix, forcemodel, positiveDefiniteSolver_, numConstrainedDOFs, constrainedDOFs, dampingMassCoef, dampingStiffnessCoef, maxIterations, epsilon, NewmarkBeta, NewmarkGama, usestatic)
{
	this->modalrotationMatrix = modalrotationMatrix_;
	this->modalRotationSparseMatrix = modalRotationSparseMatrix_;
	localOrientationMatrixR = new SparseMatrix<double>();
	computedMatrix_b = false;
}

ImplicitModalWarpingIntegrator::~ImplicitModalWarpingIntegrator()
{
	delete localOrientationMatrixR;
}

void ImplicitModalWarpingIntegrator::updateMaterial()
{
	
	forcemodel->updateMaterial();
	if (forcemodel->getUselinearStiffnessMatrix())
	{
		q.setZero();
		forcemodel->computeLinearStiffnessMatrix(q.data(), tangentStiffnessMatrix);
	}
	computedMatrix_b = false;
}

Eigen::VectorXd ImplicitModalWarpingIntegrator::getInteranlForce(VectorXd q_)
{
	//forcemodel->getForceAndMatrix(q_.data(), internalForces.data(), tangentStiffnessMatrix);
	internalForces = *tangentStiffnessMatrix*q_;
	return internalForces;
}

bool ImplicitModalWarpingIntegrator::computeStaticDisplacement(VectorXd &externalForce, bool checkwolfecondition /*= false*/)
{
	//wolfe conditions
	double c1 = 1e-4;
	double c2 = 0.9;


	int numIter = 0;

	double error0 = 0; // error after the first step
	double errorQuotient;
	double preerror = DBL_MAX;
	double error = 0;

	int maxwolfe = 10;

	q_1 = q;
	std::cout << std::endl;
	do{
		int i;
		forcemodel->getForceAndMatrix(q.data(), internalForces.data(), tangentStiffnessMatrix);

		VectorXd w = (*modalRotationSparseMatrix)*q;
		modalrotationMatrix->computeLocalOrientationMatrixR(localOrientationMatrixR, w);

		VectorXd modalForce = (*localOrientationMatrixR)*externalForce;

		internalForces *= internalForceScalingFactor;

		*systemMatrix = (*tangentStiffnessMatrix);

		VectorXd systemF =
			this->internalForces
			- modalForce
			;

		systemF *= -1;
		error = 0;
		error = systemF.norm()*systemF.norm();


		if (numIter == 0)
		{
			error0 = error;
			errorQuotient = 1;
		}
		else
		{
			errorQuotient = error / error0;
		}


		if (std::abs(error - preerror) / std::abs(preerror) < 1e-5)
		{

			break;
		}

		if (std::abs(error) < 1e-15)
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

		VectorXd subsystemF(r - numConstrainedDOFs);

		SparseMatrixRemoveRows(systemMatrix, subSystemMatrix, mapSparseMatrixEntryOldNew, r, numConstrainedDOFs, constrainedDOFs);
		VectorRemoveRows(mapOldNew, systemF, subsystemF, numConstrainedDOFs, constrainedDOFs);

		srand(1);
#ifdef EIGEN_USE_MKL_ALL
		PardisoLDLT<SparseMatrix<double>> LDLT;
#else
		SimplicialLDLT<SparseMatrix<double>> LDLT;
#endif
		LDLT.pardisoParameterArray()[33] = 1;
		LDLT.pardisoParameterArray()[59] = 0;

		LDLT.compute(*subSystemMatrix);
		//SimplicialLDLT<SparseMatrix<double>> LDLT(*subSystemMatrix);
		VectorXd result_ = LDLT.solve(subsystemF);

		this->qdelta.setZero();
		VectorInsertRows(mapOldNew, result_, qdelta, numConstrainedDOFs, constrainedDOFs);
		//check wolf condition
		double alpha = qdelta.norm();
		VectorXd direction = qdelta.normalized();
		bool wolfe1 = true;
		bool wolfe2 = true;

		int numtry = 0;
		if (checkwolfecondition)
		{
			do{
				VectorXd potentialq = q + alpha*direction;
				double xP = wolfeconditionFunction(potentialq, externalForce);
				double x = wolfeconditionFunction(q, externalForce);
				double pf = wolfeconditionGradient(q, direction, externalForce);
				double pfx = wolfeconditionGradient(potentialq, direction, externalForce);

				wolfe1 = true;
				wolfe2 = true;

				if (xP > x + c1*alpha*pf)
				{
					wolfe1 = false;
				}

				if (-pfx > -c2*pf)
				{
					wolfe2 = false;
				}

				if (!(wolfe1&&wolfe2))
				{
					alpha *= 0.5;
				}
				numtry++;

			} while (!(wolfe1 && wolfe2)&&numtry<maxwolfe);
		}

		q += alpha*direction;

		numIter++;
		std::cout << "\r";
		std::cout << numIter;
	} while (numIter < maxIterations);
	std::cout << std::endl;
	q_vel.setZero();
	q_accel.setZero();

	std::cout << "error0: " << error0 << " eror: " << error << " errorQuotient: " << errorQuotient << std::endl;
	std::cout << "numIter -> " << numIter << std::endl;
	return true;
}

void ImplicitModalWarpingIntegrator::predictQdot(VectorXd &q_, VectorXd &q_1_, VectorXd &q_dot_1, VectorXd &q_ac_1, VectorXd &result)
{
	result = alpha4*(q_ - q_1_) + alpha5*q_dot_1 + alpha6*q_ac_1;
}

void ImplicitModalWarpingIntegrator::predictQacc(VectorXd &q_, VectorXd &q_1_, VectorXd &q_dot_1, VectorXd &q_ac_1, VectorXd &result)
{
	result = alpha1*(q_ - q_1_) - alpha2*q_dot_1 - alpha3*q_ac_1;
}

int ImplicitModalWarpingIntegrator::doTimeStep2()
{
	//doTimeStepv2();
	//return 0;

	newmarkLinear();
	return 0;

	int numIter = 0;

	double error0 = 0; // error after the first step
	double errorQuotient;
	double preerror = DBL_MAX;
	q_1 = q;
	q_vel_1 = q_vel;
	q_accel_1 = q_accel;

	q_accel = alpha1*(q - q_1) - alpha2*q_vel_1 - alpha3*q_accel_1;
	q_vel = alpha4*(q - q_1) + alpha5*q_vel_1 + alpha6 * q_accel_1;

	VectorXd w = (*modalRotationSparseMatrix)*q;
	modalrotationMatrix->computeLocalOrientationMatrixR(localOrientationMatrixR, w);
	//modalrotationMatrix->computeLocalOrientationByPolarDecomposition(localOrientationMatrixR, q);
	/*modalrotationMatrix->computeLocalOrientationByPolarDecomposition(localOrientationMatrixR, q);*/
	
	VectorXd modalForce = (*localOrientationMatrixR)*externalForces;

	//no modal wapring
	//modalForce = externalForces;

	do{
		int i;

		forcemodel->getForceAndMatrix(q.data(), internalForces.data(), tangentStiffnessMatrix);

		internalForces *= internalForceScalingFactor;

		*dampingMatrix = dampingMassCoef* (*massMatrix) + dampingStiffnessCoef*(*tangentStiffnessMatrix);

		*systemMatrix = alpha1*(*massMatrix) + alpha4*(*dampingMatrix) + (*tangentStiffnessMatrix);

		/*VectorXd w = (*modalRotationSparseMatrix)*q;
		modalrotationMatrix->computeLocalOrientationByPolarDecomposition(localOrientationMatrixR, w);
		VectorXd modalForce = (*localOrientationMatrixR)*externalForces;*/

		VectorXd systemF = (*massMatrix)*(q_accel)
			+(*dampingMatrix)*(q_vel)
			+this->internalForces
			- modalForce
			;

		systemF *= -1;
		double error = 0;
		error = systemF.norm()*systemF.norm();

		if (numIter == 0)
		{
			error0 = error;
			errorQuotient = 1;
		}
		else
		{
			errorQuotient = error / error0;
		}

		//std::cout << "residual error: " << std::abs(error - preerror) / std::abs(error) << std::endl;

		if (std::abs(error - preerror) / std::abs(error) < 1e-4)
		{
			break;
		}

		preerror = error;

		if (errorQuotient < epsilon*epsilon)
		{
			break;
		}

		if (std::abs(error) < 1e-15)
		{
			break;
		}

		VectorXd subsystemF(r - numConstrainedDOFs);

		SparseMatrixRemoveRows(systemMatrix, subSystemMatrix, mapSparseMatrixEntryOldNew, r, numConstrainedDOFs, constrainedDOFs);
		VectorRemoveRows(mapOldNew, systemF, subsystemF, numConstrainedDOFs, constrainedDOFs);

		srand(1);
#ifdef EIGEN_USE_MKL_ALL
		PardisoLDLT<SparseMatrix<double>> LDLT;
#else
		SimplicialLDLT<SparseMatrix<double>> LDLT;
#endif
		LDLT.pardisoParameterArray()[33] = 1;
		LDLT.pardisoParameterArray()[59] = 0;

		LDLT.compute(*subSystemMatrix);
		//SimplicialLDLT<SparseMatrix<double>> LDLT(*subSystemMatrix);
		VectorXd result_ = LDLT.solve(subsystemF);

		this->qdelta.setZero();
		VectorInsertRows(mapOldNew, result_, qdelta, numConstrainedDOFs, constrainedDOFs);

		q += qdelta;
		q_accel = alpha1*(q - q_1) - alpha2*q_vel_1 - alpha3*q_accel_1;
		q_vel = alpha4*(q - q_1) + alpha5*q_vel_1 + alpha6*q_accel_1;

		//std::cout << std::setprecision(64)<<subSystemMatrix->norm()<<std::endl << subsystemF.norm() << " " << q.norm() << std::endl;
		
		numIter++;

	} while (numIter < maxIterations);
	

	if (getSaveStepResidual())
	{
		/*VectorXd w = (*modalRotationSparseMatrix)*q;
		modalrotationMatrix->computeLocalOrientationMatrixR(localOrientationMatrixR, w);
		VectorXd modalForce = (*localOrientationMatrixR)*externalForces;

		step_residual = (*massMatrix)*(q_accel)
		+(*dampingMatrix)*(q_vel)
		- modalForce
		;*/
		//step_residual *= -1;
	}

	if (getRecordLoadedStepResults())
	{
		sequence_q.push_back(q);
	}

	return 0;
}

void ImplicitModalWarpingIntegrator::newmarkLinear()
{
	VectorXd w = (*modalRotationSparseMatrix)*q;
	modalrotationMatrix->computeLocalOrientationMatrixR(localOrientationMatrixR, w);
	VectorXd modalForce = (*localOrientationMatrixR)*externalForces;
	//modalForce = externalForces;
	//forcemodel->getForceAndMatrix(q.data(), internalForces.data(), tangentStiffnessMatrix);

	if (!computedMatrix_b)
	{
		forcemodel->getForceAndMatrix(q.data(), internalForces.data(), tangentStiffnessMatrix);

		*dampingMatrix = dampingMassCoef* (*massMatrix) + dampingStiffnessCoef*(*tangentStiffnessMatrix);

		//srand(1);
		*systemMatrix = (*massMatrix)*alpha1 + (*dampingMatrix)*alpha4 + *tangentStiffnessMatrix;
		SparseMatrixRemoveRows(systemMatrix, subSystemMatrix, mapSparseMatrixEntryOldNew, r, numConstrainedDOFs, constrainedDOFs);
		//LDLT.pardisoParameterArray()[33] = 1;
		//LDLT.pardisoParameterArray()[59] = 0;
		QElapsedTimer timer;
		timer.start();
		LDLT.compute(*subSystemMatrix);
		std::cout <<"compute time: "<< timer.nsecsElapsed() / 1e6 << "ms" << std::endl;

		computedMatrix_b = true;
	}
	else
	{
		internalForces = *tangentStiffnessMatrix*q;
	}

	VectorXd systemF = modalForce - internalForces
		- (*dampingMatrix)*(alpha5*q_vel_1 + alpha6*q_accel_1) - (*massMatrix)*(-alpha2*q_vel_1 - alpha3*q_accel_1)
		;

	VectorXd subsystemF(r - numConstrainedDOFs);

	VectorRemoveRows(mapOldNew, systemF, subsystemF, numConstrainedDOFs, constrainedDOFs);

	//SimplicialLDLT<SparseMatrix<double>> LDLT(*subSystemMatrix);
	
	QElapsedTimer timer;
	timer.start();
	VectorXd result_ = LDLT.solve(subsystemF);

	if (getShowPerformance())
	{
		std::cout << timer.nsecsElapsed()/1e6 << "ms" << std::endl;
	}

	this->qdelta.setZero();
	VectorInsertRows(mapOldNew, result_, qdelta, numConstrainedDOFs, constrainedDOFs);

	q += qdelta;
	q_accel = alpha1*(q - q_1) - alpha2*q_vel_1 - alpha3*q_accel_1;
	q_vel = alpha4*(q - q_1) + alpha5*q_vel_1 + alpha6*q_accel_1;
	q_accel_1 = q_accel;
	q_vel_1 = q_vel;
	q_1 = q;

	if (getSaveStepResidual())
	{

	}

	if (getRecordLoadedStepResults())
	{
		sequence_q.push_back(q);
	}

}
