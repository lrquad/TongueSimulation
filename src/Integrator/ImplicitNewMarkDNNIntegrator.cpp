#include "ImplicitNewMarkDNNIntegrator.h"
#include "LoboNeuralNetwork/LoboNeuralNetwork.h"
#include "SparseMatrix/SparseMatrixRemoveRows.h"
#include <fstream>
#include <iostream>
#include "vegatest.h"
#include <iomanip> 
#include "Simulator/DeepWarp/ModalRotationMatrix.h"

ImplicitNewMarkDNNIntegrator::ImplicitNewMarkDNNIntegrator(ModalRotationMatrix* modalrotationMatrix_, SparseMatrix<double>* modalRotationSparseMatrix_, const char* DNNPath, int r, double timestep, SparseMatrix<double> *massMatrix, LoboForceModel* forcemodel, int positiveDefiniteSolver_, int numConstrainedDOFs /*= 0*/, int *constrainedDOFs /*= NULL*/, double dampingMassCoef /*= 0.0*/, double dampingStiffnessCoef /*= 0.0*/, int maxIterations /*= 1*/, double epsilon /*= 1e-6*/, double NewmarkBeta /*= 0.25*/, double NewmarkGama /*= 0.5*/, bool usestatic /*= false*/) :ImplicitNewMatkSparseIntegrator(r, timestep, massMatrix, forcemodel, positiveDefiniteSolver_, numConstrainedDOFs, constrainedDOFs, dampingMassCoef, dampingStiffnessCoef, maxIterations,epsilon,NewmarkBeta,NewmarkGama,usestatic)
{
	loboNeuralNetwork = new LoboNeuralNetwork();
	loboNeuralNetwork->loadDNNV2(DNNPath);
	//assume use linear integration
	setMaxInteration(1);
	this->modalrotationMatrix = modalrotationMatrix_;
	this->modalRotationSparseMatrix = modalRotationSparseMatrix_;

}

ImplicitNewMarkDNNIntegrator::~ImplicitNewMarkDNNIntegrator()
{
	delete loboNeuralNetwork;
}

int ImplicitNewMarkDNNIntegrator::doTimeStep2()
{
	int numIter = 0;

	double error0 = 0; // error after the first step
	double errorQuotient;
	double preerror = DBL_MAX;
	q_1 = q;
	q_vel_1 = q_vel;
	q_accel_1 = q_accel;

	q_accel = alpha1*(q - q_1) - alpha2*q_vel_1 - alpha3*q_accel_1;
	q_vel = alpha4*(q - q_1) + alpha5*q_vel_1 + alpha6 * q_accel_1;

	do{
		int i;
		forcemodel->getForceAndMatrix(q.data(), internalForces.data(), tangentStiffnessMatrix);

		internalForces *= internalForceScalingFactor;

		*dampingMatrix = dampingMassCoef* (*massMatrix) + dampingStiffnessCoef*(*tangentStiffnessMatrix);

		*systemMatrix = alpha1*(*massMatrix) + alpha4*(*dampingMatrix) + (*tangentStiffnessMatrix);

		VectorXd systemF = (*massMatrix)*(q_accel)
			+(*dampingMatrix)*(q_vel)
			+this->internalForces
			- this->externalForces
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

		//

		q_accel = alpha1*(q - q_1) - alpha2*q_vel_1 - alpha3*q_accel_1;
		q_vel = alpha4*(q - q_1) + alpha5*q_vel_1 + alpha6*q_accel_1;

		//std::cout << std::setprecision(64)<<subSystemMatrix->norm()<<std::endl << subsystemF.norm() << " " << q.norm() << std::endl;

		//if (step_ == 0)
		//{
		//	std::ofstream test("test1.txt");
		//	test.precision(64);
		//	//test << *subSystemMatrix << std::endl;
		//	test << subsystemF << std::endl;
		//	test.close();
		//}

		numIter++;

	} while (numIter < maxIterations);

	//mapQ(q);

	if (getSaveStepResidual())
	{
		step_residual = (*massMatrix)*(q_accel)
			+(*dampingMatrix)*(q_vel)
			-this->externalForces
			;
		step_residual *= -1;
	}

	if (getRecordLoadedStepResults())
	{
		sequence_q.push_back(q);
	}

	return 0;
}

void ImplicitNewMarkDNNIntegrator::mapQ(VectorXd &q_)
{
	int numVertex = q_.size() / 3;

	VectorXd w = (*modalRotationSparseMatrix)*q_;

	for (int i = 0; i < numVertex; i++)
	{
		VectorXd inputV(12);
		Matrix3d R_;
		modalrotationMatrix->computeWarpingRotationMatrixRi(w, i, R_);

		for (int j = 0; j < 9; j++)
		{
			inputV.data()[j + 3] = R_.data()[j];
		}

		inputV.data()[0] = q_.data()[i * 3 + 0];
		inputV.data()[1] = q_.data()[i * 3 + 1];
		inputV.data()[2] = q_.data()[i * 3 + 2];

		VectorXd oridis = loboNeuralNetwork->predictV2(inputV);

		for (int j = 0; j < 3; j++)
		{
			q_.data()[i * 3 + j] += oridis.data()[j];
		}
	}

	for (int i = 0; i < numConstrainedDOFs; i++)
	{
		q_.data()[constrainedDOFs[i]] = 0;
	}
}
