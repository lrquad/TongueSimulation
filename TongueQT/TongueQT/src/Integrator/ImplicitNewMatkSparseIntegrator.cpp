#include "ImplicitNewMatkSparseIntegrator.h"
#include "SparseMatrix/SparseMatrixRemoveRows.h"
#include <fstream>
#include <iostream>

#include <iomanip> 

ImplicitNewMatkSparseIntegrator::ImplicitNewMatkSparseIntegrator(int r, double timestep, SparseMatrix<double>* massMatrix_, LoboForceModel* forcemodel_, int positiveDefiniteSolver_, int numConstrainedDOFs_, int* constrainedDOFs_, double dampingMassCoef, double dampingStiffnessCoef, int maxIterations, double epsilon, double NewmarkBeta, double NewmarkGamma, bool usestatic) : SparseIntegratorBase(r, timestep, massMatrix_, forcemodel_, numConstrainedDOFs_, constrainedDOFs_, dampingMassCoef, dampingStiffnessCoef), positiveDefiniteSolver(positiveDefiniteSolver_)
{

	use_lagrange_multi = false;
	use_lagrange_multi2 = false;
	setStoreLagrangeMultipliers(false);
	this->useStaticSolver(usestatic);
	this->maxIterations = maxIterations;
	this->epsilon = epsilon;

	this->NewmarkBeta = NewmarkBeta;
	this->NewmarkGamma = NewmarkGamma;

	tangentStiffnessMatrix = new SparseMatrix<double>();
	systemMatrix = new SparseMatrix<double>(r,r);
	lagrangeConstrainMatrix = new SparseMatrix<double>();

	forcemodel->getTangentStiffnessMatrixTopology(tangentStiffnessMatrix);
	
	//create final topology
	*systemMatrix = *tangentStiffnessMatrix + *massMatrix;

	subSystemMatrix = new SparseMatrix<double>();


	createMapByConstrains(mapOldNew, r, numConstrainedDOFs, constrainedDOFs);
	createSparseMapbyTopology(systemMatrix, subSystemMatrix, mapSparseMatrixEntryOldNew, mapOldNew, r, numConstrainedDOFs, constrainedDOFs);

	updateAlphas();

	if (use_static_solver)
	{
		forcemodel->getTangentStiffnessMatrix(q.data(),tangentStiffnessMatrix);
		createSparseMapbyTopology(tangentStiffnessMatrix, subSystemMatrix, mapSparseMatrixEntryOldNew, mapOldNew, r, numConstrainedDOFs, constrainedDOFs);
		SparseMatrixRemoveRows(tangentStiffnessMatrix, subSystemMatrix, mapSparseMatrixEntryOldNew, r, numConstrainedDOFs, constrainedDOFs);
	}

	//vegatest_ = new Vegatest("./data/ElementMesh/staypuftveg.veg", 60, 0.3, 1.0);
	vegatest_ = NULL;

	if (forcemodel->getUselinearStiffnessMatrix())
	{
		forcemodel->computeLinearStiffnessMatrix(q.data(), tangentStiffnessMatrix);
	}

	/*vegatest_->test(q.data());

	std::ofstream test2("test2.txt");
	test2 << *tangentStiffnessMatrix << std::endl;
	test2.close();*/
}


ImplicitNewMatkSparseIntegrator::~ImplicitNewMatkSparseIntegrator()
{
	delete rayleighDampingMatrix;
	delete tangentStiffnessMatrix;
	delete systemMatrix;
	delete subSystemMatrix;
	delete vegatest_;
	delete lagrangeConstrainMatrix;
}

void ImplicitNewMatkSparseIntegrator::updateMaterial()
{
	forcemodel->updateMaterial();
	if (forcemodel->getUselinearStiffnessMatrix())
	{
		forcemodel->computeLinearStiffnessMatrix(q.data(), tangentStiffnessMatrix);
	}
}

void ImplicitNewMatkSparseIntegrator::setConstrainByLagrangeMulti(SparseMatrix<double>* constrainMatrix)
{
	this->numConstrainedDOFs = constrainMatrix->rows();
	this->constrainTarget.resize(numConstrainedDOFs);
	this->constrainTarget.setZero();
	int rows = r+constrainMatrix-> rows();
	this->use_lagrange_multi = true;
	constrainedSystemMatrix = new SparseMatrix<double>(rows,rows);
	std::vector<EIGEN_TRI> entrys;
	for (int i = 0; i < tangentStiffnessMatrix->outerSize(); ++i)
		for (SparseMatrix<double>::InnerIterator it(*tangentStiffnessMatrix, i); it; ++it)
		{
			entrys.push_back(EIGEN_TRI(it.row(), it.col(), it.value()));
		}

	for (int i = 0; i < constrainMatrix->outerSize(); ++i)
		for (SparseMatrix<double>::InnerIterator it(*constrainMatrix, i); it; ++it)
		{
			int row = it.row() + r;
			int col = it.col();
			entrys.push_back(EIGEN_TRI(row, col, it.value()));
			entrys.push_back(EIGEN_TRI(col, row, it.value()));
		}
	constrainedSystemMatrix->setFromTriplets(entrys.begin(), entrys.end());
}


void ImplicitNewMatkSparseIntegrator::setConstrainByLagrangeMulti(SparseMatrix<double>* constrainMatrix, VectorXd constrainTarget_)
{
	this->numConstrainedDOFs = constrainMatrix->rows();
	this->constrainTarget = constrainTarget_;
	int rows = r + constrainMatrix->rows();
	this->use_lagrange_multi = true;
	constrainedSystemMatrix = new SparseMatrix<double>(rows, rows);
	std::vector<EIGEN_TRI> entrys;
	for (int i = 0; i < tangentStiffnessMatrix->outerSize(); ++i)
		for (SparseMatrix<double>::InnerIterator it(*tangentStiffnessMatrix, i); it; ++it)
		{
			entrys.push_back(EIGEN_TRI(it.row(), it.col(), it.value()));
		}

	for (int i = 0; i < constrainMatrix->outerSize(); ++i)
		for (SparseMatrix<double>::InnerIterator it(*constrainMatrix, i); it; ++it)
		{
			int row = it.row() + r;
			int col = it.col();
			entrys.push_back(EIGEN_TRI(row, col, it.value()));
			entrys.push_back(EIGEN_TRI(col, row, it.value()));
		}
	constrainedSystemMatrix->setFromTriplets(entrys.begin(), entrys.end());
}

void ImplicitNewMatkSparseIntegrator::setMaxInteration(int maxiteration)
{
	this->maxIterations = maxiteration;
}


void ImplicitNewMatkSparseIntegrator::updateAlphas()
{
	alpha1 = 1.0 / (NewmarkBeta * timestep * timestep);
	alpha2 = 1.0 / (NewmarkBeta * timestep);
	alpha3 = (1.0 - 2.0 * NewmarkBeta) / (2.0 * NewmarkBeta);
	alpha4 = NewmarkGamma / (NewmarkBeta * timestep);
	alpha5 = 1 - NewmarkGamma / NewmarkBeta;
	alpha6 = (1.0 - NewmarkGamma / (2.0 * NewmarkBeta)) * timestep;
}

void ImplicitNewMatkSparseIntegrator::setDampingMatrix(SparseMatrix<double>* dampingMatrix)
{
	SparseIntegratorBase::setDampingMatrix(dampingMatrix);
}

int ImplicitNewMatkSparseIntegrator::setState(double *q_, double *qvel_)
{
	memcpy(q_1.data(), q_, sizeof(double)*r);
	if (qvel_ != NULL)
	{
		memcpy(q_vel_1.data(), qvel_, sizeof(double)*r);
	}
	else
	{
		q_vel_1.setZero();
	}

	for (int i = 0; i < numConstrainedDOFs; i++)
	{
		q_1.data()[constrainedDOFs[i]] = q_vel_1.data()[constrainedDOFs[i]] = 0;
	}


	//q_vel_1 = alpha4*(q_1 - q) + alpha5*q_vel + alpha6*q_accel;
	//q_accel_1 = alpha1*(q_1 - q) - alpha2*q_vel - alpha3*q_accel;

	q_vel = q_vel_1;
	//q_accel = q_accel_1;
	q = q_1;

	return 0;

	forcemodel->getForceAndMatrix(q.data(), internalForces.data(), tangentStiffnessMatrix);
	internalForces *= internalForceScalingFactor;
	*dampingMatrix = dampingMassCoef* (*massMatrix) + dampingStiffnessCoef*(*tangentStiffnessMatrix);
	VectorXd buffer = (*dampingMatrix)*q_vel;
	buffer = -buffer - internalForces;

	VectorXd subsystemF(r - numConstrainedDOFs);

	//set zero
	for (int i = 0; i < tangentStiffnessMatrix->outerSize(); ++i)
		for (SparseMatrix<double>::InnerIterator it(*tangentStiffnessMatrix, i); it; ++it)
		{
			it.valueRef() = 0;
		}

	*systemMatrix = 1.0*(*massMatrix) + 1.0*(*dampingMatrix) + (*tangentStiffnessMatrix);


	SparseMatrixRemoveRows(systemMatrix, subSystemMatrix, mapSparseMatrixEntryOldNew, r, numConstrainedDOFs, constrainedDOFs);
	VectorRemoveRows(mapOldNew, buffer, subsystemF, numConstrainedDOFs, constrainedDOFs);

#ifdef EIGEN_USE_MKL_ALL
	PardisoLDLT<SparseMatrix<double>> LDLT(*subSystemMatrix);
#else
	SimplicialLDLT<SparseMatrix<double>> LDLT(*subSystemMatrix);
#endif
	VectorXd result_ = LDLT.solve(subsystemF);

	this->qdelta.setZero();
	VectorInsertRows(mapOldNew, result_, qdelta, numConstrainedDOFs, constrainedDOFs);

	q_accel = qdelta;
	return 0;
}

void ImplicitNewMatkSparseIntegrator::changeCurrentState(VectorXd &q_1_,VectorXd &q_, VectorXd &q_vel_, VectorXd &q_accel_)
{
	q_1 = q_1_;
	q = q_;

	q_vel_1 = alpha4*(q_1 - q) + alpha5*q_vel_ + alpha6*q_accel_;
	q_accel_1 = alpha1*(q_1 - q) - alpha2*q_vel_ - alpha3*q_accel_;

	q_vel = q_vel_1;
	q_accel = q_accel_1;
	q = q_1;
}

void ImplicitNewMatkSparseIntegrator::changeCurrentState(VectorXd &q_, VectorXd &q_vel_, VectorXd &q_accel_)
{
	q = q_;
	q_vel = q_vel_;
	q_accel = q_accel_;
}

void ImplicitNewMatkSparseIntegrator::staticTimeStep()
{
	VectorXd systemF = this->externalForces;
	VectorXd subsystemF(r - numConstrainedDOFs);

	q.setZero();
	forcemodel->getForceAndMatrix(q.data(), internalForces.data(), tangentStiffnessMatrix);
	*systemMatrix = (*tangentStiffnessMatrix);

	SparseMatrixRemoveRows(systemMatrix, subSystemMatrix, mapSparseMatrixEntryOldNew, r, numConstrainedDOFs, constrainedDOFs);

	//forcemodel->getInternalForce(q.data(), internalForces.data());
	VectorRemoveRows(mapOldNew, systemF, subsystemF, numConstrainedDOFs, constrainedDOFs);

#ifdef EIGEN_USE_MKL_ALL
	PardisoLDLT<SparseMatrix<double>> LDLT(*subSystemMatrix);
#else
	SimplicialLDLT<SparseMatrix<double>> LDLT(*subSystemMatrix);
#endif
	VectorXd result_ = LDLT.solve(subsystemF);

	this->qdelta.setZero();
	VectorInsertRows(mapOldNew, result_, qdelta, numConstrainedDOFs, constrainedDOFs);
	q = qdelta;

}

void ImplicitNewMatkSparseIntegrator::staticTimeStepLagrange()
{
	VectorXd subsystemF(r + numConstrainedDOFs);
	subsystemF.setZero();
	subsystemF.block(0, 0, r, 1) = this->externalForces;
	for (int i = r; i < r + numConstrainedDOFs; i++)
	{
		subsystemF.data()[i] = constrainTarget.data()[i-r];
	}

#ifdef EIGEN_USE_MKL_ALL
	PardisoLDLT<SparseMatrix<double>> LDLT(*constrainedSystemMatrix);
#else
	SparseLU<SparseMatrix<double>> LDLT(*constrainedSystemMatrix);
#endif

	VectorXd result_ = LDLT.solve(subsystemF);
	qdelta = result_.block(0, 0, r, 1);
	q = qdelta;


	if (getStoreLagrangeMultipliers())
	{
		storedLagrangeMultipliers = result_.block(r, 0, numConstrainedDOFs, 1);
	}
}

void ImplicitNewMatkSparseIntegrator::staticTimeStepLagrangeV2()
{
	VectorXd systemF = this->externalForces;
	VectorXd subsystemF(r - numConstrainedDOFs);

	q.setZero();
	forcemodel->getForceAndMatrix(q.data(), internalForces.data(), tangentStiffnessMatrix);
	*systemMatrix = (*tangentStiffnessMatrix);

	SparseMatrixRemoveRows(systemMatrix, subSystemMatrix, mapSparseMatrixEntryOldNew, r, numConstrainedDOFs, constrainedDOFs);

	//forcemodel->getInternalForce(q.data(), internalForces.data());
	VectorRemoveRows(mapOldNew, systemF, subsystemF, numConstrainedDOFs, constrainedDOFs);

#ifdef EIGEN_USE_MKL_ALL
	PardisoLDLT<SparseMatrix<double>> LDLT(*subSystemMatrix);
#else
	SimplicialLDLT<SparseMatrix<double>> LDLT(*subSystemMatrix);
#endif
	VectorXd result_ = LDLT.solve(subsystemF);

	this->qdelta.setZero();
	VectorInsertRows(mapOldNew, result_, qdelta, numConstrainedDOFs, constrainedDOFs);
	q = qdelta;
}

int ImplicitNewMatkSparseIntegrator::doTimeStep2()
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
			+ (*dampingMatrix)*(q_vel)
			+ this->internalForces
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

		if (std::abs(error - preerror)/std::abs(error) < 1e-4)
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

	if (getSaveStepResidual())
	{
		forcemodel->getForceAndMatrix(q.data(), internalForces.data(), tangentStiffnessMatrix);
		step_residual = internalForces;
	}

	if (getRecordLoadedStepResults())
	{
		sequence_q.push_back(q);
	}

	return 0;
}

double ImplicitNewMatkSparseIntegrator::wolfeconditionFunction(VectorXd &q_, VectorXd &eForce)
{
	forcemodel->getForceAndMatrix(q_.data(), internalForces.data(), tangentStiffnessMatrix);
	VectorXd temp = internalForces - eForce;

	VectorXd subsystemF(r - numConstrainedDOFs);
	
	VectorRemoveRows(mapOldNew, temp, subsystemF, numConstrainedDOFs, constrainedDOFs);

	return subsystemF.dot(subsystemF);
}

double ImplicitNewMatkSparseIntegrator::wolfeconditionGradient(VectorXd &x, VectorXd &p, VectorXd &eForce)
{
	forcemodel->getForceAndMatrix(x.data(), internalForces.data(), tangentStiffnessMatrix);

	SparseMatrixRemoveRows(tangentStiffnessMatrix, subSystemMatrix, mapSparseMatrixEntryOldNew, r, numConstrainedDOFs, constrainedDOFs);

	VectorXd temp = internalForces - eForce;
	VectorXd subsystemF(r - numConstrainedDOFs);

	VectorRemoveRows(mapOldNew, temp, subsystemF, numConstrainedDOFs, constrainedDOFs);

	VectorXd gradient = *subSystemMatrix*(subsystemF)* 2;
	
	VectorXd subp(r - numConstrainedDOFs);
	VectorRemoveRows(mapOldNew, p, subp, numConstrainedDOFs, constrainedDOFs);


	return subp.dot(gradient);
}

int ImplicitNewMatkSparseIntegrator::doTimeStep()
{
	if (getUseLoadedStepResults())
	{
		if (step_ * skipSteps < sequence_q.size())
			q = sequence_q[step_ * skipSteps];
		step_++;
		return 1;
	}


	if (use_static_solver)
	{
		if (use_lagrange_multi2)
		{
			staticTimeStepLagrangeV2();
		}else
		if (use_lagrange_multi)
		{
			staticTimeStepLagrange();
		}
		else
			staticTimeStep();

		step_++;

		return 0;
	}

	doTimeStep2();

	step_++;

	return 1;

	this->q_1 = q;
	double error;
	double error0;
	double preerror = DBL_MAX;
	double errorQuotient;

	std::cout << "---" << std::endl;
	for (int i = 0; i < maxIterations; i++)
	{
		forcemodel->getForceAndMatrix(q.data(), internalForces.data(), tangentStiffnessMatrix);
		internalForces *= internalForceScalingFactor;

		*dampingMatrix = dampingMassCoef* (*massMatrix) + dampingStiffnessCoef*(*tangentStiffnessMatrix);

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

		std::cout << error << std::endl;
		
		if (std::abs(error - preerror) < 1e-4)
		{
			break;
		}

		preerror = error;

		if (errorQuotient < epsilon*epsilon)
		{
			break;
		}

		VectorXd subsystemF(r-numConstrainedDOFs);

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
		
		q_1 += qdelta;
	}

	q_vel_1 = alpha4*(q_1 - q) + alpha5*q_vel + alpha6*q_accel;
	q_accel_1 = alpha1*(q_1 - q) - alpha2*q_vel - alpha3*q_accel;

	q_vel = q_vel_1;
	q_accel = q_accel_1;
	q = q_1;

	if (getRecordLoadedStepResults())
	{
		sequence_q.push_back(q);
	}

	step_++;

	return 1;
}


Eigen::VectorXd ImplicitNewMatkSparseIntegrator::getInteranlForce(VectorXd q_)
{
	forcemodel->getForceAndMatrix(q_.data(), internalForces.data(), tangentStiffnessMatrix);
	return internalForces;
}

bool ImplicitNewMatkSparseIntegrator::computeStaticDisplacement(VectorXd &externalForce, bool checkwolfecondition)
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
		forcemodel->getForceAndMatrix(q.data(), internalForces.data(), tangentStiffnessMatrix);
		//internalForces *= internalForceScalingFactor;
		*systemMatrix =(*tangentStiffnessMatrix);

		VectorXd systemF = 
			this->internalForces
			- externalForce
			;

		systemF *= -1;
		error = 0;
		error = systemF.norm()*systemF.norm();

		std::cout << error << std::endl;

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

		if (errorQuotient > 4 || error != error)
		{
			return false;
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

				if (numtry > 5)
				{
					std::cout << "wolfe failed" << std::endl;
					q_vel.setZero();
					q_accel.setZero();
					//return false;
					break;
				}

			} while (!(wolfe1 && wolfe2));
		}

		q += alpha*direction;

		numIter++;
		//std::cout << "\r";
		//std::cout << numIter;
	} while (numIter < maxIterations);
	//std::cout << std::endl;
	q_vel.setZero();
	q_accel.setZero();

	//std::cout << "error0: " << error0 << " eror: " << error << " errorQuotient: " << errorQuotient << std::endl;
	//std::cout << "numIter -> " << numIter << std::endl;

	/*if (numIter >= maxIterations)
	{
		return false;
	}*/

	return true;
}

void ImplicitNewMatkSparseIntegrator::testDisplacementOutputInternalforce(VectorXd &q_input, const char* filename)
{
	forcemodel->getForceAndMatrix(q_input.data(), internalForces.data(), tangentStiffnessMatrix);
	std::ofstream test(filename);
	test << internalForces << std::endl;
	test.close();
}

void ImplicitNewMatkSparseIntegrator::useStaticSolver(bool b)
{
	this->use_static_solver = b;
}

