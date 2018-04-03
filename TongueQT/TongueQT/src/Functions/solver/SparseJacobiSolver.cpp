#include "SparseJacobiSolver.h"
#include <iostream>
#include <fstream>

SparseJacobiSolver::SparseJacobiSolver(SparseMatrix<double> &systemMatrix, int maxiteration, double threshold, bool initmatrix)
{
	this->systemMatrix = systemMatrix;
	this->maxiteration = maxiteration;
	this->tolerance = threshold;

	D.resize(systemMatrix.rows(), systemMatrix.cols());
	R.resize(systemMatrix.rows(), systemMatrix.cols());
	inverseD.resize(systemMatrix.rows(), systemMatrix.cols());

	currentX.resize(systemMatrix.cols());
	nextX.resize(systemMatrix.cols());
	residual.resize(systemMatrix.cols());

	if (initmatrix)
	decomposeMatrix();

	solutionState = Indeterminate;

	std::cout << "init jacobi solver" << std::endl;
}


void SparseJacobiSolver::checkConvergeCondition()
{
	//check condition
	MatrixXd temp = (inverseD*R);
	VectorXd eivals = temp.selfadjointView<Lower>().eigenvalues();;
	std::cout << "iteration condition: " << eivals.maxCoeff() << std::endl;
}

SparseJacobiSolver::~SparseJacobiSolver()
{
}

Eigen::VectorXd SparseJacobiSolver::solve(VectorXd &b)
{
	currentX.setZero();
	nextX.setZero();

	updateResidual(b);

	iteration = 0;
	norm2 = residual.norm();
	lastnorm2 = norm2;

	while (solutionState != Converged &&
		solutionState != Diverged)
	{
		doIteration(b);

		currentX = nextX;
		updateResidual(b);
		
		norm2 = residual.norm();

		//std::cout << norm2 << std::endl;

		if (norm2 < tolerance)
		{
			solutionState = Converged;
		}
		else
		{
			if (norm2 < lastnorm2)
			{
				solutionState = IsConverging;
			}
			else if (norm2 == lastnorm2)
			{
				solutionState = Indeterminate;
			}
			else
			{
				solutionState = IsDiverging;
			}

			if (iteration >= maxiteration)
			{
				solutionState = Converged;
			}
		}

		if (std::abs(lastnorm2 - norm2) < 1e-5)
		{
			solutionState = Converged; 
		}
		//std::cout << std::abs(lastnorm2 - norm2) << std::endl;

		lastnorm2 = norm2;
		

		iteration++;
	}

	std::cout << iteration << " iteration. "<< norm2 <<" residual." << std::endl;

	return currentX;
}

void SparseJacobiSolver::doIteration(VectorXd & b)
{
	nextX = inverseD*residual + currentX;
}

void SparseJacobiSolver::decomposeMatrix()
{
	std::vector<Eigen::Triplet<double>> coeff_diag;
	std::vector<Eigen::Triplet<double>> coeff_inverdiag;
	std::vector<Eigen::Triplet<double>> coeff_nondiag;

	for (int i = 0; i < systemMatrix.outerSize(); ++i)
	{
		for (SparseMatrix<double>::InnerIterator it(systemMatrix, i); it; ++it)
		{
			int row = it.row();
			int col = it.col();
			if (row == col)
			{
				coeff_diag.push_back(Eigen::Triplet<double>(row, col, it.value()));
				coeff_inverdiag.push_back(Eigen::Triplet<double>(row, col, 1/it.value()));
			}
			else
			{
				coeff_nondiag.push_back(Eigen::Triplet<double>(row,col,it.value()));
			}
		}
	}

	D.setFromTriplets(coeff_diag.begin(), coeff_diag.end());
	R.setFromTriplets(coeff_nondiag.begin(), coeff_nondiag.end());
	inverseD.setFromTriplets(coeff_inverdiag.begin(), coeff_inverdiag.end());
}

void SparseJacobiSolver::updateResidual(VectorXd &b)
{
	residual = b - systemMatrix*currentX;
}

void SparseJacobiSolver::isDiagonallyDomaint(SparseMatrix<double> &A)
{
	for (int i = 0; i < A.outerSize(); ++i)
	{
		for (SparseMatrix<double>::InnerIterator it(A, i); it; ++it)
		{


		}
	}
}

