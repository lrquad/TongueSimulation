#pragma once
#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <Eigen/PardisoSupport>
using namespace Eigen;

class SparseJacobiSolver
{
public:
	enum SolutionState
	{
		Indeterminate,
		IsConverging,
		Converged,
		IsDiverging,
		Diverged
	};

	SparseJacobiSolver(SparseMatrix<double> &systemMatrix,int maxiteration = 10,double threshold = 1e-10,bool initmatrix = true);

	~SparseJacobiSolver();


	/**
	 * @brief check iteration matrix's 
	 * @return Put return information here
	 */
	virtual void checkConvergeCondition();
	
	virtual VectorXd solve(VectorXd &b);

	virtual void doIteration(VectorXd & b);

protected:


	/**
	 * @brief decompose matrix to A = D + R.
	 * D is diagnoal matrix.
	 * @return Put return information here
	 */
	virtual void decomposeMatrix();

	virtual void updateResidual(VectorXd &b);
	virtual void isDiagonallyDomaint(SparseMatrix<double> &A);

	SolutionState solutionState;

	SparseMatrix<double> systemMatrix;
	SparseMatrix<double> D;
	SparseMatrix<double> inverseD;
	SparseMatrix<double> R;

	VectorXd currentX;
	VectorXd nextX;
	VectorXd residual;

	double norm2;
	double lastnorm2;

	int iteration;
	int maxiteration;
	double tolerance;
};

