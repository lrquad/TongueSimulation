#include "LoboForceModel.h"
#include "SparseMatrix/SparseMatrixGetMaxAbsEntry.h"
#include <iostream>
#include <fstream>
#include <ctime>

LoboForceModel::LoboForceModel()
{
	uselinearStiffnessMatrix = false;
}

LoboForceModel::~LoboForceModel()
{
	tangentStiffnessMatrix_rest = NULL;
}

void LoboForceModel::computeLinearStiffnessMatrix(double *u, SparseMatrix<double> * tangentStiffnessMatrix)
{
	getTangentStiffnessMatrix(u, tangentStiffnessMatrix);
	this->tangentStiffnessMatrix_rest = tangentStiffnessMatrix;
}

void LoboForceModel::getForceAndMatrix(double * u, double * internalForce, SparseMatrix<double> * tangentStiffnessMatrix)
{
	getInternalForce(u, internalForce);
	getTangentStiffnessMatrix(u, tangentStiffnessMatrix);
}

double LoboForceModel::computeElementEnergy(int eleid)
{
	return 0;
}

void LoboForceModel::TestStiffnessMatrix(double * q, double * dq)
{
	VectorXd q1(r);
	VectorXd dqeps(r);
	VectorXd internalForce0(r);
	VectorXd internalForce1(r);
	VectorXd testV(r);

	SparseMatrix<double> *K0 = new SparseMatrix<double>;
	SparseMatrix<double> *K1 = new SparseMatrix<double>;
	this->getTangentStiffnessMatrixTopology(K0);
	this->getTangentStiffnessMatrixTopology(K1);
	

	getForceAndMatrix(q, internalForce0.data(), K0);
	

	double KNorm0 = SparseMatrixGetMaxAbsEntry(K0);

	double fNorm0 = 0.0;
	for (int j = 0; j<r; j++)
		if (fabs(internalForce0[j]) > fNorm0)
			fNorm0 = fabs(internalForce0[j]);

	double eps = 1.0;
	do
	{
		for (int j = 0; j < r; j++)
		{
			dqeps[j] = eps*dq[j];
			q1[j] = q[j] + dqeps[j];
		}

		getForceAndMatrix(q1.data(),internalForce1.data(),K1);

		testV = (*K0)*dqeps;
		testV = internalForce0 - internalForce1 - testV;
		double maxEntry = 0.0;
		for (int j = 0; j<r; j++)
			if (fabs(testV[j]) > maxEntry)
				maxEntry = fabs(testV[j]);

		double dfNorm = 0.0;
		for (int j = 0; j<r; j++)
			if (fabs(internalForce1[j] - internalForce0[j]) > dfNorm)
				dfNorm = fabs(internalForce1[j] - internalForce0[j]);

		std::cout << eps << " " << maxEntry << " maxEntry/eps^2:" << maxEntry / eps / eps << " dfNorm:" << dfNorm << " f0Norm:" << fNorm0 << " K0Norm:" << KNorm0 << std::endl;

		eps *= 0.1;
	} while (eps > 1e-15);


	delete K0;
	delete K1;

}