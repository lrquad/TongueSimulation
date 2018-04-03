#include "ReducedForceModel.h"
#include <iostream>
#include <fstream>

ReducedForceModel::ReducedForceModel()
{
}


ReducedForceModel::~ReducedForceModel()
{
}

void ReducedForceModel::getForceAndMatrix(double* u, double* internalForce, MatrixXd* stiffnessmatirx)
{
	getInternalForce(u, internalForce);
	getTangentStiffnessMatrix(u, stiffnessmatirx);
}

void ReducedForceModel::TestStiffnessMatrix(int numTests, double qAmplitude)
{
	VectorXd q(r);
	VectorXd dq(r);
	VectorXd q1(r);
	VectorXd f(r);
	VectorXd df(r);
	VectorXd f1(r);
	MatrixXd K(r, r);

	for (int i = 0; i < numTests; i++)
	{
		for (int j = 0; j<r; j++)
		{
			q[j] = qAmplitude * (-1.0 + 2.0 * rand() / RAND_MAX);
			dq[j] = qAmplitude * (-1.0 + 2.0 * rand() / RAND_MAX);
		}

		std::ofstream test("framedisplacement.txt");
		test << q << std::endl;
		test.close();
		this->getForceAndMatrix(q.data(), f.data(),&K);
		test.open("test.txt");
		test << f << std::endl;
		test << "===========" << std::endl;
		test << K << std::endl;
		test.close();

		double eps = 1.0;
		do
		{
			q1 = q + eps*dq;
			this->getInternalForce(q1.data(), f1.data());
			
			df = K*dq;
			df *= eps;

			double maxEntry = 0.0;
			for (int j = 0; j < r; j++)
			{
				double entry = fabs((f1.data()[j] - f.data()[j] - df.data()[j]) / eps);
				if (entry > maxEntry)
				{
					maxEntry = entry;
				}
			}

			printf("Stiffness matrix trial %d: eps=%G: maxEntryntry=%G\n", i, eps, maxEntry);
			eps *= 0.5;

		} while (eps > 1e-15);

	}

}
