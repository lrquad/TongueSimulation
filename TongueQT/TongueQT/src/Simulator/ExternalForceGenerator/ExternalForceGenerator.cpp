#include "ExternalForceGenerator.h"

ExternalForceGenerator::ExternalForceGenerator(SparseMatrix<double>* massMatrix)
{
	this->massMatrix = massMatrix;
}

ExternalForceGenerator::~ExternalForceGenerator()
{

}

void ExternalForceGenerator::generateForce(int numFrames, int r, std::vector<VectorXd> &forces)
{
	forces.clear();
	for (int i = 0; i < numFrames; i++)
	{
		VectorXd force(r);
		force.setZero();

		for (int j = 0; j < r / 3; j++)
		{
			force.data()[j * 3 + 1] = 10.0;
		}
		if (massMatrix!=NULL)
		force = (*massMatrix)*force;

		forces.push_back(force);
	}
}
