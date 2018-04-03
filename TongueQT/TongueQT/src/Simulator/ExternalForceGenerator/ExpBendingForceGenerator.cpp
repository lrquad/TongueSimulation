#include "ExpBendingForceGenerator.h"
#include "LoboVolumetricMesh/LoboVolumetricMesh.h"
#include <iostream>
#include <fstream>

#define M_PI 3.1415

ExpBendingForceGenerator::ExpBendingForceGenerator(SparseMatrix<double>* massMatrix, LoboVolumetricMesh* volumetricmesh) :BendingForceGenerator(massMatrix,volumetricmesh)
{
	
}

ExpBendingForceGenerator::~ExpBendingForceGenerator()
{
}

void ExpBendingForceGenerator::generateForce(int numFrames, int r, std::vector<VectorXd> &forces)
{
	forces.clear();
	Matrix3d m;
	double speed = 0.009;
	m = AngleAxisd(speed, Vector3d::UnitY());
	Matrix3d rotation = m;

	for (int i = 0; i < numFrames; i++)
	{
		VectorXd force(r);
		force.setZero();
	
		rotation = m*rotation;
		//speed -= 0.000001;

		for (int j = 0; j < r / 3; j++)
		{
			Vector3d position = volumetricmesh->getNodeRestPosition(j);
			Vector3d nodeforce;
			nodeforce.setZero();
			if (position.z() > 4.0)
			{
				nodeforce.x() = 15.0 + i*0.15;//0.15
				nodeforce = rotation*nodeforce;
				//nodeforce.data()[0] *= -1;
				nodeforce.y() = 1.0;
			}

			force.data()[j * 3 + 0] = nodeforce.data()[0];
			force.data()[j * 3 + 1] = nodeforce.data()[1];
			force.data()[j * 3 + 2] = nodeforce.data()[2];
		}

		if (massMatrix != NULL)
			force = (*massMatrix)*force;

		forces.push_back(force);
	}
}
