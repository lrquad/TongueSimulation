#include "BendingForceGenerator.h"
#include "LoboVolumetricMesh/LoboVolumetricMesh.h"
#define M_PI 3.1415

BendingForceGenerator::BendingForceGenerator(SparseMatrix<double>* massMatrix, LoboVolumetricMesh* volumetricmesh) :ExternalForceGenerator(massMatrix)
{
	this->volumetricmesh = volumetricmesh;
}

BendingForceGenerator::~BendingForceGenerator()
{

}

void BendingForceGenerator::generateForce(int numFrames, int r, std::vector<VectorXd> &forces)
{
	forces.clear();
	Matrix3d m;
	m = AngleAxisd(-0.009, Vector3d::UnitX());
	Matrix3d rotation = m;


	for (int i = 0; i < numFrames; i++)
	{
		VectorXd force(r);
		force.setZero();
		rotation = m*rotation;
		for (int j = 0; j < r / 3; j++)
		{
			Vector3d position = volumetricmesh->getNodeRestPosition(j);
			Vector3d nodeforce;
			nodeforce.setZero();
			if (position.z() > 4.0)
			{
				nodeforce.y() = 15.0+i*0.1;
			}

			nodeforce = rotation*nodeforce;
			
			force.data()[j * 3 + 0] = nodeforce.data()[0];
			force.data()[j * 3 + 1] = nodeforce.data()[1];
			force.data()[j * 3 + 2] = nodeforce.data()[2]; 
		}

		if (massMatrix != NULL)
			force = (*massMatrix)*force;
		forces.push_back(force);
	}
}
