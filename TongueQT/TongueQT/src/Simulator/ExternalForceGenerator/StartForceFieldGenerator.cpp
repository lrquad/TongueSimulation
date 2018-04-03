#include "StartForceFieldGenerator.h"

#include "LoboVolumetricMesh/LoboVolumetricMesh.h"
#include <fstream>
#include <iostream>

StartForceFieldGenerator::StartForceFieldGenerator(SparseMatrix<double>* massMatrix, LoboVolumetricMesh* volumetricmesh, VectorXd fullgravity) :FunnelForceGenerator(massMatrix,volumetricmesh)
{
	numFrames = 0;
	forceField.setZero();
}

StartForceFieldGenerator::~StartForceFieldGenerator()
{
}

void StartForceFieldGenerator::readconfig(const char* filename)
{
	std::cout << filename << std::endl;
	std::ifstream inStream(filename);
	std::string token;
	while (true)
	{
		inStream >> token;

		if (token[0] == '#')
		{
			inStream.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
		}

		if (token == "numFrames")
		{
			inStream >> numFrames;
		}

		if (token == "forceField")
		{
			inStream >> forceField.data()[0] >> forceField.data()[1] >> forceField.data()[2];
		}

		if (token == "end")
		{
			break;
		}
	}
}

void StartForceFieldGenerator::generateForce(int r, std::vector<VectorXd> &forces, MatrixXd* subspacePhi /*= NULL*/)
{
	int numVertex = volumetricmesh->getNumVertices();

	for (int i = 0; i < numFrames; i++)
	{
		VectorXd force(r);
		force.setZero();

		for (int j = 0; j < numVertex; j++)
		{
			force.data()[j * 3 + 0] = forceField.data()[0];
			force.data()[j * 3 + 1] = forceField.data()[1];
			force.data()[j * 3 + 2] = forceField.data()[2];
		}

		force = *this->massMatrix*force;
		forces.push_back(force);
	}
}

