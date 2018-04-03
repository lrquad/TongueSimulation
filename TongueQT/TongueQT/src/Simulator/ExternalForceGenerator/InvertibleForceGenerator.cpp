#include "InvertibleForceGenerator.h"
#include "LoboVolumetricMesh/LoboVolumetricMesh.h"
#include <fstream>
#include <iostream>

InvertibleForceGenerator::InvertibleForceGenerator(SparseMatrix<double>* massMatrix, LoboVolumetricMesh* volumetricmesh, VectorXd fullgravity) :FunnelForceGenerator(massMatrix, volumetricmesh)
{
	numFrames = 0;
	setRestoreforceframe(100);
	this->setGravity(fullgravity);
	scale = 0;
	step = 0;
}

InvertibleForceGenerator::~InvertibleForceGenerator()
{

}

void InvertibleForceGenerator::readconfig(const char* filename)
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

		if (token == "keyframe")
		{
			inStream >> restoreforceframe;
		}

		if (token == "restorestep")
		{
			inStream >> internalforce_restor_step;
		}

		if (token == "end")
		{
			break;
		}
	}
}

void InvertibleForceGenerator::generateForce(int r, std::vector<VectorXd> &forces, MatrixXd* subspacePhi /*= NULL*/)
{
	

}

double InvertibleForceGenerator::getInternalForceScale()
{
	scale += internalforce_restor_step;
	if (scale > 1)
	{
		scale = 1;
	}
	std::cout << scale << std::endl;

	step++;
	return scale;
}
