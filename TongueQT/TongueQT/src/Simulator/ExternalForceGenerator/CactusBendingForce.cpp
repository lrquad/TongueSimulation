#include "CactusBendingForce.h"
#include "LoboVolumetricMesh/LoboVolumetricMesh.h"
#include <fstream>
#include <iostream>


CactusBendingForce::CactusBendingForce(SparseMatrix<double>* massMatrix, LoboVolumetricMesh* volumetricmesh) :ExternalForceGenerator(massMatrix)
{
	this->volumetricmesh = volumetricmesh;
	numFrame = 50;
	nodeforce = Vector3d(0, 0, 0);
	leftframestart = 0;
	leftframeend = 40;
	rightframestart = 200;
	rightframeend = 250;

}

CactusBendingForce::~CactusBendingForce()
{
}

void CactusBendingForce::generateForce(int r, std::vector<VectorXd> &forces, MatrixXd* subspacePhi /*= NULL*/)
{
	double maxy = -DBL_MAX;
	int topnodeid = -1;
	int numVertex = volumetricmesh->getNumVertices();
	for (int i = 0; i < numVertex; i++)
	{
		Vector3d position = volumetricmesh->getNodeRestPosition(i);
		if (position.y() > maxy)
		{
			maxy = position.y();
			topnodeid = i;
		}
	}

	int leftnodeid = -1;
	int rightnodeid = -1;
	Vector3d leftndoe = Vector3d(-0.665888,2.46706, -0.0604824);
	Vector3d rightndoe = Vector3d(0.665081,2.84733, -0.0604824);

	leftnodeid = volumetricmesh->searchCloseNode(leftndoe);
	rightnodeid = volumetricmesh->searchCloseNode(rightndoe);

	for (int i = 0; i < numFrame; i++)
	{
		VectorXd force(r);
		force.setZero(); 
		
		if (i >= leftframestart&&i < leftframeend)
		{
			force.data()[leftnodeid * 3 + 0] = nodeforce.data()[0];
			force.data()[leftnodeid * 3 + 1] = nodeforce.data()[1];
			force.data()[leftnodeid * 3 + 2] = nodeforce.data()[2];
		}

		if (i >= rightframestart&&i < rightframeend)
		{
			force.data()[rightnodeid * 3 + 0] = -nodeforce.data()[0];
			force.data()[rightnodeid * 3 + 1] = -nodeforce.data()[1];
			force.data()[rightnodeid * 3 + 2] = -nodeforce.data()[2];
		}

		/*force.data()[topnodeid * 3 + 0] = nodeforce.data()[0];
		force.data()[topnodeid * 3 + 1] = nodeforce.data()[1];
		force.data()[topnodeid * 3 + 2] = nodeforce.data()[2];*/

		forces.push_back(force);
	}

}

void CactusBendingForce::readconfig(const char* filename)
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
			inStream >> numFrame;
		}

		if (token == "nodeforce")
		{
			inStream >> nodeforce.data()[0] >> nodeforce.data()[1] >> nodeforce.data()[2];
		}

		if (token == "leftframestart")
		{
			inStream >> leftframestart;
		}
		
		if (token == "leftframeend")
		{
			inStream >> leftframeend;
		}

		if (token == "rightframestart")
		{
			inStream >> rightframestart;
		}

		if (token == "rightframeend")
		{
			inStream >> rightframeend;
		}

		if (token == "end")
		{
			break;
		}
	}
	inStream.close();
}
