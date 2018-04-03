#include "FunnelForceGenerator.h"
#include "LoboVolumetricMesh/LoboVolumetricMesh.h"
#include <fstream>
#include <iostream>

FunnelForceGenerator::FunnelForceGenerator(SparseMatrix<double>* massMatrix, LoboVolumetricMesh* volumetricmesh) :ExternalForceGenerator(massMatrix)
{
	this->volumetricmesh = volumetricmesh;
	nodeforce_ = 25;
	preFrames = 250;
	decrease_ratio = 0.965;
	numFrames = 500;
	inscrease_ratio = 1.01;
	second_inscrease_ratio = 1.02;
	decreaseFrames = 50;
	secondPreFrames = 105;
	funnelkeyframe = 35;
	scale_ = 3.5;
}

FunnelForceGenerator::~FunnelForceGenerator()
{
}

void FunnelForceGenerator::readconfig(const char* filename)
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

		if (token == "nodeforce_")
		{
			inStream >> nodeforce_;
		}

		if (token == "preFrames")
		{
			inStream >> preFrames;
		}

		if (token == "decrease_ratio")
		{
			inStream >> decrease_ratio;
		}

		if (token == "inscrease_ratio")
		{
			inStream >> inscrease_ratio;
		}

		if (token == "second_inscrease_ratio")
		{
			inStream >> second_inscrease_ratio;
		}
		
		if (token == "decreaseFrames")
		{
			inStream >> decreaseFrames;
		}

		if (token == "secondPreFrames")
		{
			inStream >> secondPreFrames;
		}

		if (token == "funnelkeyframe")
		{
			inStream >> funnelkeyframe;
		}

		if (token == "scale_")
		{
			inStream >> scale_;
		}

		if (token == "end")
		{
			break;
		}
	}
}

void FunnelForceGenerator::generateForce(int r, std::vector<VectorXd> &forces, MatrixXd* subspacePhi /*= NULL*/)
{
	forces.clear();

	//select nodes

	std::vector<int> surfacenodes = volumetricmesh->getSurface_node();
	std::vector<int> selectedNodes;
	for (int i = 0; i < surfacenodes.size(); i++)
	{
		int nodeid = surfacenodes[i];
		Vector3d position  = volumetricmesh->getNodeRestPosition(nodeid);
		if (position.x() > -1.8)
		{
			double radius = std::sqrt(position.y()*position.y() + position.z()*position.z());
			if (radius<0.1)
			{
				selectedNodes.push_back(nodeid);
			}
		}
	}

	Vector3d nodeforce;
	nodeforce.setZero();
	nodeforce.x() = nodeforce_;
	int numVertex = volumetricmesh->getNumVertices();
	for (int i = 0; i < numFrames; i++)
	{
		if (i < preFrames)
		{
			nodeforce *= inscrease_ratio;
		}
		else if (i<decreaseFrames + preFrames)
		{
			nodeforce *= decrease_ratio;
		}
		else if (i<decreaseFrames + preFrames + secondPreFrames)
		{
			nodeforce *= second_inscrease_ratio;
		}
		else
		{
			nodeforce *= decrease_ratio;
		}

		VectorXd force(r);
		force.setZero();

		for (int j = 0; j < selectedNodes.size(); j++)
		{
			int nodeid = selectedNodes[j];

			force.data()[nodeid * 3 + 0] = nodeforce.data()[0];
			force.data()[nodeid * 3 + 1] = nodeforce.data()[1];
			force.data()[nodeid * 3 + 2] = nodeforce.data()[2];
		}

		/*for (int j = 0; j < numVertex; j++)
		{
			Vector3d position = volumetricmesh->getNodePosition(j);
			double radius = std::sqrt(position.x()*position.x() + position.z()*position.z());
			if (radius > 0.4)
			{
				position.data()[1] = 0;
				force.data()[j * 3 + 0] += -0.005*position.data()[0];
				force.data()[j * 3 + 2] += -0.005*position.data()[2];
			}
		}*/

		if (subspacePhi != NULL)
		{
			force = subspacePhi->transpose()*force;
		}

		forces.push_back(force);
	}

	////gravity
	//VectorXd gravityforce(r);
	//gravityforce.setZero();

	//for (int i = 0; i < 35; i++)
	//{
	//	if (subspacePhi != NULL)
	//	{
	//		VectorXd force(r);
	//		force = subspacePhi->transpose()*gravityforce;
	//	}
	//	forces.push_back(gravityforce);
	//}

	//for (int i = 0; i < numVertex; i++)
	//{
	//	gravityforce.data()[i * 3 + 1] = -50;
	//}

	//gravityforce = *massMatrix*gravityforce;

	//for (int i = 0; i < numFrames; i++)
	//{
	//	if (subspacePhi != NULL)
	//	{
	//		VectorXd force(r);
	//		force = subspacePhi->transpose()*gravityforce;
	//	}
	//	forces.push_back(gravityforce);
	//}

}
