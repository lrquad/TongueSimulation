#include "LoboVolumetricMeshObjAdapter.h"
#include "LoboVolumetricMesh/LoboVolumetricMesh.h"
#include "Render/LoboObjMesh.h"
#include "LoboVolumetricMesh/LoboElement.h"
#include <fstream>


LoboVolumetricMeshObjAdapter::LoboVolumetricMeshObjAdapter(LoboVolumetricMesh* volumetric_mesh, LoboObjMesh* objmesh)
{
	this->volumetric_mesh = volumetric_mesh;
	this->objmesh = objmesh;
}

LoboVolumetricMeshObjAdapter::LoboVolumetricMeshObjAdapter()
{
	
}

LoboVolumetricMeshObjAdapter::~LoboVolumetricMeshObjAdapter()
{
}

void LoboVolumetricMeshObjAdapter::computeNodeBarycentricWeight()
{
	int numTrinode = objmesh->getNumberofVertex();
	int numVertex = volumetric_mesh->getNumVertices();

	barycentricWeightMatrix.resize(numTrinode, 4);
	barycentricWeightMatrix.setZero();
	node_element_map.resize(numTrinode);
	std::fill(node_element_map.begin(), node_element_map.end(), -1);

	barycentricPhi.resize(numTrinode * 3, numVertex * 3);
	std::vector<Eigen::Triplet<double>> sparseEntry;

	for (int i = 0; i < numTrinode; i++)
	{
		Vector3d position = objmesh->getOriPosition(i);
		Vector4d baycentric;
		int eleid = volumetric_mesh->getContainedElement(position);
		if (eleid == -1)
		{
			//std::cout << i <<" "<< "not found" << std::endl;
			continue;
		}
		node_element_map[i] = eleid;
		volumetric_mesh->computeBarycentricWeights(eleid, position, baycentric);
		LoboElement* element = volumetric_mesh->getElement(eleid);
		for (int j = 0; j < element->node_indices.size(); j++)
		{
			int nodeid = element->node_indices[j];
			barycentricWeightMatrix.data()[j*numTrinode + i] = baycentric.data()[j];
			for (int x = 0; x < 3; x++)
			{
				int row = i * 3 + x;
				int col = nodeid * 3 + x;
				sparseEntry.push_back(Eigen::Triplet<double>(row, col, baycentric.data()[j]));
			}
		}
	}
	
	barycentricPhi.setFromTriplets(sparseEntry.begin(), sparseEntry.end());
}

void LoboVolumetricMeshObjAdapter::updateObjMesh()
{
	int numVertex = volumetric_mesh->getNumVertices();
	int numTrinode = objmesh->getNumberofVertex();
	VectorXd volumetricdisplacement(numVertex * 3);
	volumetric_mesh->getDisplacement(volumetricdisplacement.data());

	VectorXd tridisplacement(numTrinode*3);//= barycentricPhi*volumetricdisplacement;
	computeTriDisplacement(volumetricdisplacement.data(),tridisplacement.data());
	objmesh->updateDisplacement(tridisplacement.data());
}

void LoboVolumetricMeshObjAdapter::computeTriDisplacement(double* tetdisplacement, double* tridisplacement)
{
	int numVertex = volumetric_mesh->getNumVertices();
	int numTrinode = objmesh->getNumberofVertex();
	memset(tridisplacement, 0, sizeof(double)*numTrinode * 3);

	for (int i = 0; i < numTrinode; i++)
	{
		int ele = node_element_map[i];
		if (ele == -1)
		{
			continue;
		}

		LoboElement* element = volumetric_mesh->getElement(ele);
		for (int j = 0; j < element->node_indices.size(); j++)
		{
			int nodeid = element->node_indices[j];
			double weight = barycentricWeightMatrix.data()[j*numTrinode + i];

			tridisplacement[i * 3 + 0] += weight*tetdisplacement[nodeid * 3 + 0];
			tridisplacement[i * 3 + 1] += weight*tetdisplacement[nodeid * 3 + 1];
			tridisplacement[i * 3 + 2] += weight*tetdisplacement[nodeid * 3 + 2];
		}
	}
}

