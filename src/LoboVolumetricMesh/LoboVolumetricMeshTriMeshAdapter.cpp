#include "LoboVolumetricMeshTriMeshAdapter.h"
#include "LoboTriMesh/LoboTriMeshH.h"
#include "LoboVolumetricMesh/LoboVolumetricMesh.h"
#include "LoboVolumetricMesh/LoboElement.h"
#include <fstream>
#include <iostream>
#include "ColorTable/Colortable.h"

LoboVolumetricMeshTriMeshAdapter::LoboVolumetricMeshTriMeshAdapter(LoboVolumetricMesh* volumetric_mesh, LoboTriMesh* triMesh)
{
	this->volumetric_mesh = volumetric_mesh;
	this->triMesh = triMesh;

	this->color1 = Vector3d(1, 0.2, 0.2);
	this->color2 = Vector3d(0.2, 1, 0.2);

	initTriMeshMap();
}

LoboVolumetricMeshTriMeshAdapter::~LoboVolumetricMeshTriMeshAdapter()
{
}

void LoboVolumetricMeshTriMeshAdapter::computeNodeBarycentricWeight()
{
	int numTrinode = triMesh->getNumVertex();
	int numVertex = volumetric_mesh->getNumVertices();

	barycentricWeightMatrix.resize(numTrinode, 4);
	barycentricWeightMatrix.setZero();
	node_element_map.resize(numTrinode);
	std::fill(node_element_map.begin(), node_element_map.end(), -1);

	barycentricPhi.resize(numTrinode * 3, numVertex * 3);
	sparseEntry.clear();

	for (int i = 0; i < numTrinode; i++)
	{
		Vector3d position = triMesh->getOriPosition(i);
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

void LoboVolumetricMeshTriMeshAdapter::updateObjMesh()
{
	int numTrinode = triMesh->getNumVertex();
	int numVertex = volumetric_mesh->getNumVertices();

	VectorXd volumetricdisplacement(numVertex * 3);
	volumetric_mesh->getDisplacement(volumetricdisplacement.data());

	VectorXd tridisplacement(numTrinode * 3);

	computeTriDisplacement(volumetricdisplacement.data(), tridisplacement.data());

	triMesh->updateTriDisplacement(tridisplacement.data());
}

int LoboVolumetricMeshTriMeshAdapter::getVolumeNodeByTriNodeId(int trinodeid)
{
	return trinode_node_map[trinodeid];
}

void LoboVolumetricMeshTriMeshAdapter::setTriMeshVertexColorData1D(double* volumenodedata)
{
	int numTriVertex = triMesh->getNumVertex();
	int numVolumeVertex= volumetric_mesh->getNumVertices();
	VectorXd tridisplacement(numTriVertex * 3);
	VectorXd volumetricdisplacement(numVolumeVertex * 3);

	for (int i = 0; i < numVolumeVertex; i++)
	{
		//volumenodedata[i] *= 1.5;
		if (volumenodedata[i] > 1)
		{
			volumenodedata[i] = 1;
		}
		if (volumenodedata[i] < 0)
		{
			volumenodedata[i] = 0;
		}

		Vector3d color = (color2 - color1)*volumenodedata[i] + color1;
		volumetricdisplacement.data()[i * 3 + 0] = color.data()[0];
		volumetricdisplacement.data()[i * 3 + 1] = color.data()[1];
		volumetricdisplacement.data()[i * 3 + 2] = color.data()[2];
	}
	
	computeTriDisplacement(volumetricdisplacement.data(), tridisplacement.data());

	triMesh->setVertexColorData3D(tridisplacement.data());
}

void LoboVolumetricMeshTriMeshAdapter::convertVolumtricDataToTri1D(double* volumenodedata, double* trinodedata)
{
	int numTriVertex = triMesh->getNumVertex();
	int numVolumeVertex = volumetric_mesh->getNumVertices();
	VectorXd tridisplacement(numTriVertex * 3);
	VectorXd volumetricdisplacement(numVolumeVertex * 3);

	for (int i = 0; i < numVolumeVertex; i++)
	{
		//volumenodedata[i] *= 1.5;
		volumetricdisplacement.data()[i * 3 + 0] = volumenodedata[i];
		volumetricdisplacement.data()[i * 3 + 1] = volumenodedata[i];
		volumetricdisplacement.data()[i * 3 + 2] = volumenodedata[i];
	}

	computeTriDisplacement(volumetricdisplacement.data(), tridisplacement.data());

	for (int i = 0; i < numTriVertex; i++)
	{
		trinodedata[i] = tridisplacement.data()[i * 3];
	}

}

void LoboVolumetricMeshTriMeshAdapter::convertVolumtricDataToTri1DNoInterpolate(double* volumenodedata, double* trinodedata)
{
	int numTriVertex = triMesh->getNumVertex();
	int numVolumeVertex = volumetric_mesh->getNumVertices();
	VectorXd tridisplacement(numTriVertex * 3);
	VectorXd volumetricdisplacement(numVolumeVertex * 3);

	for (int i = 0; i < numVolumeVertex; i++)
	{
		//volumenodedata[i] *= 1.5;
		volumetricdisplacement.data()[i * 3 + 0] = volumenodedata[i];
		volumetricdisplacement.data()[i * 3 + 1] = volumenodedata[i];
		volumetricdisplacement.data()[i * 3 + 2] = volumenodedata[i];
	}

	computeTriDisplacementNoInterpolate(volumetricdisplacement.data(), tridisplacement.data());

	for (int i = 0; i < numTriVertex; i++)
	{
		trinodedata[i] = tridisplacement.data()[i * 3];
	}
}

void LoboVolumetricMeshTriMeshAdapter::setTriMeshVertexColorData3D(double* volumenodedata)
{
	int numTriVertex = triMesh->getNumVertex();
	int numVolumeVertex = volumetric_mesh->getNumVertices();
	VectorXd tridisplacement(numTriVertex * 3);
	computeTriDisplacement(volumenodedata, tridisplacement.data());
	triMesh->setVertexColorData3D(tridisplacement.data());
}

void LoboVolumetricMeshTriMeshAdapter::initTriMeshMap()
{
	int numTrinode = triMesh->getNumVertex();
	int numVertex = volumetric_mesh->getNumVertices();
	trinode_node_map.resize(numTrinode);

	for (int i = 0; i < numTrinode; i++)
	{
		Vector3d position = triMesh->getOriPosition(i);
		trinode_node_map[i] = volumetric_mesh->searchCloseNode(position);
	}
}

void LoboVolumetricMeshTriMeshAdapter::computeTriDisplacement(double* tetdisplacement, double* tridisplacement)
{
	int numVertex = volumetric_mesh->getNumVertices();
	int numTrinode = triMesh->getNumVertex();
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

void LoboVolumetricMeshTriMeshAdapter::computeTriDisplacementNoInterpolate(double* tetdisplacement, double* tridisplacement)
{
	int numVertex = volumetric_mesh->getNumVertices();
	int numTrinode = triMesh->getNumVertex();
	memset(tridisplacement, 0, sizeof(double)*numTrinode * 3);

	for (int i = 0; i < numTrinode; i++)
	{
		int ele = node_element_map[i];
		if (ele == -1)
		{
			continue;
		}

		LoboElement* element = volumetric_mesh->getElement(ele);
		int nodeid = element->node_indices[0];
		tridisplacement[i * 3 + 0] = tetdisplacement[nodeid * 3 + 0];
		tridisplacement[i * 3 + 1] = tetdisplacement[nodeid * 3 + 1];
		tridisplacement[i * 3 + 2] = tetdisplacement[nodeid * 3 + 2];
		
	}
}

void LoboVolumetricMeshTriMeshAdapter::saveInfoBinary(std::ofstream &outstream)
{
	int mapsize = trinode_node_map.size();
	outstream.write((char*)&mapsize, sizeof(int));
	outstream.write((char*)trinode_node_map.data(), sizeof(int)*mapsize);

	int elemapsize = node_element_map.size();
	outstream.write((char*)&elemapsize, sizeof(int));
	outstream.write((char*)node_element_map.data(), sizeof(int)*elemapsize);

	int row = barycentricWeightMatrix.rows();
	int col = barycentricWeightMatrix.cols();

	outstream.write((char*)&row, sizeof(int));
	outstream.write((char*)&col, sizeof(int));

	outstream.write((char*)barycentricWeightMatrix.data(), sizeof(MatrixXd::Scalar)*row*col);

	int entry_size = sparseEntry.size();
	outstream.write((char*)&entry_size, sizeof(int));
	for (int i = 0; i < entry_size; i++)
	{
		int r = sparseEntry[i].row();
		int c = sparseEntry[i].col();
		double v = sparseEntry[i].value();
		outstream.write((char*)&r, sizeof(int));
		outstream.write((char*)&c, sizeof(int));
		outstream.write((char*)&v, sizeof(double));
	}

	int phir,phic;
	phir = barycentricPhi.rows();
	phic = barycentricPhi.cols();
	outstream.write((char*)&phir, sizeof(int));
	outstream.write((char*)&phic, sizeof(int));
}

void LoboVolumetricMeshTriMeshAdapter::readInfoBinary(std::ifstream &instream)
{
	int mapsize;
	instream.read((char*)&mapsize, sizeof(int));
	trinode_node_map.resize(mapsize);
	instream.read((char*)trinode_node_map.data(), sizeof(int)*mapsize);

	int elemapsize;
	instream.read((char*)&elemapsize, sizeof(int));
	node_element_map.resize(elemapsize);
	instream.read((char*)node_element_map.data(), sizeof(int)*elemapsize);
	
	int row, col;
	instream.read((char*)&row, sizeof(int));
	instream.read((char*)&col, sizeof(int));

	barycentricWeightMatrix.resize(row, col);
	instream.read((char*)barycentricWeightMatrix.data(), sizeof(MatrixXd::Scalar)*row*col);
	
	int entry_size;
	instream.read((char*)&entry_size, sizeof(int));
	sparseEntry.clear();

	for (int i = 0; i < entry_size; i++)
	{
		int r;
		int c;
		double v;

		instream.read((char*)&r, sizeof(int));
		instream.read((char*)&c, sizeof(int));
		instream.read((char*)&v, sizeof(double));

		sparseEntry.push_back(Eigen::Triplet<double>(r, c, v));
	}

	int phir, phic;
	//phir = barycentricPhi.rows();
	//phic = barycentricPhi.cols();
	instream.read((char*)&phir, sizeof(int));
	instream.read((char*)&phic, sizeof(int));

	barycentricPhi.resize(phir, phic);
	barycentricPhi.setFromTriplets(sparseEntry.begin(), sparseEntry.end());

}

void LoboVolumetricMeshTriMeshAdapter::exportVertexColorMesh1D(const char* filename, double* volumenodedata)
{
	int numTriVertex = this->triMesh->getNumVertex();
	VectorXd triweights(numTriVertex);
	triweights.setZero();

	this->convertVolumtricDataToTri1D(volumenodedata, triweights.data());

	double maxweight = triweights.maxCoeff();
	double minweight = triweights.minCoeff();

	double length = maxweight - minweight;
	
	for (int i = 0; i < numTriVertex; i++)
	{
		if (triweights[i] > 0.99)
		{
			triweights[i] = 0.99;
		}

		if (triweights[i] < 0.01)
		{
			triweights[i] = 0.01;
		}
	}

	VectorXd triTexture(numTriVertex * 2);
	triTexture.setZero();
	for (int j = 0; j < numTriVertex; j++)
	{
		triTexture.data()[j * 2 + 0] = 0.05;
		triTexture.data()[j * 2 + 1] = triweights[j];
	}
	triMesh->setVertexTexCoor(triTexture.data());
	triMesh->exportObj(filename);
	std::cout << filename << std::endl;
}

void LoboVolumetricMeshTriMeshAdapter::setColorMap(Vector3d color1_, Vector3d color2_)
{
	this->color1 = color1_;
	this->color2 = color2_;
}

void LoboVolumetricMeshTriMeshAdapter::markTriNodes(std::vector<int> elementMark, int coloroffset /*= 0*/)
{
	
	int numTriVertex = triMesh->getNumVertex();
	VectorXd tridisplacement(numTriVertex * 3);
	tridisplacement.setZero();
	for (int i = 0; i < numTriVertex; i++)
	{
		int ele = node_element_map[i];
		if (ele == -1)
		{
			continue;
		}

		int colormark = elementMark[ele] + coloroffset;
		Vector3d color;
		getColor(colormark, color.data());
		tridisplacement.data()[i * 3 + 0] = color.data()[0];
		tridisplacement.data()[i * 3 +1] = color.data()[1];
		tridisplacement.data()[i * 3 + 2] = color.data()[2];
	}
	triMesh->setVertexColorData3D(tridisplacement.data());
}

void LoboVolumetricMeshTriMeshAdapter::sendVertexDataToTriMesh(VectorXd &nodeDataInpude, int dimension)
{
	int numTrinode = triMesh->getNumVertex();
	int numVertex = volumetric_mesh->getNumVertices();
	//trinode_node_map.resize(numTrinode);
	VectorXd trinodeDataInpue(numTrinode*dimension);
	for (int i = 0; i < numTrinode; i++)
	{
		for (int j = 0; j < dimension; j++)
		{
			trinodeDataInpue.data()[i*dimension + j] = nodeDataInpude.data()[trinode_node_map[i] * dimension + j];
		}
	}
	triMesh->vertexData = trinodeDataInpue;
	triMesh->setMeshUpdated(true);
}
