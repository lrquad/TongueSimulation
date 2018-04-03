#include "PointsCloud.h"
#include "LoboVolumetricMesh/LoboNodeBase.h"
#include <fstream>
#include <iostream>

PointsCloud::PointsCloud()
{
	numVertex = 0;
	m_nType = 0;
	m_nSlice = 0;
	m_nNodesPerSlice = 0;
}


PointsCloud::~PointsCloud()
{
}

void PointsCloud::readPointsBinary(const char* filepath, Vector3d translate)
{
	std::cout << "read mesh.." << std::endl;
	std::ifstream in(filepath, std::ios::in | std::ios::binary);
	if (!in.good())
	{
		std::cout << "file not open" << std::endl;
		return;
	}

	in.read((char*)&m_nType, sizeof(unsigned short));
	in.read((char*)&m_nSlice, sizeof(unsigned short));
	in.read((char*)&m_nNodesPerSlice, sizeof(unsigned short));

	std::cout << "m_nType " << m_nType << std::endl;
	std::cout << "m_nSlice " << m_nSlice << std::endl;
	std::cout << "m_nNodesPerSlice " << m_nNodesPerSlice << std::endl;
	std::cout << "Totaly we have " << m_nNodesPerSlice*m_nSlice << " vertices" << std::endl;
	numVertex = m_nNodesPerSlice*m_nSlice;

	float* data = new float[numVertex * 3];
	in.read((char*)data,sizeof(float)*numVertex*3);

	node_list.resize(numVertex);
	for (int i = 0; i < m_nSlice; i++)
	{
		for (int j = 0; j < m_nNodesPerSlice; j++)
		{
			int nodeid = i*m_nNodesPerSlice + j;
			node_list[nodeid] = new LoboNodeBase();
			node_list[nodeid]->ori_position.data()[0] = data[i*m_nNodesPerSlice * 3 + j];
			node_list[nodeid]->ori_position.data()[1] = -1 * data[i*m_nNodesPerSlice * 3 + 1 * m_nNodesPerSlice + j];
			node_list[nodeid]->ori_position.data()[2] = data[i*m_nNodesPerSlice * 3 + 2 * m_nNodesPerSlice + j];

			node_list[nodeid]->ori_position += translate;
		}
	}

	in.close();

	delete[] data;
}

void PointsCloud::readPointsBinaryFILE(const char* filepath)
{
	FILE* fp;
	fp = fopen(filepath, "rb");
	if (fp != NULL){
		fread(&m_nType, sizeof(unsigned short), 1, fp);
		fread(&m_nSlice, sizeof(unsigned short), 1, fp);
		fread(&m_nNodesPerSlice, sizeof(unsigned short), 1, fp);
		std::cout << "m_nType " << m_nType << std::endl;
		std::cout << "m_nSlice " << m_nSlice << std::endl;
		std::cout << "m_nNodesPerSlice " << m_nNodesPerSlice << std::endl;
		std::cout << "Totaly we have " << m_nNodesPerSlice*m_nSlice << " vertices" << std::endl;
		numVertex = m_nNodesPerSlice*m_nSlice;

		float *data = new float[numVertex * 3];
		int numRead = fread(data, sizeof(float), m_nSlice*m_nNodesPerSlice * 3, fp);
		fclose(fp);

		node_list.resize(numVertex);

		
		for (int i = 0; i < m_nSlice; i++)
		{
			for (int j = 0; j < m_nNodesPerSlice; j++)
			{
				int nodeid = i*m_nNodesPerSlice + j;
				node_list[nodeid] = new LoboNodeBase();
				node_list[nodeid]->ori_position.data()[0] = data[i*m_nNodesPerSlice * 3 + j];
				node_list[nodeid]->ori_position.data()[1] = -1 * data[i*m_nNodesPerSlice * 3 + 1 * m_nNodesPerSlice + j];
				node_list[nodeid]->ori_position.data()[2] = data[i*m_nNodesPerSlice * 3 + 2 * m_nNodesPerSlice + j];
				std::cout << node_list[nodeid]->ori_position.transpose() << std::endl;
			}
		}
		
	}
	
}

void PointsCloud::saveObj(const char* filename)
{
	std::ofstream output(filename);
	int numFace = (m_nNodesPerSlice-1)*(m_nSlice-1);

	for (int i = 0; i < m_nSlice; i++)
	{
		for (int j = 0; j < m_nNodesPerSlice; j++)
		{
			int nodeid = i*m_nNodesPerSlice + j;
			output << "v " << node_list[nodeid]->ori_position.transpose()<<std::endl;
		}
	}
	
	//for (int i = 0; i < m_nSlice - 1; i++)
	//{
	//	for (int j = 0; j < m_nNodesPerSlice - 1; j++)
	//	{
	//		int n1 = i*m_nNodesPerSlice + j;
	//		int n2 = n1 + 1;
	//		int n3 = (i + 1)*m_nNodesPerSlice + j;
	//		int n4 = n3 + 1;
	//		
	//		output << "f " << n1+1 << " " << n2+1 << " " << n4+1 << " " << n3+1 << std::endl;
	//	}
	//}

	for (int i = 0; i < m_nSlice - 1; i++)
	{
		for (int j = 0; j < m_nNodesPerSlice - 1; j++)
		{
			int n1, n2, n3, n4;
			n1 = i*m_nNodesPerSlice + j;
			n2 = (i + 1)*m_nNodesPerSlice + j;
			n3 = (i + 1)*m_nNodesPerSlice + j + 1;
			n4 = i*m_nNodesPerSlice + j + 1;

			output << "f " << n3 + 1 << " " << n2 + 1 << " " << n1 + 1 << std::endl;
			output << "f " << n4 + 1 << " " << n3 + 1 << " " << n1 + 1 << std::endl;
		}
	}

	output.close();
}

Eigen::Vector3d PointsCloud::getPointOriPosition(int pointid)
{
	return node_list[pointid]->ori_position;
}

Vector3d PointsCloud::getCurPosition(int pointid)
{
	return node_list[pointid]->cur_position;
}

int PointsCloud::searchColseNode(Vector3d input)
{
	double mindistance = DBL_MAX;
	int idx = -1;
	for (int i = 0; i < numVertex; i++)
	{
		double distance = (this->getPointOriPosition(i)-input).norm();
		if (distance < mindistance)
		{
			mindistance = distance;
			idx = i;
		}
	}
	return idx;
}

void PointsCloud::pushPoint(Vector3d position)
{
	LoboNodeBase* node =  new LoboNodeBase();
	node->ori_position = position;
	node_list.push_back(node);
	numVertex = node_list.size();
}

void PointsCloud::randomGeneratePoints2D(int targetNumVertexs)
{
	std::cout << "randomly generated points" << std::endl;

	std::srand(0);
	node_list.clear();
	node_list.resize(targetNumVertexs);
	numVertex = targetNumVertexs;
	
	for (int i = 0; i < targetNumVertexs; i++)
	{
		double x = (std::rand() % 20000 - 10000) / 10000.0;
		double y = (std::rand() % 20000 - 10000) / 10000.0;
		node_list[i] = new LoboNodeBase();
		node_list[i]->ori_position.setZero();
		node_list[i]->ori_position.data()[0] = x;
		node_list[i]->ori_position.data()[1] = y;
		node_list[i]->cur_position = node_list[i]->ori_position;
	}
}
