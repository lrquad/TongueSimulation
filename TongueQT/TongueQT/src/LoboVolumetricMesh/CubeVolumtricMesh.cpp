#include "CubeVolumtricMesh.h"
#include <iostream>
#include <fstream>
#include "CubeElement.h"
#include "Functions/findElementInVector.h"
#include "Functions/computeTriangle.h"
#include "LoboNodeBase.h"

CubeVolumtricMesh::CubeVolumtricMesh()
{
	cubeNodeOrderMap[6] = 0;
	cubeNodeOrderMap[5] = 1;
	cubeNodeOrderMap[2] = 2;
	cubeNodeOrderMap[1] = 3;

	cubeNodeOrderMap[7] = 4;
	cubeNodeOrderMap[4] = 5;
	cubeNodeOrderMap[3] = 6;
	cubeNodeOrderMap[0] = 7;
}


CubeVolumtricMesh::~CubeVolumtricMesh()
{

}

void CubeVolumtricMesh::readElementList()
{
	element_list.resize(numElements);
	for (int i = 0; i < numElements; i++)
	{
		element_list[i] = new CubeElement;
		element_list[i]->index_ = i;
		element_list[i]->node_indices.resize(numer_element_vertcies);

		for (int j = 0; j < 8; j++)
		{
			element_list[i]->node_indices[j] = elements[i * 8 + j];
		}

		for (int j = 0; j < 8; j++)
		{
			node_list[element_list[i]->node_indices[j]]->element_list.push_back(i);
		}
	}

}

void CubeVolumtricMesh::readElementMesh(const char* filenamebase, double* translate /*= NULL*/, double scale /*= 1.0*/)
{
	LoboVolumetricMesh::readElementMesh(filenamebase, translate, scale);
	initVolumetricMesh();
	this->searchNeighborElement();
}

void CubeVolumtricMesh::saveElementMesh(std::ofstream &outstream)
{

}

void CubeVolumtricMesh::saveElementMeshAscii(const char* filenamebase, std::vector<LoboNodeBase*> &newNodeList, std::vector<LoboElement*> &newElementList)
{
	std::cout << "save new vox mesh ..." << std::endl;
	std::cout << "num elements: " << newElementList.size() << std::endl;

	std::ostringstream data;
	data << filenamebase << ".ele";
	std::string elefilename = data.str();

	std::ostringstream data2;
	data2 << filenamebase << ".node";
	std::string nodefilename = data2.str();


	std::ofstream eleoutput(elefilename);
	eleoutput << newElementList.size() << " " << numer_element_vertcies << " " << 0 << std::endl;
	for (int i = 0; i < newElementList.size(); i++)
	{
		eleoutput << i << " ";

		LoboElement* ele = newElementList[i];
		for (int j = 0; j < ele->node_indices.size(); j++)
		{
			int nodeid = ele->node_indices[j];
			eleoutput << nodeid << " ";
		}
		eleoutput << std::endl;
	}
	eleoutput.close();

	std::ofstream nodeoutput(nodefilename);
	nodeoutput << newNodeList.size() << " " << 3 << " " << 0 << " " << 0 << std::endl;
	int nodeindex = 0;

	for (int i = 0; i < newNodeList.size(); i++)
	{
		nodeoutput << i << " ";
		Vector3d position = newNodeList[i]->ori_position;
		nodeoutput << position.transpose() << std::endl;
	}
	nodeoutput.close();
}

void CubeVolumtricMesh::initVolumetricMesh()
{
	for (int i = 0; i < element_list.size(); i++)
	{
		computeElementAttri(downCastCubeElement(element_list[i]));
	}
}

void CubeVolumtricMesh::exportNewCubeVolumtricMeshByNode(const char* filenamebase)
{
	std::vector<LoboElement*> newCubeElementList;
	std::vector<LoboNodeBase*>  newNodeList;
	std::vector<std::vector<int*>> nodePointerList;

	newCubeElementList.resize(node_list.size());
	double curedge = downCastCubeElement(element_list[0])->edge_length;

	std::vector<int> element_ndoeindices(node_list.size()*numer_element_vertcies);


	std::vector<bool> element_node_init(node_list.size()*numer_element_vertcies);
	std::fill(element_node_init.begin(), element_node_init.end(), false);

	for (int i = 0; i < node_list.size(); i++)
	{
		newCubeElementList[i] = new CubeElement;
		newCubeElementList[i]->index_ = i;
		newCubeElementList[i]->node_indices.resize(numer_element_vertcies);
		std::fill(newCubeElementList[i]->node_indices.begin(), newCubeElementList[i]->node_indices.end(), 0);
	}

	//start merge
	std::cout << "start merge" << std::endl;
	for (int i = 0; i < node_list.size(); i++)
	{
		CubeElement* curElement = downCastCubeElement(newCubeElementList[i]);
		
		//loop the neighbor
		for (int j = 0; j < node_list[i]->neighbor.size(); j++)
		{
			int neighbornodeid = node_list[i]->neighbor[j];
			
			CubeElement* neighborEle = downCastCubeElement(newCubeElementList[neighbornodeid]);

			int faceid = computeNeighborDirection(node_list[i]->ori_position, node_list[neighbornodeid]->ori_position);

			for (int k = 0; k < 4; k++)
			{
				int nodeindex = curElement->face_indices[faceid * 4 + k];
				int neighborNodeindex = curElement->inverface_indeices[faceid * 4 + k];

				bool curinit = element_node_init[i*numer_element_vertcies + nodeindex];
				bool neighborinit = element_node_init[neighbornodeid*numer_element_vertcies + neighborNodeindex];

				if ((!curinit) && (!neighborinit))
				{
					// generate new ndoe
					int m = cubeNodeOrderMap[nodeindex];
					int c[3];
					c[0] = m % 2;
					c[1] = m % 4 / 2;
					c[2] = m / 4;

					Vector3d nodeCenter;
					for (int l = 0; l < 3; l++)
					{
						nodeCenter.data()[l] = node_list[i]->ori_position.data()[l] + (c[l] * 2 - 1)*curedge / 2.0;
					}

					LoboNodeBase* node = new LoboNodeBase();
					node->index_ = newNodeList.size();
					node->ori_position = nodeCenter;
					newNodeList.push_back(node);

					element_node_init[i*numer_element_vertcies + nodeindex] = true;
					element_node_init[neighbornodeid*numer_element_vertcies + neighborNodeindex] = true;

					curElement->node_indices[nodeindex] = node->index_;
					neighborEle->node_indices[neighborNodeindex] = node->index_;

					std::vector<int*> nodepointer;
					nodepointer.push_back(&(curElement->node_indices[nodeindex]));
					nodepointer.push_back(&(neighborEle->node_indices[neighborNodeindex]));
					nodePointerList.push_back(nodepointer);

				}else
				if ((curinit) && (!neighborinit))
				{
					int nodeid =  curElement->node_indices[nodeindex];
					neighborEle->node_indices[neighborNodeindex] = nodeid;
					nodePointerList[nodeid].push_back(&(neighborEle->node_indices[neighborNodeindex]));
					element_node_init[neighbornodeid*numer_element_vertcies + neighborNodeindex] = true;
				}else
				if ((!curinit) && (neighborinit))
				{
					int nodeid = neighborEle->node_indices[neighborNodeindex];
					curElement->node_indices[nodeindex] = nodeid;
					nodePointerList[nodeid].push_back(&(curElement->node_indices[nodeindex]));
					element_node_init[i*numer_element_vertcies + nodeindex] = true;
				}
				else
				{
					//check if merged
					if (curElement->node_indices[nodeindex] != neighborEle->node_indices[neighborNodeindex])
					{
						int curnodeid = curElement->node_indices[nodeindex];
						int neighborid = neighborEle->node_indices[neighborNodeindex];
						std::cout << curnodeid << " " << neighborid << std::endl;
						for (int n = 0; n < nodePointerList[curnodeid].size(); n++)
						{
							(*(nodePointerList[curnodeid][n])) = neighborid;
							nodePointerList[neighborid].push_back(nodePointerList[curnodeid][n]);
						}
					}
				}
			}
		}

		for (int j = 0; j < 8; j++)
		{
			if (!element_node_init[i*numer_element_vertcies + j])
			{
				int m = cubeNodeOrderMap[j];
				int c[3];
				c[0] = m % 2;
				c[1] = m % 4 / 2;
				c[2] = m / 4;

				Vector3d nodeCenter;
				for (int l = 0; l < 3; l++)
				{
					nodeCenter.data()[l] = node_list[i]->ori_position.data()[l] + (c[l] * 2 - 1)*curedge / 2.0;
				}

				LoboNodeBase* node = new LoboNodeBase();
				node->index_ = newNodeList.size();
				node->ori_position = nodeCenter;
				newNodeList.push_back(node);

				curElement->node_indices[j] = node->index_;
				element_node_init[i*numer_element_vertcies + j] = true;


				std::vector<int*> nodepointer;
				nodepointer.push_back(&(curElement->node_indices[j]));
				nodePointerList.push_back(nodepointer);
			}
		}

	}


	std::cout << "saveElementMeshAscii" << std::endl;
	saveElementMeshAscii(filenamebase, newNodeList, newCubeElementList);

}

void CubeVolumtricMesh::searchNeighborElement()
{
	for (int i = 0; i < element_list.size(); i++)
	{
		CubeElement* element = downCastCubeElement(element_list[i]);

		element->neighbors.resize(6);
		for (int f = 0; f < 6; f++)
		{
			element->neighbors[f] = -1;

			int nodeindex[4];
			for (int j = 0; j < 4; j++)
			{
				nodeindex[j] = element->node_indices[element->face_indices[f * 4 + j]];
			}

			for (int i_1 = 0; i_1 < node_list[nodeindex[0]]->element_list.size(); i_1++)
			{
				int eleid = node_list[nodeindex[0]]->element_list[i_1];
				if (eleid != i)
				{
					bool isNeighbor = true;
					for (int j = 1; j < 4; j++)
					{
						if(!findElement(node_list[nodeindex[j]]->element_list, eleid))
						isNeighbor = false;
					}

					if (isNeighbor)
					{
						element->neighbors[f] = eleid;
						break;
					}
				}
			}
		}

		int count = 0;
		for (int f = 0; f < 6; f++)
		{
			if (element->neighbors[f] != -1)
			{
				count++;
			}
		}
		if (count < 3)
		{
			std::cout << count << std::endl;
		}

	}

	//search node's neighbor

	for (int i = 0; i < node_list.size(); i++)
	{
		node_list[i]->neighbor.clear();
	}

	for (int i = 0; i < numElements; i++)
	{
		for (int j = 0; j < 8; j++)
		{
			int nodeidj = element_list[i]->node_indices[j];
			Vector3d nodepj = node_list[nodeidj]->ori_position;
			for (int m = j + 1; m < 8; m++)
			{
				int nodeidm = element_list[i]->node_indices[m];
				Vector3d nodepm = node_list[nodeidm]->ori_position;

				if (this->computeNeighborDirection(nodepj, nodepm) == -1)
				{
					continue;
				}

				if (!findElement(node_list[nodeidj]->neighbor, nodeidm))
				{
					node_list[nodeidj]->neighbor.push_back(nodeidm);
				}

				if (!findElement(node_list[nodeidm]->neighbor, nodeidj))
				{
					node_list[nodeidm]->neighbor.push_back(nodeidj);
				}
			}
		}
	}
}

bool CubeVolumtricMesh::containsVertex(int element, const Vector3d &pos) const
{
	Vector3d center = downCastCubeElement(element_list[element])->cube_center;
	double edge = downCastCubeElement(element_list[element])->edge_length/2.0;
	double distance = (pos - center).lpNorm<1>();

	if (pos.x() < center.x() - edge || pos.x() > center.x() + edge)
	{
		return false;
	}

	if (pos.y() < center.y() - edge || pos.y() > center.y() + edge)
	{
		return false;
	}

	if (pos.z() < center.z() - edge || pos.z() > center.z() + edge)
	{
		return false;
	}

	return true;
}

void CubeVolumtricMesh::computeMassMatrix(std::vector<LoboElement*>* elementlist, SparseMatrix<double>* mass) const
{

}

void CubeVolumtricMesh::computeMassMatrix(SparseMatrix<double>* mass)
{

}

void CubeVolumtricMesh::computeBarycentricWeights(int elementid, const Vector3d &pos, Vector4d &weight) const
{

}

void CubeVolumtricMesh::computeSurface()
{

}

void CubeVolumtricMesh::releaseSurface()
{

}

void CubeVolumtricMesh::saveSurfaceToObjAscii(const char* filename) const
{

}

void CubeVolumtricMesh::computeNodeVolume(double *nodeweight)
{

}

double CubeVolumtricMesh::computeCurrentMeshVolume()
{
	return 0;
}

void CubeVolumtricMesh::computeDeformationGradient(int elementid, Matrix3d &output)
{

}

CubeElement* CubeVolumtricMesh::getCubeElement(int index)
{
	return downCastCubeElement(element_list[index]);
}

void CubeVolumtricMesh::protrudingCube(int selectedCube, int type)
{

}

void CubeVolumtricMesh::computeElementAttri(CubeElement* element)
{
	Vector3d no0 = this->getNodeRestPosition(element->node_indices[0]);
	Vector3d no1 = this->getNodeRestPosition(element->node_indices[1]);
	element->edge_length = (no0 - no1).norm();
	element->volume_ = std::pow(element->edge_length, 3);

	Vector3d center;
	center.setZero();
	for (int i = 0; i < element->node_indices.size(); i++)
	{
		center += this->getNodeRestPosition(element->node_indices[i]);
	}
	center /= element->node_indices.size();
	element->cube_center = center;
}


int CubeVolumtricMesh::computeNeighborDirection(Vector3d centeri, Vector3d centerj)
{
	double edge = (centerj - centeri).norm();
	int direction = -1;

	if (std::abs((centerj.x() - centeri.x()) + edge)/edge < 1e-10)
	{
		direction = 2; 
		return direction;
	}

	if (std::abs((centerj.x() - centeri.x()) - edge) / edge < 1e-10)
	{
		direction = 3;
		return direction;
	}

	if (std::abs((centerj.y() - centeri.y()) + edge) / edge < 1e-10)
	{
		direction = 5;
		return direction;
	}

	if (std::abs((centerj.y() - centeri.y()) - edge) / edge < 1e-10)
	{
		direction = 4;
		return direction;
	}

	if (std::abs((centerj.z() - centeri.z()) + edge) / edge < 1e-10)
	{
		direction = 1;
		return direction;
	}

	if (std::abs((centerj.z() - centeri.z()) - edge) / edge < 1e-10)
	{
		direction = 0;
		return direction;
	}

	return direction;

}

bool CubeVolumtricMesh::mergeNodePosition(Vector3d centeri, Vector3d centerj, double edge)
{
	double distance = (centerj - centeri).norm();

	if (distance < edge / 100.0)
	{
		return true;
	}
	else
	{
		return false;
	}

}

CubeVolumtricMesh* downCastCubeVolMesh(LoboVolumetricMesh* mesh)
{
	if (mesh->getMeshType() == LoboVolumetricMesh::VolumetricmeshType::CUBE)
	{
		return (CubeVolumtricMesh*)mesh;
	}
	else
	{
		return NULL;
	}
}
