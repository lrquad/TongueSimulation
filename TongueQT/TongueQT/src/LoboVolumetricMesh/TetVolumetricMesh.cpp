#include <iostream>
#include <fstream>
#include "TetVolumetricMesh.h"
#include "TetElement.h"
#include "LoboNodeBase.h"
#include "Functions/findElementInVector.h"
#include "Functions/computeTriangle.h"
typedef Eigen::Triplet<double> EIGEN_TRI;

TetVolumetricMesh::TetVolumetricMesh()
{
}


TetVolumetricMesh::~TetVolumetricMesh()
{

}

void TetVolumetricMesh::readElementMeshOnly(const char* filenamebase, double* translate /*= NULL*/, double scale /*= 1.0*/)
{
	LoboVolumetricMesh::readElementMesh(filenamebase, translate, scale);
}


void TetVolumetricMesh::initTetVolumetricMesh()
{
	initVolumetricMesh();
	this->searchNeighborElement();
	this->computeSurface();
}

void TetVolumetricMesh::readElementMesh(const char* filenamebase, double* translate /*= NULL*/, double scale /*= 1.0*/)
{
	LoboVolumetricMesh::readElementMesh(filenamebase, translate, scale);
	//this->readElementList();
	//need correction element's order
	initVolumetricMesh();
	this->searchNeighborElement();
	this->computeSurface();
}

void TetVolumetricMesh::readElementMesh(std::ifstream &instream)
{
	LoboVolumetricMesh::readElementMesh(instream);
	initVolumetricMesh();
	this->searchNeighborElement();
	this->computeSurface();
}

void TetVolumetricMesh::saveElementMesh(std::ofstream &outstream)
{
	LoboVolumetricMesh::saveElementMesh(outstream);
}

void TetVolumetricMesh::computeElementVolume(TetElement* te)
{
	Vector3d a = this->getNodeRestPosition(te->node_indices[0]);
	Vector3d b = this->getNodeRestPosition(te->node_indices[1]);
	Vector3d c = this->getNodeRestPosition(te->node_indices[2]);
	Vector3d d = this->getNodeRestPosition(te->node_indices[3]);
	te->volume_ = computeTetVolumeABS(a, b, c, d);
}


double TetVolumetricMesh::computeTetVolumeABS(Vector3d &a, Vector3d &b, Vector3d &c, Vector3d &d)
{
	Matrix3d D;
	D.col(0) = b - a;
	D.col(1) = c - a;
	D.col(2) = d - a;

	return D.determinant()/6.0;
	//return (1.0 / 6 * fabs((a - d).dot((b - d).cross(c - d))));
}

double TetVolumetricMesh::computeTetVolume(Vector3d &a, Vector3d &b, Vector3d &c, Vector3d &d)
{
	return (1.0 / 6 * ((a - d).dot((b - d).cross(c - d))));
}

void TetVolumetricMesh::correctElementNodeOrder(TetElement * element)
{
	int n0, n1, n2,n3;
	n0 = element->node_indices[0];
	n1 = element->node_indices[1];
	n2 = element->node_indices[2];
	n3 = element->node_indices[3];
	Vector3d direction_v;
	computeTriangleNorm(node_list[n0]->ori_position, node_list[n1]->ori_position, node_list[n2]->ori_position, direction_v);

	Vector3d n3n0 = node_list[n3]->ori_position - node_list[n0]->ori_position;
	if (n3n0.dot(direction_v) < 0)
	{
		element->node_indices[1] = n2;
		element->node_indices[2] = n1;
	}

	for (int i = 0; i < 4; i++)
	{
		int n0, n1, n2;
		n0 = element->node_indices[element->face_indices[i * 3 + 0]];
		n1 = element->node_indices[element->face_indices[i * 3 + 1]];
		n2 = element->node_indices[element->face_indices[i * 3 + 2]];
		computeTriangleNorm(node_list[n0]->ori_position, node_list[n1]->ori_position, node_list[n2]->ori_position, element->faces_norm_[i]);

		int n3;
		for (int j = 0; j < element->node_indices.size(); j++)
		{
			if (!(element->node_indices[j] == n0 || element->node_indices[j] == n1 || element->node_indices[j] == n2))
			{
				n3 = element->node_indices[j];
				break;
			}
		}

		Vector3d n3n0 = node_list[n3]->ori_position - node_list[n0]->ori_position;
		if (n3n0.dot(element->faces_norm_[i])>0)
		{
			element->faces_norm_[i] *= -1;
			int temp = element->face_indices[i * 3 + 0];
			element->face_indices[i * 3 + 0] = element->face_indices[i * 3 + 2];
			element->face_indices[i * 3 + 2] = temp;
		}
	}


}

void TetVolumetricMesh::computeElementFaceArea(TetElement * element)
{
	for (int i = 0; i < 4; i++)
	{
		int n0, n1, n2;
		n0 = element->node_indices[element->face_indices[i * 3 + 0]];
		n1 = element->node_indices[element->face_indices[i * 3 + 1]];
		n2 = element->node_indices[element->face_indices[i * 3 + 2]];
		element->area[i] = computeTriangleArea(node_list[n0]->ori_position, node_list[n1]->ori_position, node_list[n2]->ori_position);
	}
}

void TetVolumetricMesh::computeElementCenter(TetElement* element)
{
	Vector3d center;
	center.setZero();
	for (int i = 0; i < 4; i++)
	{
		int nid = element->node_indices[i];
		center += this->getNodeRestPosition(nid);
	}
	center /= 4.0;
	element->setOriCenterPosition(center);
}

void TetVolumetricMesh::computeTetElementNodeNorm(TetElement* element)
{
	for (int n = 0; n < 4; n++)
	{
		element->nodes_norm_[n].setZero();
		for (int f = 0; f < 4; f++)
		{
			for (int fn = 0; fn < 3; fn++)
			{
				if (element->face_indices[f * 3 + fn] == n)
				{
					element->nodes_norm_[n] += element->area[f] * element->faces_norm_[f];
				}
			}
		}
		element->nodes_norm_[n] /= 3.0;
		//element->nodes_norm_[n].normalize();
	}
}

void TetVolumetricMesh::initVolumetricMesh()
{
	for (int i = 0; i < element_list.size(); i++)
	{
		correctElementNodeOrder(downCastTetelement(element_list[i]));
		computeElementVolume(downCastTetelement(element_list[i]));
		computeElementCenter(downCastTetelement(element_list[i]));
		computeElementFaceArea(downCastTetelement(element_list[i]));
		computeTetElementNodeNorm(downCastTetelement(element_list[i]));
		computeElementShapeFunctionDerivate(downCastTetelement(element_list[i]));
	}
}


void TetVolumetricMesh::readElementList()
{

	element_list.resize(numElements);
	for (int i = 0; i < numElements; i++)
	{
		element_list[i] = new TetElement;
		element_list[i]->index_ = i;
		element_list[i]->node_indices.resize(numer_element_vertcies);
		
		element_list[i]->node_indices[0] = elements[i * 4 + 0];
		element_list[i]->node_indices[1] = elements[i * 4 + 1];
		element_list[i]->node_indices[2] = elements[i * 4 + 2];
		element_list[i]->node_indices[3] = elements[i * 4 + 3];

		for (int j = 0; j < 4; j++)
		{
			node_list[element_list[i]->node_indices[j]]->element_list.push_back(i);
		}
	}
}


void TetVolumetricMesh::searchNeighborElement()
{
	for (int i = 0; i < element_list.size(); i++)
	{
		TetElement * element = downCastTetelement(element_list[i]);
		

		element->neighbors.resize(4);
		for (int f = 0; f < 4; f++)
		{
			element->neighbors[f] = -1;
			int n1, n2, n3;
			n1 = element->node_indices[element->face_indices[f * 3 + 0]];
			n2 = element->node_indices[element->face_indices[f * 3 + 1]];
			n3 = element->node_indices[element->face_indices[f * 3 + 2]];

			for (int i_1 = 0; i_1 < node_list[n1]->element_list.size(); i_1++)
			{
				int eleid = node_list[n1]->element_list[i_1];
				if (eleid != i)
				{
					if (findElement(node_list[n2]->element_list, eleid)
						&& findElement(node_list[n3]->element_list, eleid))
					{
						element->neighbors[f] = eleid;
						break;
					}
				}
			}
		}
	}

	// search node's neighbor

	for (int i = 0; i < node_list.size(); i++)
	{
		node_list[i]->neighbor.clear();
		
	}

	for (int i = 0; i < numElements; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			int nodeidj = element_list[i]->node_indices[j];
			for (int m = j + 1; m < 4; m++)
			{
				int nodeidm = element_list[i]->node_indices[m];
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

void TetVolumetricMesh::computeMassMatrix(SparseMatrix<double>* mass)
{
	computeMassMatrix(&element_list, mass);
}

void TetVolumetricMesh::computeMassMatrix(std::vector<LoboElement*>* elementlist, SparseMatrix<double>* mass) const
{
	std::vector<EIGEN_TRI> coefficients;
	for (int i = 0; i < elementlist->size(); i++)
	{
		TetElement * element = downCastTetelement(elementlist->at(i));
		Material* material = materials[element_material[i]];
		double vol = element->volume_;
		double density = material->getDensity();

		for (int j = 0; j < 4; j++)
		{
			for (int k = 0; k < 4; k++)
			{
				double mass_value = 0;
				if (j == k)
				{
					mass_value = density*vol / 10.0;
				}
				else
				{
					mass_value = density*vol / 20.0;
				}

				int row = element->node_indices[j] * 3;
				int col = element->node_indices[k] * 3;

				for (int d = 0; d < 3; d++)
				{
					coefficients.push_back(EIGEN_TRI(row + d, col + d, mass_value));
				}
			}
		}
	}
	mass->setFromTriplets(coefficients.begin(), coefficients.end());

}

void TetVolumetricMesh::computeMassMatrix2(std::vector<LoboElement*>* elementlist, SparseMatrix<double>* mass) const
{
	std::cout << "compute mass matrix 2" << std::endl;
	std::vector<EIGEN_TRI> coefficients;
	double totalmass = 0;
	for (int i = 0; i < elementlist->size(); i++)
	{
		TetElement * element = downCastTetelement(elementlist->at(i));
		Material* material = materials[element_material[i]];
		double vol = element->volume_;
		double density = material->getDensity();
		totalmass += density*vol;
	}

	for (int i = 0; i < node_list.size(); i++)
	{
		double nodemass = totalmass / node_list.size();
		for (int d = 0; d < 3; d++)
		{
			coefficients.push_back(EIGEN_TRI(i * 3 + d, i * 3 + d, nodemass));
		}
	}
	mass->setFromTriplets(coefficients.begin(), coefficients.end());
}

void TetVolumetricMesh::computeBarycentricWeights(int elementid, const Vector3d &pos, Vector4d &weight) const
{
	Vector4d point;
	point.data()[0] = pos.data()[0];
	point.data()[1] = pos.data()[1];
	point.data()[2] = pos.data()[2];
	point.data()[3] = 1;

	if (elementid == -1)
	{
		weight.setZero();
		std::cout << "-1" << std::endl;
		return;
	}

	weight = downCastTetelement(element_list[elementid])->shape_function_inv*point;
}

void TetVolumetricMesh::saveSurfaceToObjAscii(const char* filename) const
{
	std::cout << "This tet element mesh 's surface will save as :" << filename << std::endl;
	std::vector<int> node_map(this->node_list.size());
	std::fill(node_map.begin(), node_map.end(), -1);
	for (int i = 0; i < surface_node_.size(); i++)
	{
		node_map[surface_node_[i]] = i;
	}

	std::ofstream output(filename);
	output.precision(32);

	for (int i = 0; i < surface_node_.size(); i ++ )
	{
		int nodeindex = surface_node_[i];
		output << "v ";
		output << node_list[nodeindex]->ori_position.x() << " ";
		output << node_list[nodeindex]->ori_position.y() << " ";
		output << node_list[nodeindex]->ori_position.z() << std::endl;
	}

	int facenodei;
	for (int i = 0; i < surface_face_3i_.size() / 3; i++)
	{
		output << "f ";
		for (int j = 0; j < 3; j++)
		{
			facenodei = surface_face_3i_[i * 3 + j];
			facenodei = node_map[facenodei] + 1;
			if (facenodei == 1)
			{
				std::cout << "error" << std::endl;
			}
			output << facenodei << " ";
		}
		output << std::endl;
	}
	output.close();
}

void TetVolumetricMesh::saveSubMeshAsscii(const char* filenamebase, std::vector<int> element_list, std::vector<bool> subnodemark, std::vector<int> submap) 
{
	std::cout << "save sub mesh ..." << std::endl;
	std::cout << "num elements: " << element_list.size() << std::endl;

	std::ostringstream data;
	data << filenamebase << ".ele";
	std::string elefilename = data.str();

	std::ostringstream data2;
	data2 << filenamebase << ".node";
	std::string nodefilename = data2.str();

	std::ofstream eleoutput(elefilename);
	eleoutput << element_list.size() << " " << 4 << " " << 0 << std::endl;
	for (int i = 0; i < element_list.size(); i++)
	{
		eleoutput << i << " ";
		
		int eleid = element_list[i];
		LoboElement* ele = this->getElement(eleid);
		for (int j = 0; j < ele->node_indices.size(); j++)
		{
			int nodeid = ele->node_indices[j];
			int subnodeid = submap[nodeid];
			eleoutput << subnodeid << " ";
		}
		eleoutput << std::endl;
	}
	
	eleoutput.close();


	int subnodesize = 0;
	for (int i = 0; i < subnodemark.size(); i++)
	{
		if (subnodemark[i])
		{
			subnodesize++;
		}
	}

	std::ofstream nodeoutput(nodefilename);
	nodeoutput << subnodesize << " " << 3 << " " << 0 << " " << 0 << std::endl;
	int nodeindex = 0;
	for (int i = 0; i < subnodemark.size(); i++)
	{
		if (subnodemark[i])
		{
			nodeoutput << nodeindex << " ";
			Vector3d position = this->getNodeRestPosition(i);
			nodeoutput << position.transpose() << std::endl;
			nodeindex++;
		}
	}
	nodeoutput.close();
}

void TetVolumetricMesh::cutAndExportByInsideTetMesh(TetVolumetricMesh* inputMesh, const char* filename)
{
	std::vector<bool> subnodemark(numVertices);
	std::fill(subnodemark.begin(), subnodemark.end(), false);

	for (int i = 0; i < numVertices; i++)
	{
		Vector3d nodeposition = this->getNodeRestPosition(i);
		int eleid = inputMesh->getContainedElement(nodeposition);
		if (eleid == -1)
		{
			subnodemark[i] = true;
		}
	}

	std::vector<int> subelement_list;

	for (int i = 0; i < numElements; i++)
	{
		LoboElement* ele = this->getElement(i);
		bool inSubMesh = false;
		for (int j = 0; j < ele->node_indices.size(); j++)
		{
			int nodeid = ele->node_indices[j];
			if (subnodemark[nodeid])
			{
				inSubMesh = true;
			}
		}
		if (inSubMesh)
		subelement_list.push_back(i);
	}

	//
	std::vector<bool> submeshmark(numVertices);
	std::fill(submeshmark.begin(), submeshmark.end(), false);
	std::vector<int> submap(numVertices);
	std::fill(submap.begin(), submap.end(), -1);


	for (int i = 0; i < subelement_list.size(); i++)
	{
		int eleid = subelement_list[i];
		LoboElement* ele = this->getElement(eleid);
		for (int j = 0; j < ele->node_indices.size(); j++)
		{
			int nodeid = ele->node_indices[j];
			submeshmark[nodeid] = true;
		}
	}

	int count = 0;
	for (int i = 0; i < submeshmark.size(); i++)
	{
		if (submeshmark[i])
		{
			submap[i] = count;
			count++;
		}
	}


	saveSubMeshAsscii(filename, subelement_list, submeshmark, submap);
}

void TetVolumetricMesh::computeSurface()
{
	std::vector<bool> node_mark(this->node_list.size());
	std::fill(node_mark.begin(), node_mark.end(), false);

	std::vector<bool> ele_mark(this->element_list.size());
	std::fill(ele_mark.begin(), ele_mark.end(), false);


	surface_face_3i_.clear();
	for (int i = 0; i < element_list.size(); i++)
	{
		TetElement* element =downCastTetelement(element_list[i]);

		for (int f = 0; f < 4; f++)
		{
			if (element->neighbors[f] == -1)
			{
				//this is a surface element
				int n1, n2, n3;
				n1 = element->node_indices[element->face_indices[f * 3 + 0]];
				n2 = element->node_indices[element->face_indices[f * 3 + 1]];
				n3 = element->node_indices[element->face_indices[f * 3 + 2]];

				node_mark[n1] = true;
				node_mark[n2] = true;
				node_mark[n3] = true;

				for (int j = 0; j < 4; j++)
				{
					int nid = element->node_indices[j];
					node_mark[nid] = true;
				}


				ele_mark[i] = true;
				//surface_face_3i_.push_back(n1);
				//surface_face_3i_.push_back(n2);
				//surface_face_3i_.push_back(n3);
			}
		}
	}

	surface_node_.clear();
	for (int i = 0; i < node_mark.size(); i++)
	{
		if (node_mark[i])
		{
			surface_node_.push_back(i);
		}
	}

	issurface_node_ = node_mark;

	surface_ele.clear();
	for (int i = 0; i < ele_mark.size(); i++)
	{
		if (ele_mark[i])
		{
			surface_ele.push_back(i);
		}
	}


	//Tell other this surface already computed.
	setSurfaceReady(true);
}

void TetVolumetricMesh::releaseSurface()
{
	surface_node_.clear();
	surface_face_3i_.clear();
	setSurfaceReady(false);
}

bool TetVolumetricMesh::containsVertex(int element, const Vector3d &pos) const
{
	Vector4d weight;
	computeBarycentricWeights(element, pos, weight);
	return ((weight.data()[0] >= -1e-2) && (weight.data()[1] >= -1e-2) && (weight.data()[2] >= -1e-2) && (weight.data()[3] >= -1e-2));
}

TetElement* TetVolumetricMesh::getTetElement(int index)
{
	return downCastTetelement(element_list[index]);
}

void TetVolumetricMesh::computeDeformationGradient(int elementid, Matrix3d &output)
{
	output.setIdentity();
	MatrixXd UMatrix(3, 4);
	for (int i = 0; i < 4; i++)
	{
		int nodeid = this->getElementNode(elementid, i);
		Vector3d position = this->getNodeDisplacement(nodeid);
		for (int x = 0; x < 3; x++)
		{
			UMatrix.data()[i * 3 + x] = position.data()[x];
		}
	}
	output += UMatrix*this->getTetElement(elementid)->Phi_derivate;
}

void TetVolumetricMesh::computeDeformationOperator(SparseMatrix<double>* G)
{
	G->resize(this->numElements * 9, this->numVertices * 3);
	std::vector<EIGEN_TRI> entry;

	for (int i = 0; i < numElements; i++)
	{
	 	TetElement* ele = this->getTetElement(i);

		MatrixXd G_i(9, 12);
		G_i.setZero();

		for (int j = 0; j < 3; j++)
		{
			int Dmrow = j;
			for (int k = 0; k < 3; k++)
			{
				G_i.data()[(k + 0) * 9 + j*3+k] += ele->Dm_inverse.data()[Dmrow * 3 + 0];
				G_i.data()[(9 + k) * 9 + j*3+k] -= ele->Dm_inverse.data()[Dmrow * 3 + 0];

				G_i.data()[(k + 3) * 9 + j*3+k] += ele->Dm_inverse.data()[Dmrow * 3 + 1];
				G_i.data()[(9 + k) * 9 + j*3+k] -= ele->Dm_inverse.data()[Dmrow * 3 + 1];

				G_i.data()[(k + 6) * 9 + j*3+k] += ele->Dm_inverse.data()[Dmrow * 3 + 2];
				G_i.data()[(9 + k) * 9 + j*3+k] -= ele->Dm_inverse.data()[Dmrow * 3 + 2];
			}
		}

		for (int j = 0; j < 9; j++)
		{
			for (int k = 0; k < 12; k++)
			{
				int nodeid = ele->node_indices[k / 3];
				int col = nodeid * 3 + k % 3;
				int row = i * 9 + j;

				entry.push_back(EIGEN_TRI(row, col, G_i.data()[k * 9 + j]));
			}
		}
	}
	G->setFromTriplets(entry.begin(), entry.end());
}

void TetVolumetricMesh::computeElementVolumeMatrix(SparseMatrix<double>* V)
{
	V->resize(numElements * 9, numElements * 9);
	std::vector<EIGEN_TRI> entry;
	double val = 0;
	for (int i = 0; i < numElements; i++)
	{
		val = computeCurElementVolume(this->getTetElement(i));

		for (int j = 0; j < 9; j++)
		{
			entry.push_back( EIGEN_TRI(i * 9 + j, i * 9 + j, val));
		}
	}
	V->setFromTriplets(entry.begin(), entry.end());
}

void TetVolumetricMesh::computeNodeTransformation(VectorXd &linearDisplacement, VectorXd &nonlinearDisplacement, std::vector<Matrix3d> &ourT)
{
	ourT.clear();
	for (int i = 0; i < this->getNumVertices(); i++)
	{
		LoboNodeBase* node = this->getNodeRef(i);
		Vector3d coordinate_zero = this->getNodeRestPosition(i);
		Matrix3d nodeT;
		nodeT.setZero();
		for (int j = 0; j < node->element_list.size(); j++)
		{
			int eleid = node->element_list[j];

			std::vector<int> orthernoid;
			TetElement* element = this->getTetElement(eleid);
			for (int k = 0; k < element->node_indices.size(); k++)
			{
				if (element->node_indices[k] != i)
				{
					orthernoid.push_back(element->node_indices[k]);
				}
			}
			assert(orthernoid.size() == 3);
			Matrix3d Dm_element;
			Matrix3d Ds_element;
			Matrix3d T_element;
			T_element.setZero();
			for (int l = 0; l < 3; l++)
			{
				for (int k = l + 1; k < 3; k++)
				{
					//curid
					std::vector<int> includedNode;
					includedNode.push_back(i);
					includedNode.push_back(orthernoid[l]);
					includedNode.push_back(orthernoid[k]);

					for (int h = 0; h < 3; h++)
					{
						int nodeid = includedNode[h];
						Vector3d ori_pos = this->getNodeRestPosition(nodeid);
						Vector3d new_ori_pos;
						Vector3d new_cur_pose;

						for (int p = 0; p < 3; p++)
						{
							new_ori_pos.data()[p] = linearDisplacement.data()[nodeid * 3 + p] + ori_pos.data()[p] - coordinate_zero.data()[p];
							Dm_element.data()[h * 3 + p] = new_ori_pos.data()[p];

							new_cur_pose.data()[p] = nonlinearDisplacement.data()[nodeid * 3 + p] + ori_pos.data()[p] - coordinate_zero.data()[p];
							Ds_element.data()[h * 3 + p] = new_cur_pose.data()[p];
						}
					}
					T_element.noalias() += Ds_element*Dm_element.inverse();
				}
			}
			T_element /= 3.0;
			nodeT.noalias() += T_element;
		}
		nodeT /= node->element_list.size();
		//Test
		/*Vector3d lq, nq;
		lq.data()[0] = linearDisplacement.data()[i * 3 + 0];
		lq.data()[1] = linearDisplacement.data()[i * 3 + 1];
		lq.data()[2] = linearDisplacement.data()[i * 3 + 2];

		nq.data()[0] = nonlinearDisplacement.data()[i * 3 + 0];
		nq.data()[1] = nonlinearDisplacement.data()[i * 3 + 1];
		nq.data()[2] = nonlinearDisplacement.data()[i * 3 + 2];

		std::cout << (nodeT*lq).transpose() << std::endl;
		std::cout << nq.transpose() << std::endl;
		system("pause");*/

		ourT.push_back(nodeT);
	}
}

void TetVolumetricMesh::computeCurrentFaceArea(double *elementfacearea)
{
	for (int i = 0; i < element_list.size(); i++)
	{
		for (int j = 0; j < 4; j++)
		{
			TetElement* element = this->getTetElement(i);
			int n0, n1, n2;
			n0 = element->node_indices[element->face_indices[j * 3 + 0]];
			n1 = element->node_indices[element->face_indices[j * 3 + 1]];
			n2 = element->node_indices[element->face_indices[j * 3 + 2]];
			int faceid = i * 4 + j;
			Vector3d position0, position1, position2;
			position0 = this->getNodePosition(n0);
			position1 = this->getNodePosition(n1);
			position2 = this->getNodePosition(n2);
			elementfacearea[faceid] = computeTriangleArea(position0, position1, position2);
		}
	}
}

void TetVolumetricMesh::computeCurrentFaceNorm(std::vector<Vector3d> &face_norm)
{
	assert(face_norm.size() == numElements * 4);

	for (int i = 0; i < element_list.size(); i++)
	{
		TetElement* element = this->getTetElement(i);
		for (int j = 0; j < 4; j++)
		{
			int faceid = i * 4 + j;
			int n0, n1, n2;
			n0 = element->node_indices[element->face_indices[j * 3 + 0]];
			n1 = element->node_indices[element->face_indices[j * 3 + 1]];
			n2 = element->node_indices[element->face_indices[j * 3 + 2]];

			Vector3d position0, position1, position2;
			position0 = this->getNodePosition(n0);
			position1 = this->getNodePosition(n1);
			position2 = this->getNodePosition(n2);

			computeTriangleNorm(position0, position1, position2, face_norm[faceid]);
		}
	}
}

void TetVolumetricMesh::computeCurrentNodeNorm(std::vector<Vector3d> &element_norm)
{
	assert(element_norm.size() == numElements * 4);
	double *elementfacearea = (double*)malloc(sizeof(double)*numElements*4);
	this->computeCurrentFaceArea(elementfacearea);
	std::vector<Vector3d> facenorm(numElements * 4);
	this->computeCurrentFaceNorm(facenorm);


	for (int i = 0; i < element_list.size(); i++)
	{
		TetElement* element = this->getTetElement(i);

		for (int n = 0; n < 4; n++)
		{
			int normid = i * 4 + n;
			element_norm[normid].setZero();
			for (int f = 0; f < 4; f++)
			{
				int faceid = i * 4 + f;
				for (int fn = 0; fn < 3; fn++)
				{
					if (element->face_indices[f * 3 + fn] == n)
					{
						element_norm[normid] += elementfacearea[faceid] * facenorm[faceid];
					}
				}
			}
			element_norm[normid] /= 3.0;
		}
	}
}

void TetVolumetricMesh::initMeshABCD()
{
	for (int i = 0; i < element_list.size(); i++)
	{
		initElementABCD(downCastTetelement(element_list[i]));
	}
}

void TetVolumetricMesh::initElementShapeFunctionDerivate()
{
	for (int i = 0; i < element_list.size(); i++)
	{
		this->computeElementShapeFunctionDerivate(downCastTetelement(element_list[i]));
	}
}

void TetVolumetricMesh::createSubTetMesh(TetVolumetricMesh* tetmesh, std::vector<int> elelist, std::vector<int> &nodeindex, std::vector<int> &nodemap)
{
	if (tetmesh == NULL)
	{
		std::cout << "sub tet mesh need create object first" << std::endl;
		return;
	}

	std::vector<bool> nodemark(numVertices);
	nodemap.resize(numVertices);

	std::fill(nodemark.begin(), nodemark.end(), false);
	std::fill(nodemap.begin(), nodemap.end(), -1);

	for (int i = 0; i < elelist.size(); i++)
	{
		int eleid = elelist[i];
		LoboElement* ele = this->getElement(eleid);
		for (int j = 0; j < ele->node_indices.size(); j++)
		{
			int nodeid = ele->node_indices[j];
			nodemark[nodeid] = true;
		}
	}

	nodeindex.clear();
	for (int i = 0; i < numVertices; i++)
	{
		if (nodemark[i])
		{
			nodeindex.push_back(i);
			nodemap[i] = nodeindex.size() - 1;
		}
	}

	//tetmesh = new TetVolumetricMesh();
	tetmesh->readNodeList(this, nodeindex);
	tetmesh->readElements(this, elelist, nodemap);
	tetmesh->readElementList();

	tetmesh->initVolumetricMesh();
	tetmesh->searchNeighborElement();
	tetmesh->computeSurface();

	tetmesh->assignDefualtSet();
	tetmesh->assignDefaultMaterial();

	tetmesh->volumetricmeshDisplacment.resize(tetmesh->getNumVertices()*3);
	tetmesh->volumetricmeshDisplacment.setZero();

}

void TetVolumetricMesh::computeDivergenceMatrix(MatrixXd &DIV)
{
	DIV.resize(numVertices,numVertices*3);
	DIV.setZero();

	//compute 4X12 matrix for per element
	//an0 an1 an2 an3 is the norm of each node
	for (int i = 0; i < numElements; i++)
	{
		TetElement* ele = downCastTetelement(element_list[i]);
		for (int j = 0; j < 4; j++)
		{
			Vector3d anj = -ele->nodes_norm_[j];
			int colnodeid = ele->node_indices[j];

			for (int k = 0; k < 4; k++)
			{
				int rownodeid = ele->node_indices[k];
				for (int x = 0; x < 3; x++)
				{
					int col = colnodeid * 3 + x;
					DIV.data()[col*DIV.rows() + rownodeid] += anj.data()[x];
				}
			}
		}
	}

	DIV *= -1.0 / 12.0;
}

void TetVolumetricMesh::computeDivergenceMatrixCurrent(MatrixXd &DIV)
{
	DIV.resize(numVertices, numVertices * 3);
	DIV.setZero();
	std::vector<Vector3d> element_node_norm(numElements*4);
	this->computeCurrentNodeNorm(element_node_norm);

	for (int i = 0; i < numElements; i++)
	{
		TetElement* ele = downCastTetelement(element_list[i]);
		for (int j = 0; j < 4; j++)
		{
			int normid = i * 4 + j;
			Vector3d anj = -element_node_norm[normid];
			int colnodeid = ele->node_indices[j];

			for (int k = 0; k < 4; k++)
			{
				int rownodeid = ele->node_indices[k];
				for (int x = 0; x < 3; x++)
				{
					int col = colnodeid * 3 + x;
					DIV.data()[col*DIV.rows() + rownodeid] += anj.data()[x];
				}
			}
		}
	}
	DIV *= -1.0 / 12.0;
}

void TetVolumetricMesh::computeElementShapeFunctionDerivate(TetElement* te)
{
	Vector4d phi;
	Matrix4d referenceShape;
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			referenceShape.data()[i * 4 + j] = node_list[te->node_indices[i]]->ori_position[j];
		}
		referenceShape.data()[i * 4 + 3] = 1;
	}

	//need test
	Matrix4d inverseShapefunction = referenceShape.inverse();
	te->shape_function_inv = inverseShapefunction;

	te->Phi_derivate.resize(4, 3);
	for (int i = 0; i < 4; i++)
	{
		te->Phi_g[i].data()[0] = inverseShapefunction.data()[0 * 4 + i];
		te->Phi_g[i].data()[1] = inverseShapefunction.data()[1 * 4 + i];
		te->Phi_g[i].data()[2] = inverseShapefunction.data()[2 * 4 + i];

		te->Phi_derivate.data()[0 * 4 + i] = te->Phi_g[i].data()[0];
		te->Phi_derivate.data()[1 * 4 + i] = te->Phi_g[i].data()[1];
		te->Phi_derivate.data()[2 * 4 + i] = te->Phi_g[i].data()[2];
	}

	//init Dm Dm_inverse
	
	// FEM simulation of 3D deformable solids: a practitioner's guide to theory, discretization and model reduction

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			te->Dm.data()[j*3+i] = node_list[te->node_indices[j]]->ori_position[i] -
				node_list[te->node_indices[3]]->ori_position[i];
		}
	}
	te->Dm_inverse = te->Dm.inverse();

}

void TetVolumetricMesh::initElementABCD(TetElement* te)
{
	//computeElementShapeFunctionDerivate(te);

	//init ABCD
	//init A
	for (int a = 0; a < 4; a++)
	{
		for (int b = 0; b < 4; b++)
		{
			te->A[a][b] = te->Phi_g[a] * te->Phi_g[b].transpose()*te->volume_;
		}
	}

	//init B
	for (int a = 0; a < 4; a++)
	{
		for (int b = 0; b < 4; b++)
		{
			te->B[a][b] = te->Phi_g[a].dot(te->Phi_g[b])*te->volume_;
		}
	}

	//init C
	for (int a = 0; a < 4; a++)
	{
		for (int b = 0; b < 4; b++)
		{
			for (int c = 0; c < 4; c++)
			{
				te->C[a][b][c] = te->Phi_g[a] * (te->Phi_g[b].dot(te->Phi_g[c]))*te->volume_;
			}
		}
	}

	//init D
	for (int a = 0; a < 4; a++)
	{
		for (int b = 0; b < 4; b++)
		{
			for (int c = 0; c < 4; c++)
			{
				for (int d = 0; d < 4; d++)
				{
					te->D[a][b][c][d] = te->Phi_g[a].dot(te->Phi_g[b]) * te->Phi_g[c].dot(te->Phi_g[d])*te->volume_;
				}
			}
		}
	}

}

double TetVolumetricMesh::computeOriElementVolume(TetElement* te)
{
	Vector3d a = this->getNodeRestPosition(te->node_indices[0]);
	Vector3d b = this->getNodeRestPosition(te->node_indices[1]);
	Vector3d c = this->getNodeRestPosition(te->node_indices[2]);
	Vector3d d = this->getNodeRestPosition(te->node_indices[3]);
	double volume = computeTetVolumeABS(a, b, c, d);
	return volume;
}

double TetVolumetricMesh::computeCurElementVolume(TetElement* te)
{
	Vector3d a = this->getNodePosition(te->node_indices[0]);
	Vector3d b = this->getNodePosition(te->node_indices[1]);
	Vector3d c = this->getNodePosition(te->node_indices[2]);
	Vector3d d = this->getNodePosition(te->node_indices[3]);
	double volume = computeTetVolumeABS(a, b, c, d);
	return volume;
}

void TetVolumetricMesh::computeNodeVolume(double *nodevolume)
{
	memset(nodevolume, 0, numVertices*sizeof(double));

	for (int i = 0; i < numElements; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			int nodeid = this->getElementNode(i, j);
			nodevolume[nodeid] += this->getTetElement(i)->volume_ / 4.0;
		}
	}
}

void TetVolumetricMesh::computeCurrentNodeVolume(double *nodevolume)
{
	std::vector<double> elementvolume(numElements);
	memset(nodevolume, 0, numVertices*sizeof(double));

	for (int i = 0; i < numElements; i++)
	{
		elementvolume[i] = computeCurElementVolume(this->getTetElement(i));
	}

	for (int i = 0; i < numElements; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			int nodeid = this->getElementNode(i, j);
			nodevolume[nodeid] += elementvolume[i] / 4.0;
		}
	}

	//for (int i = 0; i < numVertices; i++)
	//{
	//	int elementsize = node_list[i]->element_list.size();
	//	for (int j = 0; j < elementsize; j++)
	//	{
	//		nodevolume[i] += elementvolume[node_list[i]->element_list[j]] / elementsize;
	//	}
	//}
}

double TetVolumetricMesh::computeCurrentMeshVolume()
{
	double val = 0;
	for (int i = 0; i < numElements; i++)
	{
		val+= computeCurElementVolume(this->getTetElement(i));
	}
	return val;
}



Vector3d TetVolumetricMesh::computeMassCenter()
{
	Vector3d massCenter = Vector3d::Zero();
	double totalMass;
	totalMass = 0;
	for (int i = 0; i < numElements; i++)
	{
		TetElement* ele = this->getTetElement(i);

		double volume = computeOriElementVolume(ele);
		double density = this->getElementMaterial(i)->getDensity();
		double mass = density*volume;
		
		Vector3d elecenter = Vector3d::Zero();
		for (int j = 0; j < ele->node_indices.size(); j++)
		{
			int nodeid = ele->node_indices[j];
			Vector3d nodeposition = this->getNodePosition(nodeid);
			elecenter += nodeposition;
		}
		elecenter /= ele->node_indices.size();
		massCenter += elecenter*mass;
		totalMass += mass;
	}
	massCenter /= totalMass;
	return massCenter;
}

void TetVolumetricMesh::computeMassCenterDrive(MatrixXd& jacobiMatrix)
{
	int dofs = this->getNumVertices() * 3;
	jacobiMatrix.resize(3, dofs);
	jacobiMatrix.setZero();

	double totalMass;
	totalMass = 0;
	for (int i = 0; i < numElements; i++)
	{
		TetElement* ele = this->getTetElement(i);

		double volume = computeOriElementVolume(ele);
		double density = this->getElementMaterial(i)->getDensity();
		double mass = density*volume;
		totalMass += mass;

		for (int j = 0; j < ele->node_indices.size(); j++)
		{
			int nodeid = ele->node_indices[j];
			//Vector3d nodeposition = this->getNodeRestPosition(nodeid);
			////nodeposition.data()
			
			int col = nodeid * 3;
			jacobiMatrix.data()[col * 3 + 0] += mass / ele->node_indices.size();
			jacobiMatrix.data()[(col + 1) * 3 + 1] += mass / ele->node_indices.size();
			jacobiMatrix.data()[(col + 2) * 3 + 2] += mass / ele->node_indices.size();
			//elecenter += nodeposition;
		}
	}
	jacobiMatrix /= totalMass;
}

TetVolumetricMesh* downCastTetVolMesh(LoboVolumetricMesh* mesh)
{
	if (mesh->getMeshType() == LoboVolumetricMesh::VolumetricmeshType::TET)
	{
		return (TetVolumetricMesh*)mesh;
	}
	else
	{
		return NULL;
	}
}

