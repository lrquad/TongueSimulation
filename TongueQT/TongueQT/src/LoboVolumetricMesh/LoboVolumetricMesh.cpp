#include "LoboVolumetricMesh.h"
#include <fstream>
#include <iostream>
#include "LoboNodeBase.h"
#include "LoboElement.h"
#include "Functions/deleteSTDvector.h"
#include "ENuMaterial.h"
#include <random>

LoboVolumetricMesh::LoboVolumetricMesh()
{
	mesh_name = "defualt_mesh_name";
	surfaceready = false;
	uniformMeshAfterLoad = false;
	unifromUseScaleEx = false;
	transformBaseUniform.setIdentity();
	translateBaseUniform.setZero();
}

LoboVolumetricMesh::LoboVolumetricMesh(const LoboVolumetricMesh* volumetricmesh)
{

}

LoboVolumetricMesh::~LoboVolumetricMesh()
{
	deleteStdvectorPointer(node_list);
	deleteStdvectorPointer(element_list);
	deleteStdvectorPointer(sets);
	deleteStdvectorPointer(regions);
	deleteStdvectorPointer(materials);
}

void LoboVolumetricMesh::readNodeList(const char* nodefilename, Vector3d translate, double scale /*= 1.0*/)
{
	std::cout << "scaled mesh: " << scale << std::endl;
	//read node list
	std::ifstream instream;

	instream.open(nodefilename);
	int numberofnodes; int dimension; int temp;
	instream >> numberofnodes >> dimension >> temp >> temp;
	node_list.resize(numberofnodes);
	//std::cout << "nodes size: " << numberofnodes << std::endl;
	this->numVertices = numberofnodes;
	for (int i = 0; i < numVertices; i++)
	{
		node_list[i] = new LoboNodeBase();
		instream >> node_list[i]->index_;
		node_list[i]->index_ = i;
		for (int j = 0; j < dimension; j++)
		{
			instream >> node_list[i]->ori_position.data()[j];
		}
		
		if (!getUniformMeshAfterLoad())
		{
			node_list[i]->ori_position += translate;
			node_list[i]->ori_position *= scale;
		}

	}
	issurface_node_.resize(numVertices);
	std::fill(issurface_node_.begin(), issurface_node_.end(), false);

	instream.close();


	if (getUniformMeshAfterLoad())
	{
		uniformMesh(scale,getUnifromUseScaleEx());
	}


}

void LoboVolumetricMesh::readNodeList(LoboVolumetricMesh* input, std::vector<int> nodeindex)
{
	int numberofnodes = nodeindex.size();
	node_list.resize(numberofnodes);
	this->numVertices = numberofnodes;
	for (int i = 0; i < numVertices; i++)
	{
		node_list[i] = new LoboNodeBase();
		node_list[i]->index_ = i;
		int nodeid = nodeindex[i];
		node_list[i]->ori_position = input->getNodeRestPosition(nodeid);
	}

	issurface_node_.resize(numVertices);
	std::fill(issurface_node_.begin(), issurface_node_.end(), false);

	if (getUniformMeshAfterLoad())
	{
		uniformMesh();
	}
}

void LoboVolumetricMesh::readNodeList(std::ifstream &instream)
{
	int numberofnodes; int dimension; int temp;
	numberofnodes;
	dimension = 3;
	temp = 0;

	instream.read((char*)&numberofnodes, sizeof(int));
	instream.read((char*)&dimension, sizeof(int));
	instream.read((char*)&temp, sizeof(int));
	instream.read((char*)&temp, sizeof(int));
	this->numVertices = numberofnodes;
	node_list.resize(numberofnodes);
	for (int i = 0; i < numVertices; i++)
	{
		node_list[i] = new LoboNodeBase();
		instream.read((char*)&(node_list[i]->index_), sizeof(int));
		instream.read((char*)node_list[i]->ori_position.data(), sizeof(Vector3d::Scalar) * 3);
	}

	issurface_node_.resize(numVertices);
	std::fill(issurface_node_.begin(), issurface_node_.end(), false);

	if (getUniformMeshAfterLoad())
	{
		uniformMesh();
	}
}

void LoboVolumetricMesh::transformMesh(Matrix3d Transform, Vector3d translate)
{
	for (int i = 0; i < node_list.size(); i++)
	{
		node_list[i]->ori_position = transformBaseUniform.transpose()*(node_list[i]->ori_position - translateBaseUniform);
		node_list[i]->ori_position = Transform*node_list[i]->ori_position + translate;
	}

	transformBaseUniform = Transform;
	translateBaseUniform = translate;
}

void LoboVolumetricMesh::readElements(LoboVolumetricMesh* input, std::vector<int> eleindex, std::vector<int> node_map)
{
	int numberofelement = eleindex.size();
	this->numElements = numberofelement;
	this->numer_element_vertcies = input->numer_element_vertcies;

	int elementbuffersize = numberofelement*numer_element_vertcies;

	elements.resize(elementbuffersize);

	for (int i = 0; i < numberofelement; i++)
	{
		int eleid = eleindex[i];
		for (int j = 0; j < numer_element_vertcies; j++)
		{
			int nodeid = input->elements[eleid*numer_element_vertcies + j];
			int mapednodeid = node_map[nodeid];
			elements[i*numer_element_vertcies + j] = mapednodeid;
		}
	}
}

void LoboVolumetricMesh::readElementsIndex(const char* elefilename)
{
	//store first
	std::ifstream instream;
	instream.open(elefilename);
	int numberofelement;
	int nodes_per_element;
	int neighbors;

	instream >> numberofelement >> nodes_per_element >> neighbors;
	//std::cout << "element size: " << numberofelement << std::endl;
	
	this->numElements = numberofelement;
	this->numer_element_vertcies = nodes_per_element;

	int elementssize = numberofelement*nodes_per_element;
	elements.resize(elementssize);
	elementsNeighbor.resize(numberofelement*neighbors);

	for (int i = 0; i < numberofelement; i++)
	{
		int index;
		instream >> index;
		for (int j = 0; j < nodes_per_element;j++)
			instream >> elements[i*nodes_per_element+j];
		for (int j = 0; j < neighbors; j++)
		{
			instream >> elementsNeighbor[i*neighbors + j];
		}
	}
	instream.close();
	readElementList();
}

void LoboVolumetricMesh::readElementsIndex(std::ifstream &instream)
{
	int numberofelement;
	int nodes_per_element;
	int neighbors;

	instream.read((char*)&numberofelement, sizeof(int));
	instream.read((char*)&nodes_per_element, sizeof(int));
	instream.read((char*)&neighbors, sizeof(int));

	this->numElements = numberofelement;
	this->numer_element_vertcies = nodes_per_element;

	int elementssize = numberofelement*nodes_per_element;
	elements.resize(elementssize);
	instream.read((char*)elements.data(), sizeof(int)*elementssize);
	readElementList();
}

void LoboVolumetricMesh::readElementMesh(const char* filenamebase, double* translate_,double scale)
{
	if (translate_ != NULL)
	std::cout << "volume mesh translate" << translate_[0] << " " << translate_[1] << " " << translate_[2] << std::endl;

	std::ostringstream data;
	data << filenamebase << ".ele";
	std::string elefilename = data.str();

	std::ostringstream data2;
	data2 << filenamebase << ".node";
	std::string nodefilename = data2.str();

	Vector3d translate = Vector3d::Zero();
	if (translate_!=NULL)
	memcpy(translate.data(), translate_, translate.size()*sizeof(double));

	this->readNodeList(nodefilename.c_str(), translate, scale);
	this->readElementsIndex(elefilename.c_str());

	assignDefualtSet();
	assignDefaultMaterial();

	volumetricmeshDisplacment.resize(numVertices * 3);
	volumetricmeshDisplacment.setZero();
}

void LoboVolumetricMesh::readElementMesh(std::ifstream &instream)
{
	readNodeList(instream);
	readElementsIndex(instream);

	assignDefualtSet();
	assignDefaultMaterial();

	volumetricmeshDisplacment.resize(numVertices * 3);
	volumetricmeshDisplacment.setZero();
}

void LoboVolumetricMesh::saveElementMesh(std::ofstream &outstream)
{
	saveNodeList(outstream);
	saveElementsIndex(outstream);
}

void LoboVolumetricMesh::saveNodeList(std::ofstream &outstream)
{
	int numberofnodes; int dimension; int temp;
	numberofnodes = node_list.size();
	dimension = 3;
	temp = 0;

	outstream.write((char*)&numberofnodes, sizeof(int));
	outstream.write((char*)&dimension, sizeof(int));
	outstream.write((char*)&temp, sizeof(int));
	outstream.write((char*)&temp, sizeof(int));

	for (int i = 0; i < numVertices; i++)
	{
		outstream.write((char*)&(node_list[i]->index_), sizeof(int));
		outstream.write((char*)node_list[i]->ori_position.data(), sizeof(Vector3d::Scalar) * 3);
	}
}

void LoboVolumetricMesh::saveElementsIndex(std::ofstream &outstream)
{
	int numberofelement;
	int nodes_per_element;
	int neighbors;

	numberofelement = this->numElements;
	nodes_per_element = this->numer_element_vertcies;
	neighbors = 0;

	outstream.write((char*)&numberofelement, sizeof(int));
	outstream.write((char*)&nodes_per_element, sizeof(int));
	outstream.write((char*)&neighbors, sizeof(int));

	int elementsize = elements.size();
	outstream.write((char*)elements.data(), sizeof(int)*elementsize);
}

void LoboVolumetricMesh::assignDefualtSet()
{
	sets.clear();
	sets.push_back(new Set("defaultSet"));
	for (int i = 0; i < numElements;i++)
	sets[0]->insert(i);
}



void LoboVolumetricMesh::scaleVolumetricMesh(double scale)
{
	for (int i = 0; i < numVertices; i++)
	{
		node_list[i]->ori_position *= scale;
	}
}

void LoboVolumetricMesh::computeNodeRotationRing(int nodeid, Matrix3d& rotation)
{
	LoboNodeBase* node_p = node_list[nodeid];
	Vector3d ori_p = node_p->ori_position;
	Vector3d cur_p = this->getNodePosition(nodeid);
	Matrix3d H = Matrix3d::Zero();
	for (int i = 0; i < node_p->neighbor.size(); i++)
	{
		int nodeindex = node_p->neighbor[i];
		Vector3d a = node_list[nodeindex]->ori_position - ori_p;
		Vector3d b = this->getNodePosition(nodeindex) - cur_p;
		H += a*b.transpose();
	}
	JacobiSVD<Matrix3d> svd(H, ComputeFullU | ComputeFullV);
	rotation = svd.matrixV()*svd.matrixU().transpose();
}

void LoboVolumetricMesh::setOriPosition(double *q)
{
	for (int i = 0; i < numVertices; i++)
	{
		node_list[i]->ori_position.data()[0] += q[i * 3 + 0];
		node_list[i]->ori_position.data()[1] += q[i * 3 + 1];
		node_list[i]->ori_position.data()[2] += q[i * 3 + 2];
	}
}

void LoboVolumetricMesh::creatMaterialForBar(double maxYoung, double minYoung)
{
	double maxz = 7;
	double minz = -7;

	double mid1 = 1;
	double mid2 = -1;

	double tangent = (maxYoung - minYoung) / (mid2-mid1);

	element_material.resize(numElements);
	std::fill(element_material.begin(), element_material.end(), 0);

	for (int i = 0; i < numElements; i++)
	{
		LoboElement* ele = this->getElement(i);
		Vector3d eleposition = ele->getOriCenterPosition();
		double eleYoungs = 1000;
		if (eleposition.z() <= maxz&&eleposition.z() > mid1)
		{
			eleYoungs = maxYoung;
		}
		else if (eleposition.z()<=mid1&&eleposition.z()>=mid2)
		{
			eleYoungs = (mid2 - eleposition.z())*tangent + minYoung;
		}
		else
		{
			eleYoungs = minYoung;
		}

		materials.push_back(new ENuMaterial("", 1.0, eleYoungs, 0.40));
		num_Materials++;
		element_material[i] = materials.size() - 1;
	}
}

void LoboVolumetricMesh::createMaterialForLollipop(double maxYoung, double minYoung)
{
	double miny = -1;
	double mid = 1.5;
	double tangent = (maxYoung - minYoung) / (mid - miny);
	element_material.resize(numElements);
	std::fill(element_material.begin(), element_material.end(), 0);

	for (int i = 0; i < numElements; i++)
	{
		LoboElement* ele = this->getElement(i);
		Vector3d eleposition = ele->getOriCenterPosition();
		double eleYoungs = 3000;

		if (eleposition.y() <= mid)
		{
			eleYoungs = maxYoung - (eleposition.y() - miny)*tangent;
		}
		else
		{
			eleYoungs = minYoung;
		}

		materials.push_back(new ENuMaterial("", 1.0, eleYoungs, 0.40));
		num_Materials++;
		element_material[i] = materials.size() - 1;
	}




}

void LoboVolumetricMesh::assignDefaultMaterial()
{
	materials.clear();
	materials.push_back(new ENuMaterial("", 1.0, 1000, 0.40));
	num_Materials = 1;

	element_material.clear();
	for (int i = 0; i < numElements; i++)
	{
		element_material.push_back(0);
	}

	this->node_material.clear();
	
	for (int i = 0; i < numVertices; i++)
	{
		this->node_material.push_back(0);
	}
}

void LoboVolumetricMesh::computeLaplacianMatrix(SparseMatrix<double>* L)
{
	assert(L->rows() == this->numVertices);
	std::vector<Triplet<double>> sparseL;

	for (int i = 0; i < numVertices; i++)
	{
		int numberofneighbor = 0;
		double edgelengthtotal = 0;

		sparseL.push_back(Triplet<double>(i, i, 1));

		for (int j = 0; j < node_list[i]->neighbor.size(); j++)
		{
			int neighborid = node_list[i]->neighbor[j];
			double edge = (node_list[neighborid]->ori_position - node_list[i]->ori_position).norm();
			edgelengthtotal += edge;
		}

		for (int j = 0; j < node_list[i]->neighbor.size(); j++)
		{
			int neighborid = node_list[i]->neighbor[j];
			double edge = (node_list[neighborid]->ori_position - node_list[i]->ori_position).norm();
			double wij = (edge) / edgelengthtotal;
			sparseL.push_back(Triplet<double>(i, neighborid, -wij));
		}
	}

	L->setFromTriplets(sparseL.begin(), sparseL.end());

}

void LoboVolumetricMesh::computeRigidModes(MatrixXd &phi)
{
	int r = getNumVertices() * 3;
	phi.resize(r, 3);
	int numVertex = getNumVertices();

	phi.setZero();

	Vector3d unit_x;
	Vector3d unit_y;
	Vector3d unit_z;

	unit_x(0) = 1; unit_x(1) = 0; unit_x(2) = 0;
	unit_y(0) = 0; unit_y(1) = 1; unit_y(2) = 0;
	unit_z(0) = 0; unit_z(1) = 0; unit_z(2) = 1;


	Vector3d u_cross_x;
	Vector3d u_cross_y;
	Vector3d u_cross_z;

	for (int i = 0; i < numVertex; i++)
	{
		int ordered_idx = i;
		Vector3d position = this->getNodeRestPosition(i);

		// the first column is the x-translation
		phi(ordered_idx * 3 + 0, 0) = 1;
		phi(ordered_idx * 3 + 1, 0) = 0;
		phi(ordered_idx * 3 + 2, 0) = 0;

		// the second column is the y-translation
		phi(ordered_idx * 3 + 0, 1) = 0;
		phi(ordered_idx * 3 + 1, 1) = 1;
		phi(ordered_idx * 3 + 2, 1) = 0;

		// the third column is the z-translation
		phi(ordered_idx * 3 + 0, 2) = 0;
		phi(ordered_idx * 3 + 1, 2) = 0;
		phi(ordered_idx * 3 + 2, 2) = 1;

		continue;
		// the fourth column is the x-infinitesimal rotation
		u_cross_x = position.cross(unit_x);
		phi(ordered_idx * 3 + 0, 3) = u_cross_x(0);
		phi(ordered_idx * 3 + 1, 3) = u_cross_x(1);
		phi(ordered_idx * 3 + 2, 3) = u_cross_x(2);

		// the fifth column is the y-infinitesimal rotation
		u_cross_y = position.cross(unit_y);
		phi(ordered_idx * 3 + 0, 4) = u_cross_y(0);
		phi(ordered_idx * 3 + 1, 4) = u_cross_y(1);
		phi(ordered_idx * 3 + 2, 4) = u_cross_y(2);

		// the sixth column is the z-infinitesimal rotation
		u_cross_z = position.cross(unit_z);
		phi(ordered_idx * 3 + 0, 5) = u_cross_z(0);
		phi(ordered_idx * 3 + 1, 5) = u_cross_z(1);
		phi(ordered_idx * 3 + 2, 5) = u_cross_z(2);
	}

}

int LoboVolumetricMesh::searchCloseNode(const Vector3d &position) const
{
	double closestDist = DBL_MAX;
	int closestVertex = -1;

	for (int i = 0; i < node_list.size(); i++)
	{
	 	double len = (position-node_list[i]->ori_position).norm();
		if (len < closestDist)
		{
			closestDist = len;
			closestVertex = i;
		}
	}
	return  closestVertex;
}

int LoboVolumetricMesh::searchCloseSurfaceNode(const Vector3d &position) const
{
	double closestDist = DBL_MAX;
	int closestVertex = -1;
	for (int i = 0; i < surface_node_.size(); i++)
	{
		double len = (position - node_list[surface_node_[i]]->ori_position).norm();
		if (len < closestDist)
		{
			closestDist = len;
			closestVertex = surface_node_[i];
		}
	}
	return closestVertex;
}

int LoboVolumetricMesh::getContainedElement(const Vector3d &position)
{
	for (int element = 0; element < numElements; element++)
	{
		if (containsVertex(element, position))
			return element;
	}
	return -1;
}

void LoboVolumetricMesh::scaleMesh(double scale)
{
	
}

void LoboVolumetricMesh::uniformMesh(double scaleEx, bool useExScale)
{
	double minX, maxX, minZ, maxZ, minY, maxY;
	Vector3d massCenter;
	
	minX = DBL_MAX;
	maxX = -DBL_MAX;
	minZ = DBL_MAX;
	maxZ = -DBL_MAX;
	minY = DBL_MAX;
	maxY = -DBL_MAX;

	massCenter.setZero();
	for (int i = 0; i < node_list.size(); i++)
	{
		Vector3d nodeposition = node_list[i]->ori_position;
		massCenter += nodeposition;

		if (nodeposition.x() < minX)
		{
			minX = nodeposition.x();
		}

		if (nodeposition.x() > maxX)
		{
			maxX = nodeposition.x();
		}


		if (nodeposition.y() < minY)
		{
			minY = nodeposition.y();
		}

		if (nodeposition.y() > maxY)
		{
			maxY = nodeposition.y();
		}


		if (nodeposition.z() < minZ)
		{
			minZ = nodeposition.z();
		}

		if (nodeposition.z() > maxZ)
		{
			maxZ = nodeposition.z();
		}

	}
	massCenter /= node_list.size();

	Vector3d scale;
	scale[0] = maxX - minX;
	scale[1] = maxY - minY;
	scale[2] = maxZ - minZ;

	double maxscale = scale.maxCoeff();
	if (useExScale)
	{
		maxscale = scaleEx;
	}

	for (int i = 0; i < node_list.size(); i++)
	{
		node_list[i]->ori_position -= massCenter;
		node_list[i]->ori_position /= maxscale;
	}
	this->uniformScale = maxscale;
	this->uniformT = massCenter;
}

double LoboVolumetricMesh::computeUniformScale()
{
	double minX, maxX, minZ, maxZ, minY, maxY;
	Vector3d massCenter;

	minX = DBL_MAX;
	maxX = -DBL_MAX;
	minZ = DBL_MAX;
	maxZ = -DBL_MAX;
	minY = DBL_MAX;
	maxY = -DBL_MAX;

	massCenter.setZero();
	for (int i = 0; i < node_list.size(); i++)
	{
		Vector3d nodeposition = node_list[i]->ori_position;
		massCenter += nodeposition;

		if (nodeposition.x() < minX)
		{
			minX = nodeposition.x();
		}

		if (nodeposition.x() > maxX)
		{
			maxX = nodeposition.x();
		}


		if (nodeposition.y() < minY)
		{
			minY = nodeposition.y();
		}

		if (nodeposition.y() > maxY)
		{
			maxY = nodeposition.y();
		}


		if (nodeposition.z() < minZ)
		{
			minZ = nodeposition.z();
		}

		if (nodeposition.z() > maxZ)
		{
			maxZ = nodeposition.z();
		}

	}
	massCenter /= node_list.size();

	Vector3d scale;
	scale[0] = maxX - minX;
	scale[1] = maxY - minY;
	scale[2] = maxZ - minZ;

	double maxscale = scale.maxCoeff();

	return maxscale;
}

double LoboVolumetricMesh::computeUniformScaleExceptConstraint(int numconstraints, int* constrainedDofs)
{
	double minX, maxX, minZ, maxZ, minY, maxY;
	Vector3d massCenter;

	minX = DBL_MAX;
	maxX = -DBL_MAX;
	minZ = DBL_MAX;
	maxZ = -DBL_MAX;
	minY = DBL_MAX;
	maxY = -DBL_MAX;

	massCenter.setZero();

	std::vector<bool> nodemark(node_list.size());
	std::fill(nodemark.begin(), nodemark.end(), false);

	for (int i = 0; i < numconstraints/3; i++)
	{
		int nodeid = constrainedDofs[i * 3] / 3;
		nodemark[nodeid] = true;
	}

	for (int i = 0; i < node_list.size(); i++)
	{
		if (nodemark[i])
		{
			continue;
		}
		Vector3d nodeposition = node_list[i]->ori_position;
		massCenter += nodeposition;

		if (nodeposition.x() < minX)
		{
			minX = nodeposition.x();
		}

		if (nodeposition.x() > maxX)
		{
			maxX = nodeposition.x();
		}


		if (nodeposition.y() < minY)
		{
			minY = nodeposition.y();
		}

		if (nodeposition.y() > maxY)
		{
			maxY = nodeposition.y();
		}


		if (nodeposition.z() < minZ)
		{
			minZ = nodeposition.z();
		}

		if (nodeposition.z() > maxZ)
		{
			maxZ = nodeposition.z();
		}

	}
	massCenter /= node_list.size();

	Vector3d scale;
	scale[0] = maxX - minX;
	scale[1] = maxY - minY;
	scale[2] = maxZ - minZ;

	double maxscale = scale.maxCoeff();

	return maxscale;
}

bool LoboVolumetricMesh::isSurfaceNode(int nodeid)
{
	return issurface_node_[nodeid];
}

Vector3d LoboVolumetricMesh::getNodePosition(int i) const
{
	return node_list[i]->ori_position + node_list[i]->displacement;
}

Vector3d LoboVolumetricMesh::getNodeRestPosition(int i) const
{
	return node_list[i]->ori_position;
}

Vector3d LoboVolumetricMesh::getNodeDisplacement(int i) const
{
	return node_list[i]->displacement;
}

void LoboVolumetricMesh::setDisplacement(double *q)
{
	for (int i = 0; i < numVertices; i++)
	{
		node_list[i]->displacement.data()[0] = q[i * 3 + 0];
		node_list[i]->displacement.data()[1] = q[i * 3 + 1];
		node_list[i]->displacement.data()[2] = q[i * 3 + 2];
	}
}

void LoboVolumetricMesh::setDisplacementBuffer(double* q)
{
	for (int i = 0; i < volumetricmeshDisplacment.size(); i++)
	{
		volumetricmeshDisplacment[i] = q[i];
	}
}

double* LoboVolumetricMesh::getDisplacementRef()
{
	return volumetricmeshDisplacment.data();
}

void LoboVolumetricMesh::resetDisplacement()
{
	for (int i = 0; i < numVertices; i++)
	{
		node_list[i]->displacement.data()[0] = 0;
		node_list[i]->displacement.data()[1] = 0;
		node_list[i]->displacement.data()[2] = 0;
	}
}

void LoboVolumetricMesh::getDisplacement(double *q)
{
	for (int i = 0; i < numVertices; i++)
	{
		q[i * 3 + 0] = node_list[i]->displacement.data()[0];
		q[i * 3 + 1] = node_list[i]->displacement.data()[1];
		q[i * 3 + 2] = node_list[i]->displacement.data()[2];
	}
}

void LoboVolumetricMesh::exportNodeDisplacementCSV(const char* filename)
{
	std::ofstream outputStream(filename);
	for (int i = 0; i < numVertices; i++)
	{
		outputStream << node_list[i]->displacement.data()[0] << ","
			<< node_list[i]->displacement.data()[1] << ","
			<< node_list[i]->displacement.data()[2];
		outputStream << std::endl;
	}
	outputStream.close();

}

void LoboVolumetricMesh::exportNodeRegionDisplacementCSV(const char* filename)
{
	std::ofstream outputStream(filename);
	for (int i = 0; i < numVertices; i++)
	{
		outputStream << node_list[i]->displacement.data()[0] << ","
			<< node_list[i]->displacement.data()[1] << ","
			<< node_list[i]->displacement.data()[2]<<",";

		int numNeighbors = node_list[i]->neighbor.size();
		std::vector<int> pickedNodeid = node_list[i]->neighbor;
		std::random_shuffle(pickedNodeid.begin(), pickedNodeid.end());

		for (int j = 0; j < 3; j++)
		{
			int neighbornodeid = pickedNodeid[j];

			outputStream << node_list[neighbornodeid]->displacement.data()[0] << ","
				<< node_list[neighbornodeid]->displacement.data()[1] << ","
				<< node_list[neighbornodeid]->displacement.data()[2];
			if (j != node_list[i]->neighbor.size() - 1)
			{
				outputStream << ",";
			}
		}
		outputStream << std::endl;
	}

	outputStream.close();

}

void LoboVolumetricMesh::exportNodeDisplacementStream(std::vector<double> &target)
{
	for (int i = 0; i < numVertices; i++)
	{

		for (int j = 0; j < 3; j++)
		{
			target.push_back(node_list[i]->displacement.data()[j]);
		}
	}
}

void LoboVolumetricMesh::exportNodeRegionDisplacementStream(std::vector<double> &origin_)
{
	for (int i = 0; i < numVertices; i++)
	{

		for (int j = 0; j < 3; j++)
		{
			origin_.push_back(node_list[i]->displacement.data()[j]);
		}
		
		int numNeighbors = node_list[i]->neighbor.size();
		std::vector<int> pickedNodeid = node_list[i]->neighbor;
		std::random_shuffle(pickedNodeid.begin(), pickedNodeid.end());

		for (int j = 0; j < 0; j++)
		{
			int neighbornodeid = pickedNodeid[j];
			origin_.push_back(node_list[neighbornodeid]->displacement.data()[0]);
			origin_.push_back(node_list[neighbornodeid]->displacement.data()[1]);
			origin_.push_back(node_list[neighbornodeid]->displacement.data()[2]);
		}
	}
}

VectorXd LoboVolumetricMesh::getNodeRegionDisplacement(int nodeid)
{
	VectorXd inputV(3);
	inputV.setZero();
	inputV.data()[0] = node_list[nodeid]->displacement.data()[0];
	inputV.data()[1] = node_list[nodeid]->displacement.data()[1];
	inputV.data()[2] = node_list[nodeid]->displacement.data()[2];
	return inputV;

	int numNeighbors = node_list[nodeid]->neighbor.size();
	std::vector<int> pickedNodeid = node_list[nodeid]->neighbor;
	std::random_shuffle(pickedNodeid.begin(), pickedNodeid.end());

	for (int j = 0; j < 3; j++)
	{
		int neighbornodeid = pickedNodeid[j];
		inputV.data()[3+j*3+0] = node_list[neighbornodeid]->displacement.data()[0];
		inputV.data()[3 + j * 3 + 1] = node_list[neighbornodeid]->displacement.data()[1];
		inputV.data()[3 + j * 3 + 2] = node_list[neighbornodeid]->displacement.data()[2];
	}
	return inputV;
}

LoboNodeBase* LoboVolumetricMesh::getNodeRef(int i) const
{
	return node_list[i];
}

int LoboVolumetricMesh::getNumElementNeighbors(int ele)
{
	return element_list[ele]->neighbors.size();
}

int LoboVolumetricMesh::getElmentNeighborid(int ele, int neighbori)
{
	return element_list[ele]->neighbors[neighbori];
}

LoboVolumetricMesh::Material* LoboVolumetricMesh::getElementMaterial(int ele) const
{
	return materials[element_material[ele]];
}

LoboVolumetricMesh::Material* LoboVolumetricMesh::getNodeMaterial(int node) const
{
	return materials[node_material[node]];
}

LoboVolumetricMesh::Material* LoboVolumetricMesh::getMaterialById(int mid)
{
	return materials[mid];
}

