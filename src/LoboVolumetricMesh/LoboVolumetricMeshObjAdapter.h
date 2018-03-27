#pragma once
#include <Eigen/Sparse>
using namespace Eigen;

class LoboVolumetricMesh;
class LoboObjMesh;

class LoboVolumetricMeshObjAdapter
{
public:
	LoboVolumetricMeshObjAdapter();
	LoboVolumetricMeshObjAdapter(LoboVolumetricMesh* volumetric_mesh, LoboObjMesh* objmesh);

	~LoboVolumetricMeshObjAdapter();

	virtual void computeNodeBarycentricWeight();
	virtual void updateObjMesh();

protected:
	
	virtual void computeTriDisplacement(double* tetdisplacement, double* tridisplacement);

	SparseMatrix<double> barycentricPhi;
	std::vector<Eigen::Triplet<double>> sparseEntry;

	MatrixXd barycentricWeightMatrix;
	std::vector<int> node_element_map;
	

	LoboVolumetricMesh* volumetric_mesh;
	LoboObjMesh* objmesh;

};

