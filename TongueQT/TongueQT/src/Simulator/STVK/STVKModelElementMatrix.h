#pragma once
#include <Eigen/Dense>
#include <Eigen/Sparse>
using namespace Eigen;

class STVKModel;
class LoboVolumetricMesh;

class STVKModelElementMatrix
{
public:
	STVKModelElementMatrix(STVKModel* stvkmodel, LoboVolumetricMesh* volumetricmesh, double *u = NULL);
	~STVKModelElementMatrix();

	virtual void computeSubMatrix(std::vector<int> elementindices, SparseMatrix<double>* sparseMatirx);
	virtual void getStiffnessMatrixTopo(SparseMatrix<double>* matrix);


protected:

	virtual void storeMatrix();

	STVKModel* stvkmodel;

	int numElements;
	int r;
	SparseMatrix<double>* stiffnessMatirxTopo;
	LoboVolumetricMesh* volumetricmesh;
	std::vector<MatrixXd*> elementMatrix;

};


