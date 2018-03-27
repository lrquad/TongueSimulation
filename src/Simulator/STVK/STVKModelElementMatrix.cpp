#include "STVKModelElementMatrix.h"
#include "Simulator\STVK\STVKModel.h"
#include "LoboVolumetricMesh\TetVolumetricMesh.h"
#include "Functions\deleteSTDvector.h"

STVKModelElementMatrix::STVKModelElementMatrix(STVKModel* stvkmodel, LoboVolumetricMesh* volumetricmesh, double *u)
{
	this->stvkmodel = stvkmodel;
	r = stvkmodel->getR();
	this->volumetricmesh = volumetricmesh;
	numElements = volumetricmesh->getNumElements();

	if (u == NULL)
	{
		VectorXd displacement(r);
		displacement.setZero();
		stvkmodel->setDisplacement(displacement.data());
	}
	else
	{
		stvkmodel->setDisplacement(u);
	}

	stiffnessMatirxTopo = new SparseMatrix<double>(r,r);
	stvkmodel->getSparseMatrixTopology(*stiffnessMatirxTopo);

	storeMatrix();
}


STVKModelElementMatrix::~STVKModelElementMatrix()
{
	deleteStdvectorPointer(elementMatrix);
	delete stiffnessMatirxTopo;
}

void STVKModelElementMatrix::getStiffnessMatrixTopo(SparseMatrix<double>* matrix)
{
	stvkmodel->getSparseMatrixTopology(*matrix);
}

void STVKModelElementMatrix::storeMatrix()
{
	TetVolumetricMesh* tetmesh = downCastTetVolMesh(volumetricmesh);
	elementMatrix.resize(numElements);
	int numElementnodes = volumetricmesh->getNumElementVertices();
	
	for (int i = 0; i < numElements; i++)
	{
		elementMatrix[i] = new MatrixXd();
		stvkmodel->computeTetElementMatrix(tetmesh->getTetElement(i), elementMatrix[i]);
	}
}

void STVKModelElementMatrix::computeSubMatrix(std::vector<int> elementindices, SparseMatrix<double>* sparseMatirx)
{
	stvkmodel->computeStiffnessMatrix(elementindices, elementMatrix, sparseMatirx);
}


