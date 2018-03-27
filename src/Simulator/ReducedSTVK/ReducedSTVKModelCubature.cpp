#include "ReducedSTVKModelCubature.h"
#include "LoboVolumetricMesh/ENuMaterial.h"
#include "EigenMatrixIO/EigenMatrixIO.h"
#include "Simulator/STVK/STVKModel.h"
#include <fstream>

ReducedSTVKModelCubature::ReducedSTVKModelCubature(LoboVolumetricMesh* volumetricMesh_, STVKModel* stvkmodel_, SparseMatrix<double>* massMatrix_, MatrixXd * reducedModes_)
	:ReducedSTVKModel()
{
	this->stvkmodel = stvkmodel_;
	this->reducedModes = *reducedModes_;
	this->massMatrix = massMatrix_;
	this->volumetricMesh = volumetricMesh_;

	int numElements = volumetricMesh->getNumElements();
	numElementVertices = volumetricMesh->getNumElementVertices();

	lambdaLame = (double*)malloc(sizeof(double) * numElements);
	muLame = (double*)malloc(sizeof(double) * numElements);

	for (int el = 0; el < numElements; el++)
	{
		LoboVolumetricMesh::Material * material = volumetricMesh->getElementMaterial(el);
		LoboVolumetricMesh::ENuMaterial * eNuMaterial = downcastENuMaterial(material);
		if (eNuMaterial == NULL)
		{
			printf("Error: StVKStiffnessMatrix: mesh does not consist of E, nu materials.\n");
			throw 1;
		}

		lambdaLame[el] = eNuMaterial->getLambda();
		muLame[el] = eNuMaterial->getMu();
	}

	//
	stiffnessMatrixTopology = new SparseMatrix<double>;
	getSparseMatrixTopology(*stiffnessMatrixTopology);
	fullstiffnessmatrix = new SparseMatrix<double>;
	getSparseMatrixTopology(*fullstiffnessmatrix);

	r = reducedModes.cols();
	r2 = r*r;

	initElementPhi();

}

void ReducedSTVKModelCubature::computeForce(double * reducedDisplacement, double * internalForces, std::vector<int>* ele_indices /*= NULL*/)
{
	int numVertex = volumetricMesh->getNumVertices();
	VectorXd q(r);
	memcpy(q.data(), reducedDisplacement, sizeof(double)*r);

	VectorXd fullq(numVertex * 3);
	fullq = reducedModes*q;

	VectorXd fullForce(numVertex * 3);
	fullForce.setZero();
	VectorXd reducedforce(r);
	reducedforce.setZero();

	computeForceNoCubature(fullq.data(), fullForce, reducedforce);

	if (addGravity)
	{
		reducedforce -= reducedGravityForce;
	}

	memcpy(internalForces, reducedforce.data(), sizeof(double)*r);
}




void ReducedSTVKModelCubature::computeStiffnessMatrix(double *reducedDisplacement, MatrixXd *matrix, bool accelerationindices /*= true*/, std::vector<int>* ele_indices /*= NULL*/)
{
	int numVertex = volumetricMesh->getNumVertices();
	VectorXd q(r);
	memcpy(q.data(), reducedDisplacement, sizeof(double)*r);
	VectorXd fullq(numVertex * 3);
	fullq = reducedModes*q;

	computeStiffnessMatrixNoCubature(fullq.data(), matrix);

}

void ReducedSTVKModelCubature::computeStiffnessMatrixNoCubature(double* fulldisplacement, MatrixXd* matrix)
{
	matrix->resize(r, r);
	stvkmodel->computeStiffnessMatrix(fulldisplacement, fullstiffnessmatrix);

	(*matrix) = reducedModes.transpose()*(*fullstiffnessmatrix)*reducedModes;
}

void ReducedSTVKModelCubature::computeForceNoCubature(double* fulldisplacement, VectorXd& internalForces, VectorXd& reducedForces)
{
	stvkmodel->computeForce(fulldisplacement, internalForces.data());
	reducedForces = reducedModes.transpose()*internalForces;
}

void ReducedSTVKModelCubature::initElementPhi()
{
	int numElement = volumetricMesh->getNumElements();
	ele_phi.clear();
	ele_phi_T.clear();

	for (int i = 0; i < numElement; i++)
	{
		MatrixXd elementphi(12, reducedModes.cols());

		for (int x = 0; x < 4; x++)
		{
			int nodeid = volumetricMesh->getElementNode(i, x);
			for (int y = 0; y < 3; y++)
			{
				int rowInEle = x * 3 + y;
				int rowInPhi = nodeid * 3 + y;

				for (int c = 0; c < reducedModes.cols(); c++)
				{
					elementphi.data()[c*elementphi.rows() + rowInEle] = reducedModes.data()[c*reducedModes.rows() + rowInPhi];
				}

			}
		}
		ele_phi.push_back(elementphi);
	}
}

