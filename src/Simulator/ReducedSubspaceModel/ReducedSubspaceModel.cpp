#include "ReducedSubspaceModel.h"
#include "LoboVolumetricMesh/LoboVolumetriceMeshCore.h"

#include "Simulator/IsotropicHyperelasticFEM/isotropicHyperelasticCore.h"
#include "cubature\ReducedCubatureTraining.h"
#include <fstream>


ReducedSubspaceModel::ReducedSubspaceModel(LoboVolumetricMesh* volumetricMesh, SparseMatrix<double>* massMatrix, IsotropicHyperelasticModel* coremodel, MatrixXd* phi)
{
	this->phi = *phi;
	int numVertex = volumetricMesh->getNumVertices();
	fullstiffnessmatrix = new SparseMatrix<double>(numVertex * 3, numVertex * 3);

	coremodel->getSparseMatrixTopology(*fullstiffnessmatrix);

	isotropicModel = coremodel;

	this->r =this->phi.cols();

	useCubature = false;
	usegravity = true;

	this->volumetricMesh = volumetricMesh;
	this->sparseMassMatrix = massMatrix;

	reducedCubatureTraining = NULL;
	initElementPhi();
	initGravity();
}

ReducedSubspaceModel::~ReducedSubspaceModel()
{

}

void ReducedSubspaceModel::computeForceAndMatrix(double* rq, double * internalForces, MatrixXd *matrix, std::vector<int>* node_indice /*= NULL*/)
{
	int numVertex = volumetricMesh->getNumVertices();

	VectorXd q(r);
	memcpy(q.data(), rq, sizeof(double)*r);

	VectorXd fullq(numVertex * 3);
	fullq = phi*q;

	if (!getUseCubature())
	{
		computeForceAndMatrixNoCubature(fullq.data(), internalForces, *matrix);
	}
	else
	{
		computeForceAndMatrixCubature(fullq.data(), internalForces, *matrix);
	}
}


void ReducedSubspaceModel::computeForce(double * frameDisplacement, double * internalForces, std::vector<int>* ele_indice /*= NULL*/)
{
	int numVertex = volumetricMesh->getNumVertices();
	VectorXd q(r);
	memcpy(q.data(), frameDisplacement, sizeof(double)*r);
	VectorXd fullq(numVertex * 3);
	fullq = phi*q;

	VectorXd eleforce(12);
	VectorXd reducedForces(r);
	reducedForces.setZero();
	volumetricMesh->setDisplacementBuffer(fullq.data());

	for (int i = 0; i < ele_indice->size(); i++)
	{
		int eleid = (*ele_indice)[i];
		
		TetElement* eleref = (TetElement*)volumetricMesh->getElement(eleid);
		eleforce.setZero();

		isotropicModel->computeTetElementForce(eleref, eleforce.data());

		reducedForces += ele_phi[eleid].transpose()*eleforce;
	}

	memcpy(internalForces, reducedForces.data(), sizeof(double)*r);
}

void ReducedSubspaceModel::computeForcebyfull(double* fulldisplacement, double* internalForces)
{
	int numVertex = volumetricMesh->getNumVertices();

	VectorXd fullForce(numVertex * 3);
	fullForce.setZero();
	VectorXd reducedforce(r);
	reducedforce.setZero();
	VectorXd cubatureforce(r);
	cubatureforce.setZero();

	if (!getUseCubature())
	{
		computeForceNoCubature(fulldisplacement, fullForce, reducedforce);
	}
	else
	{
		computeForceCubature(fulldisplacement, fullForce, reducedforce);
	}


	if (usegravity)
	{
		reducedforce -= gravityForce;
	}

	memcpy(internalForces, reducedforce.data(), sizeof(double)*r);
}



void ReducedSubspaceModel::computeStiffnessMatrixbyfull(double* fulldisplacement, MatrixXd* matrix)
{
	int numVertex = volumetricMesh->getNumVertices();
	if (!getUseCubature())
	{
		computeStiffnessMatrixNoCubature(fulldisplacement, *matrix);
	}
	else
	{
		computeStiffnessMatrixCubature(fulldisplacement, *matrix);
	}
}

void ReducedSubspaceModel::initElementPhi()
{
	int numElement = volumetricMesh->getNumElements();
	ele_phi.clear();
	ele_phi_T.clear();
	for (int i = 0; i < numElement; i++)
	{
		MatrixXd elementphi(12, phi.cols());

		for (int x = 0; x < 4; x++)
		{
			int nodeid = volumetricMesh->getElementNode(i, x);
			for (int y = 0; y < 3; y++)
			{
				int rowInEle = x * 3 + y;
				int rowInPhi = nodeid * 3 + y;
				for (int c = 0; c < phi.cols(); c++)
				{
					elementphi.data()[c*elementphi.rows() + rowInEle] = phi.data()[c*phi.rows() + rowInPhi];
				}
			}
		}
		ele_phi.push_back(elementphi);
	}
}

void ReducedSubspaceModel::computeGravity(VectorXd* gravity, SparseMatrix<double>* massMatrix)
{
	int R = volumetricMesh->getNumVertices() * 3;
	VectorXd accerlation(R);
	accerlation.setZero();
	for (int i = 0; i < R / 3; i++)
	{
		accerlation.data()[i * 3 + 1] = -9.8;
	}

	VectorXd result = *massMatrix*accerlation;
	result = phi.transpose()*result;
	memcpy(gravity->data(), result.data(), r*sizeof(double));
}

int ReducedSubspaceModel::getR()
{
	return phi.cols();
}

void ReducedSubspaceModel::initGravity()
{
	int R = volumetricMesh->getNumVertices() * 3;
	VectorXd accerlation(R);
	accerlation.setZero();
	for (int i = 0; i < R / 3; i++)
	{
		accerlation.data()[i * 3 + 1] = -9.8;
	}

	VectorXd result = *sparseMassMatrix*accerlation;
	result = phi.transpose()*result;
	gravityForce = result;
}

void ReducedSubspaceModel::computeForceCubature(double* fulldisplacement, VectorXd& internalforces, VectorXd& reducedForces)
{
	std::vector<int>* eleindice = reducedCubatureTraining->getCubatureSetRef();
	std::vector<double>* elewegiths = reducedCubatureTraining->getCubatureWeightSetRef();

	VectorXd eleforce(12);

	reducedForces.setZero();
	volumetricMesh->setDisplacementBuffer(fulldisplacement);

	for (int i = 0; i < eleindice->size(); i++)
	{
		int eleid = (*eleindice)[i];
		TetElement* eleref = (TetElement*)volumetricMesh->getElement(eleid);
		eleforce.setZero();
		isotropicModel->computeTetElementForce(eleref, eleforce.data());

		reducedForces += (*elewegiths)[i] * ele_phi[eleid].transpose()*eleforce;
	}

}

void ReducedSubspaceModel::computeForceNoCubature(double* fulldisplacement, VectorXd& internalforces, VectorXd& reducedForces)
{
	isotropicModel->computeForce(fulldisplacement, internalforces.data(),NULL);
	reducedForces = phi.transpose()*internalforces;
}

void ReducedSubspaceModel::computeStiffnessMatrixCubature(double* fulldisplacement, MatrixXd &matrix)
{
	std::vector<int>* eleindice = reducedCubatureTraining->getCubatureSetRef();
	std::vector<double>* eleweights = reducedCubatureTraining->getCubatureWeightSetRef();

	MatrixXd elementMatrix;
	matrix.resize(phi.cols(), phi.cols());
	matrix.setZero();

	volumetricMesh->setDisplacementBuffer(fulldisplacement);
	
	for (int i = 0; i < eleindice->size(); i++)
	{
		int eleid = (*eleindice)[i];

		TetElement* eleref = (TetElement*)volumetricMesh->getElement(eleid);
		elementMatrix.setZero();
		isotropicModel->computeTetElementMatrix(eleref, elementMatrix);
		matrix.noalias() += (*eleweights)[i] * ele_phi[eleid].transpose()*elementMatrix*ele_phi[eleid];
	}
}

void ReducedSubspaceModel::computeStiffnessMatrixNoCubature(double* fulldisplacement, MatrixXd& matrix)
{
	isotropicModel->computeStiffnessMatrix(fulldisplacement, fullstiffnessmatrix);
	matrix = phi.transpose()*(*fullstiffnessmatrix)*phi;
}

void ReducedSubspaceModel::computeForceAndMatrixCubature(double* fulldisplacement, double* reducediForces, MatrixXd& matrix)
{
	std::vector<int>* eleindice = reducedCubatureTraining->getCubatureSetRef();
	std::vector<double>* eleweights = reducedCubatureTraining->getCubatureWeightSetRef();

	VectorXd eleforce(12);
	
	MatrixXd elementMatrix;
	matrix.resize(phi.cols(), phi.cols());
	matrix.setZero();

	memset(reducediForces, 0, sizeof(double)*phi.cols());
	volumetricMesh->setDisplacementBuffer(fulldisplacement);

	for (int i = 0; i < eleindice->size(); i++)
	{
		int eleid = (*eleindice)[i];

		TetElement* eleref = (TetElement*)volumetricMesh->getElement(eleid);

		eleforce.setZero();
		elementMatrix.setZero();
		isotropicModel->computeTetForceAndMatrix(eleref, eleforce.data(), elementMatrix);
		
		matrix.noalias() += (*eleweights)[i] * ele_phi[eleid].transpose()*elementMatrix*ele_phi[eleid];

		VectorXd eleProjectedForce = (*eleweights)[i] * ele_phi[eleid].transpose()*eleforce;

		for (int j = 0; j < phi.cols(); j++)
		{
			reducediForces[j] += eleProjectedForce.data()[j];
		}
	}

	/*std::ofstream test("test.txt");
	for (int j = 0; j < phi.cols(); j++)
	{
	test << reducediForces[j] << std::endl;
	}
	test.close();*/
}

void ReducedSubspaceModel::computeForceAndMatrixNoCubature(double* fulldisplacement, double* reducidiForces, MatrixXd& matrix)
{
	memset(reducidiForces, 0, sizeof(double)*phi.cols());
	VectorXd fullinternalforce(volumetricMesh->getNumVertices() * 3);

	isotropicModel->computeForceAndMatrix(fulldisplacement, fullinternalforce.data(), fullstiffnessmatrix);

	VectorXd reducedInternalForce = phi.transpose()*fullinternalforce;

	for (int i = 0; i < phi.cols(); i++)
	{
		reducidiForces[i] = reducedInternalForce.data()[i];
	}

	matrix = phi.transpose()* (*fullstiffnessmatrix)*phi;

}

