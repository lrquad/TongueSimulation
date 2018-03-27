#include "STVKModel.h"
#include "LoboVolumetricMesh/TetVolumetricMesh.h"
#include "SparseMatrix/SparseMatrixTopology.h"
#include <iostream>
#include "LoboVolumetricMesh/TetElement.h"
#include "LoboVolumetricMesh/LoboVolumetricMesh.h"
#include "LoboVolumetricMesh/ENuMaterial.h"
#include <fstream>

typedef Eigen::Triplet<double> EIGEN_TRI;

STVKModel::STVKModel(LoboVolumetricMesh* volumetricMesh, SparseMatrix<double>* massMatrix_) :addGravity(0)
{
	this->massMatrix = massMatrix_;
	this->volumetricMesh = volumetricMesh;
	numElementVertices = this->volumetricMesh->getNumElementVertices();
	dofs_r = volumetricMesh->getNumVertices() * 3;
	//init volumetric Mesh
	initABCD();

	stiffnessMatrixTopology = new SparseMatrix<double>;
	getSparseMatrixTopology(*stiffnessMatrixTopology);

	SparseMatrixTopology* sparseMatrixTopology = new SparseMatrixTopology(stiffnessMatrixTopology);
	
	int numElements = volumetricMesh->getNumElements();

	lambdaLame = (double*)malloc(sizeof(double) * numElements);
	muLame = (double*)malloc(sizeof(double) * numElements);
	currentdisplacement = (double*)malloc(sizeof(double)*dofs_r);

	for (int el = 0; el<numElements; el++)
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

	row_ = (int **)malloc(sizeof(int*)* numElements);
	column_ = (int **)malloc(sizeof(int*)*numElements);

	for (int el = 0; el < numElements; el++)
	{
		//std::cout << el << "\r";
		row_[el] = (int*)malloc(sizeof(int)*numElementVertices);
		column_[el] = (int*)malloc(sizeof(int)*numElementVertices*numElementVertices * 3 * 3);

		for (int ver = 0; ver < numElementVertices; ver++)
		{
			row_[el][ver] = volumetricMesh->getVertexIndex(el, ver);
		}

		for (int i = 0; i < numElementVertices; i++)
			for (int j = 0; j < numElementVertices; j++)
			{
				
				for (int k = 0; k < 3; k++)
				{
					for (int l = 0; l < 3; l++)
					{
						int block_r = i*3 + k;
						int block_c = j*3 + l;

						column_[el][3 * numElementVertices*block_c + block_r] = sparseMatrixTopology->getValueIndex(3 * row_[el][i] + k, 3 * row_[el][j] + l);
					}
				}
			}
	}

	initGravity();

}


STVKModel::~STVKModel()
{
	free(lambdaLame);
	free(muLame);

	int numElements = volumetricMesh->getNumElements();
	for (int i = 0; i<numElements; i++)
		free(row_[i]);
	free(row_);

	for (int i = 0; i<numElements; i++)
		free(column_[i]);
	free(column_);

	free(currentdisplacement);
}

void STVKModel::updateMaterial()
{

	int numElements = volumetricMesh->getNumElements();

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
}

void STVKModel::initGravity()
{
	int r = volumetricMesh->getNumVertices() * 3;
	this->gravityForce.resize(r);
	this->gravityForce.setZero();

	for (int i = 0; i < volumetricMesh->getNumVertices(); i++)
	{
		gravityForce.data()[i * 3 + 1] = -9.8;
		//gravityForce.data()[i * 3 + 2] = -9.8;
	}
	gravityForce = (*massMatrix)*gravityForce;
}

void STVKModel::initABCD()
{
	downCastTetVolMesh(volumetricMesh)->initMeshABCD();
}

void STVKModel::getSparseMatrixTopology(SparseMatrix<double>& sparesMatrixTopology)
{
	int numVertices = volumetricMesh->getNumVertices();
	int numElements = volumetricMesh->getNumElements();

	std::vector<int> vertices(numElementVertices);
	std::vector<EIGEN_TRI> entrys;

	for (int ele = 0; ele < numElements; ele ++)
	{
		for (int ver = 0; ver < numElementVertices; ver++)
		{
			vertices[ver] = volumetricMesh->getVertexIndex(ele, ver);
		}

		for (int i = 0; i < numElementVertices; i++)
		{
			for (int j = 0; j < numElementVertices; j++)
			{
				for (int k = 0; k < 3; k++)
					for (int l = 0; l < 3; l++)
					{
						//non-fixed
						entrys.push_back(EIGEN_TRI(3 * vertices[i] + k, 3 * vertices[j] + l, 1.0));
					}
			}
		}
	}

	sparesMatrixTopology.resize(numVertices * 3, numVertices * 3);
	sparesMatrixTopology.setFromTriplets(entrys.begin(), entrys.end());

}

void STVKModel::exportABCD(const char* filename, fileFormatType filetype)
{

}

void STVKModel::exportAccelerationIndices(const char* filename, fileFormatType filetype)
{

}

void STVKModel::setDisplacement(double *displacemenet)
{
	volumetricMesh->setDisplacement(displacemenet);
	memcpy(currentdisplacement, displacemenet, dofs_r*sizeof(double));
	volumetricMesh->setDisplacementBuffer(displacemenet);
}

void STVKModel::computeForce(double * vertexDisplacement, double * internalForces, std::vector<int>* ele_indices,bool linear,bool quadratic,bool cubic)
{
	int numElement = volumetricMesh->getNumElements();
	int numVertices = volumetricMesh->getNumVertices();
	if (ele_indices == NULL)
	{
		ele_indices = new std::vector < int >;
		ele_indices->resize(numElement);
		for (int i = 0; i < numElement; i++)
		{
			ele_indices->at(i) = i;
		}
	}

	resetVector(internalForces);

	if (linear)
		addLinearTermsContributionForce(vertexDisplacement, internalForces, ele_indices);
	if (quadratic)
		addQuadraticTermsContributionForce(vertexDisplacement, internalForces, ele_indices);
	if (cubic)
		addCubicTermsContributionForce(vertexDisplacement, internalForces, ele_indices);

	if (addGravity)
	{
		for (int i = 0; i < numVertices * 3; i++)
		{
			internalForces[i] -= gravityForce.data()[i];
		}
	}
}

void STVKModel::computeForceCubature(double * vertexDisplacement, double * internalForces, std::vector<int>* ele_indices, std::vector<double>* ele_weights)
{
	int numElement = volumetricMesh->getNumElements();
	int numVertices = volumetricMesh->getNumVertices();
	resetVector(internalForces);

	volumetricMesh->setDisplacement(vertexDisplacement);
	TetVolumetricMesh* tetmesh = downCastTetVolMesh(volumetricMesh);
	for (int i = 0; i < ele_indices->size(); i++)
	{
		int ver = ele_indices->at(i);
		addLinearTetElementForces(tetmesh->getTetElement(ver), internalForces,ele_weights->at(i));
		addQuadraticTetElementForces(tetmesh->getTetElement(ver), internalForces, ele_weights->at(i));
		addCubicTetElementForces(tetmesh->getTetElement(ver), internalForces,ele_weights->at(i));
	}

	if (addGravity)
	{
		for (int i = 0; i < numVertices * 3; i++)
		{
			internalForces[i] -= gravityForce.data()[i];
		}
	}
}

void STVKModel::resetVector(double * vec)
{
	memset(vec, 0, sizeof(double) * 3 * volumetricMesh->getNumVertices());
}

void STVKModel::resetSparseMatrix(SparseMatrix<double>* sparseMatrix)
{
	for (int i = 0; i < sparseMatrix->outerSize(); ++i)
		for (SparseMatrix<double>::InnerIterator it(*sparseMatrix, i); it; ++it)
		{
			it.valueRef() = 0;
		}
}

void STVKModel::setGravity(bool addGravity)
{
	this->addGravity = addGravity;
}

bool STVKModel::getUseGravity()
{
	return addGravity;
}

/// <summary>
/// Computes the stiffness matrix.
/// </summary>
/// <param name="vertexDisplacement">The vertex displacement.</param>
/// <param name="sparseMatrix">The sparse matrix.</param>
/// <param name="accelerationindices">if set to <c>true</c> will use [accelerationindices].</param>
/// <param name="ele_indices">The elements indices whilch will contribute to results.</param>
/// <param name="linear">if set to <c>true</c> [linear].</param>
/// <param name="quadratic">if set to <c>true</c> [quadratic].</param>
/// <param name="cubic">if set to <c>true</c> [cubic].</param>
void STVKModel::computeStiffnessMatrix(double * vertexDisplacement, SparseMatrix<double>* sparseMatrix, bool accelerationindices, std::vector<int>* ele_indices, bool linear, bool quadratic, bool cubic)
{
	int numElement = volumetricMesh->getNumElements();

	if (ele_indices == NULL)
	{
		ele_indices = new std::vector < int >;
		ele_indices->resize(numElement);
		for (int i = 0; i < numElement; i++)
		{
			ele_indices->at(i) = i;
		}
	}

	resetSparseMatrix(sparseMatrix);

	std::vector<Triplet<double>>* matirxcoef = NULL;
	if (!accelerationindices)
	{
		matirxcoef = new std::vector<Triplet<double>>;
		matirxcoef->reserve(numElement*12*12);
	}

	if (linear)
		addLinearTermsContributionMatrix(vertexDisplacement, sparseMatrix, ele_indices, matirxcoef);
	if (quadratic)
		addQuadraticTermContributionMatrix(vertexDisplacement, sparseMatrix, ele_indices, matirxcoef);
	if (cubic)
		addCubicTermsContributionMatrix(vertexDisplacement, sparseMatrix, ele_indices, matirxcoef);
	
	if (!accelerationindices)
	{
		sparseMatrix->setZero();
		sparseMatrix->setFromTriplets(matirxcoef->begin(), matirxcoef->end());
		delete matirxcoef;
	}
}

void STVKModel::computeStiffnessMatrixCubature(double * vertexDisplacement, SparseMatrix<double>* sparseMatrix, bool accelerationindices, std::vector<int>* ele_indices, std::vector<double>* ele_weights)
{
	int numElement = volumetricMesh->getNumElements();

	resetSparseMatrix(sparseMatrix);

	std::vector<Triplet<double>>* matirxcoef = NULL;
	if (!accelerationindices)
	{
		matirxcoef = new std::vector<Triplet<double>>;
		matirxcoef->reserve(numElement * 12 * 12);
	}

	volumetricMesh->setDisplacement(vertexDisplacement);
	TetVolumetricMesh* tetmesh = downCastTetVolMesh(volumetricMesh);
	for (int i = 0; i < ele_indices->size(); i++)
	{
		int ver = ele_indices->at(i);
		this->addLinearTetElementMatrix(tetmesh->getTetElement(ver), sparseMatrix, matirxcoef, ele_weights->at(i));
		this->addQuadraticTetElementMatrix(tetmesh->getTetElement(ver), sparseMatrix, matirxcoef, ele_weights->at(i));
		this->addCubicTetElementMatrix(tetmesh->getTetElement(ver), sparseMatrix, matirxcoef, ele_weights->at(i));
	}

	if (!accelerationindices)
	{
		sparseMatrix->setZero();
		sparseMatrix->setFromTriplets(matirxcoef->begin(), matirxcoef->end());
		delete matirxcoef;
	}
}

void STVKModel::computeForceAndMatrix(double* vertexDisplacement, double* internalForces, SparseMatrix<double>* sparseMatrix, std::vector<int>* ele_indices /*= NULL*/)
{
	computeForce(vertexDisplacement, internalForces);
	computeStiffnessMatrix(vertexDisplacement, sparseMatrix);
}

void STVKModel::computeStiffnessMatrix(std::vector<int> ele_indices, std::vector<MatrixXd*> elementmatrix, SparseMatrix<double>* stiffnessmatrix)
{
	int subNumElements = ele_indices.size();
	resetSparseMatrix(stiffnessmatrix);
	TetVolumetricMesh* tetmesh = downCastTetVolMesh(volumetricMesh);
	for (int i = 0; i < subNumElements; i++)
	{
		addTetElementMatrix(tetmesh->getTetElement(ele_indices[i]), stiffnessmatrix, elementmatrix[ele_indices[i]]);
	}
}



void STVKModel::addTetElementMatrix(TetElement *te, SparseMatrix<double> * sparseMatrix, MatrixXd* elematrix)
{
	for (int c = 0; c < 4; c++)
	{
		for (int a = 0; a < 4; a++)
		{
			for (int r = 0; r < 3; r++)
			{
				for (int l = 0; l < 3; l++)
				{
					
					int block_r = c * 3 + r;
					int block_c = a * 3 + l;

					int index = column_[te->index_][3 * numElementVertices*block_c + block_r];

					sparseMatrix->valuePtr()[index] += elematrix->data()[3 * numElementVertices*block_c + block_r];
				}
			}
		}
	}
}

void STVKModel::updateGravityDirection(Vector3d direction)
{
	int r = volumetricMesh->getNumVertices() * 3;
	this->gravityForce.resize(r);
	this->gravityForce.setZero();

	for (int i = 0; i < volumetricMesh->getNumVertices(); i++)
	{
		gravityForce.data()[i * 3 + 0] = direction.data()[0];
		gravityForce.data()[i * 3 + 1] = direction.data()[1];
		gravityForce.data()[i * 3 + 2] = direction.data()[2];
	}
	gravityForce = (*massMatrix)*gravityForce;
}

Eigen::VectorXd STVKModel::getGravity()
{
	return gravityForce;
}

void STVKModel::setGravityVector(VectorXd g)
{
	gravityForce = g;
}

void STVKModel::scaleGravityAndMass(double scale)
{
	*massMatrix *= scale;
	int r = volumetricMesh->getNumVertices() * 3;
	this->gravityForce.resize(r);
	this->gravityForce.setZero();

	for (int i = 0; i < volumetricMesh->getNumVertices(); i++)
	{
		gravityForce.data()[i * 3 + 1] = -9.8;
	}
	gravityForce = (*massMatrix)*gravityForce;
}

void STVKModel::setGravityDirection(Vector3d direction)
{
	int r = volumetricMesh->getNumVertices() * 3;
	this->gravityForce.resize(r);
	this->gravityForce.setZero();

	for (int i = 0; i < volumetricMesh->getNumVertices(); i++)
	{
		gravityForce.data()[i * 3 + 0] = direction.data()[0] * 9.8;
		gravityForce.data()[i * 3 + 1] = direction.data()[1] * 9.8;
		gravityForce.data()[i * 3 + 2] = direction.data()[2] * 9.8;
		//gravityForce.data()[i * 3 + 2] = -9.8;
	}
	gravityForce = (*massMatrix)*gravityForce;
}

void STVKModel::addLinearTermsContributionForce(double * vertexDisplacement, double * force, std::vector<int>* ele_indices)
{
	volumetricMesh->setDisplacement(vertexDisplacement);
	TetVolumetricMesh* tetmesh = downCastTetVolMesh(volumetricMesh);
	for (int i = 0; i < ele_indices->size(); i++)
	{
		int ver = ele_indices->at(i);
		addLinearTetElementForces(tetmesh->getTetElement(ver), force);
	}

}
void STVKModel::addQuadraticTermsContributionForce(double * vertexDisplacement, double * force, std::vector<int>* ele_indices)
{
	volumetricMesh->setDisplacement(vertexDisplacement);

	TetVolumetricMesh* tetmesh = downCastTetVolMesh(volumetricMesh);
	for (int i = 0; i < ele_indices->size(); i++)
	{
		int ver = ele_indices->at(i);
		addQuadraticTetElementForces(tetmesh->getTetElement(ver), force);
	}
}
void STVKModel::addCubicTermsContributionForce(double * vertexDisplacement, double * force, std::vector<int>* ele_indices)
{
	volumetricMesh->setDisplacement(vertexDisplacement);

	TetVolumetricMesh* tetmesh = downCastTetVolMesh(volumetricMesh);
	for (int i = 0; i < ele_indices->size(); i++)
	{
		int ver = ele_indices->at(i);
		addCubicTetElementForces(tetmesh->getTetElement(ver), force);
	}
}

void STVKModel::addLinearTermsContributionMatrix(double * vertexDisplacement, SparseMatrix<double> * sparseMatrix, std::vector<int>* ele_indices, std::vector<Triplet<double>>* matirxcoef)
{
	volumetricMesh->setDisplacement(vertexDisplacement);
	TetVolumetricMesh* tetmesh = downCastTetVolMesh(volumetricMesh);
	for (int i = 0; i < ele_indices->size(); i++)
	{
		int ver = ele_indices->at(i);
		this->addLinearTetElementMatrix(tetmesh->getTetElement(ver), sparseMatrix, matirxcoef);
	}
}

void STVKModel::addQuadraticTermContributionMatrix(double * vertexDisplacement, SparseMatrix<double> * sparseMatrix, std::vector<int>* ele_indices, std::vector<Triplet<double>>* matirxcoef)
{
	volumetricMesh->setDisplacement(vertexDisplacement);

	TetVolumetricMesh* tetmesh = downCastTetVolMesh(volumetricMesh);
	for (int i = 0; i < ele_indices->size(); i++)
	{
		int ver = ele_indices->at(i);
		this->addQuadraticTetElementMatrix(tetmesh->getTetElement(ver), sparseMatrix, matirxcoef);
	}
}

void STVKModel::addCubicTermsContributionMatrix(double * vertexDisplacement, SparseMatrix<double> * sparseMatrix, std::vector<int>* ele_indices, std::vector<Triplet<double>>* matirxcoef)
{
	volumetricMesh->setDisplacement(vertexDisplacement);

	TetVolumetricMesh* tetmesh = downCastTetVolMesh(volumetricMesh);
	for (int i = 0; i < ele_indices->size(); i++)
	{
		int ver = ele_indices->at(i);
		this->addCubicTetElementMatrix(tetmesh->getTetElement(ver), sparseMatrix, matirxcoef);
	}
}


void STVKModel::addLinearTetElementForces(TetElement *te, double *internalforce, double weight /*= 1*/)
{
	double lambda = lambdaLame[te->index_];
	double mu = muLame[te->index_];

	for (int c = 0; c < 4; c++)
	{
		for (int a = 0; a < 4; a++)
		{
			Vector3d qa = volumetricMesh->getNodeDisplacement(te->node_indices[a]);
			
			Vector3d force = lambda*(te->A[c][a] * qa) + mu*te->B[a][c]*qa + mu*te->A[a][c]*qa;

			internalforce[te->node_indices[c] * 3 + 0] += weight*force[0];
			internalforce[te->node_indices[c] * 3 + 1] += weight*force[1];
			internalforce[te->node_indices[c] * 3 + 2] += weight*force[2];
		}
	}
}

void STVKModel::addQuadraticTetElementForces(TetElement *te, double *internalforce, double weight/*=1*/)
{
	double lambda = lambdaLame[te->index_];
	double mu = muLame[te->index_];

	for (int c = 0; c < 4; c++)
	{
		for (int a = 0; a < 4; a++)
		{
			for (int b = 0; b < 4; b++)
			{
				Vector3d qa = volumetricMesh->getNodeDisplacement(te->node_indices[a]);
				Vector3d qb = volumetricMesh->getNodeDisplacement(te->node_indices[b]);
				double dotab = qa.dot(qb);

				Vector3d forceterm1 = 0.5*lambda*dotab*te->C[c][a][b] + mu*dotab*te->C[a][b][c];

				Vector3d C = lambda*te->C[a][b][c] + mu*te->C[c][a][b] + mu*te->C[b][a][c];

				double dotCqa = C.dot(qa);

				internalforce[te->node_indices[c] * 3 + 0] += weight*(forceterm1[0] + dotCqa*qb.data()[0]);
				internalforce[te->node_indices[c] * 3 + 1] += weight*(forceterm1[1] + dotCqa*qb.data()[1]);
				internalforce[te->node_indices[c] * 3 + 2] += weight*(forceterm1[2] + dotCqa*qb.data()[2]);

			}
		}
	}
}

void STVKModel::addCubicTetElementForces(TetElement *te, double *internalforce, double weight /*= 1*/)
{
	double lambda = lambdaLame[te->index_];
	double mu = muLame[te->index_];

	for (int c = 0; c < 4; c++)
	{
		for (int a = 0; a < 4; a++)
		{
			for (int b = 0; b < 4; b++)
			{
				for (int d = 0; d < 4; d++)
				{
					Vector3d qa = volumetricMesh->getNodeDisplacement(te->node_indices[a]);
					Vector3d qb = volumetricMesh->getNodeDisplacement(te->node_indices[b]);
					Vector3d qd = volumetricMesh->getNodeDisplacement(te->node_indices[d]);

					double dotp = qa.dot(qb);
					double scalar = dotp*(0.5*lambda*te->D[a][b][c][d] + mu*te->D[a][c][b][d]);

					internalforce[te->node_indices[c] * 3 + 0] += weight*scalar*qd[0];
					internalforce[te->node_indices[c] * 3 + 1] += weight*scalar*qd[1];
					internalforce[te->node_indices[c] * 3 + 2] += weight*scalar*qd[2];
				}
			}
		}
	}
}


void STVKModel::computeTetElementMatrix(TetElement *te, MatrixXd* elementmatrix)
{
	elementmatrix->resize(3 * numElementVertices, 3 * numElementVertices);
	elementmatrix->setZero();

	double lambda = lambdaLame[te->index_];
	double mu = muLame[te->index_];
	for (int c = 0; c < 4; c++)
	{
		for (int a = 0; a < 4; a++)
		{
			Matrix3d matrix;
			matrix.setIdentity();
			matrix *= mu*te->B[a][c];
			matrix += lambda*te->A[c][a] + mu*te->A[a][c];

			int row = te->node_indices[c] * 3;
			int col = te->node_indices[a] * 3;

			for (int r = 0; r < 3; r++)
			{
				for (int l = 0; l < 3; l++)
				{
					int block_r = c * 3 + r;
					int block_c = a * 3 + l;

					elementmatrix->data()[3 * numElementVertices*block_c + block_r]
							+= matrix.data()[l * 3 + r];
				}
			}
		}
	}
	
	for (int c = 0; c < 4; c++)
	{
		for (int e = 0; e < 4; e++)// compute contribution to block (c,e)
		{
			Matrix3d matrix;
			matrix.setZero();
			for (int a = 0; a < 4; a++)
			{
				Vector3d qa = volumetricMesh->getNodeDisplacement(te->node_indices[a]);
				Vector3d C0v = lambda*te->C[c][a][e] + mu*(te->C[e][a][c] + te->C[a][e][c]);
				//C0 tensor qa
				matrix.data()[0] += C0v.data()[0] * qa.data()[0]; matrix.data()[1] += C0v.data()[0] * qa.data()[1]; matrix.data()[2] += C0v.data()[0] * qa.data()[2];
				matrix.data()[3] += C0v.data()[1] * qa.data()[0]; matrix.data()[4] += C0v.data()[1] * qa.data()[1]; matrix.data()[5] += C0v.data()[1] * qa.data()[2];
				matrix.data()[6] += C0v.data()[2] * qa.data()[0]; matrix.data()[7] += C0v.data()[2] * qa.data()[1]; matrix.data()[8] += C0v.data()[2] * qa.data()[2];

				Vector3d C1v = lambda*te->C[e][a][c] + mu*(te->C[c][e][a] + te->C[a][e][c]);
				// qa tensor C1
				matrix.data()[0] += qa.data()[0] * C1v.data()[0]; matrix.data()[1] += qa.data()[0] * C1v.data()[1]; matrix.data()[2] += qa.data()[0] * C1v.data()[2];
				matrix.data()[3] += qa.data()[1] * C1v.data()[0]; matrix.data()[4] += qa.data()[1] * C1v.data()[1]; matrix.data()[5] += qa.data()[1] * C1v.data()[2];
				matrix.data()[6] += qa.data()[2] * C1v.data()[0]; matrix.data()[7] += qa.data()[2] * C1v.data()[1]; matrix.data()[8] += qa.data()[2] * C1v.data()[2];

				Vector3d C2v = lambda*te->C[a][e][c] + mu*(te->C[c][a][e] + te->C[e][a][c]);

				double dotp = qa.dot(C2v);

				matrix.data()[0] += dotp;
				matrix.data()[4] += dotp;
				matrix.data()[8] += dotp;
			}

			int row = te->node_indices[c] * 3;
			int col = te->node_indices[e] * 3;

			for (int k = 0; k < 3; k++)
			{
				for (int l = 0; l < 3; l++)
				{
					
					int block_r = c * 3 + k;
					int block_c = e * 3 + l;

					elementmatrix->data()[3 * numElementVertices*block_c + block_r]
							+= matrix.data()[k * 3 + l];
						
				}
			}
		}
	}

	for (int c = 0; c < 4; c++)
	{
		for (int e = 0; e < 4; e++) // compute contribution to block (c,e)
		{
			Matrix3d matrix;
			matrix.setZero();
			for (int a = 0; a < 4; a++)
			{
				Vector3d qa = volumetricMesh->getNodeDisplacement(te->node_indices[a]);

				for (int b = 0; b < 4; b++)
				{
					Vector3d qb = volumetricMesh->getNodeDisplacement(te->node_indices[b]);

					double D0 = lambda*te->D[a][c][b][e] + mu*(te->D[a][e][b][c] + te->D[a][b][c][e]);

					matrix.data()[0] += D0*qa.data()[0] * qb.data()[0]; matrix.data()[1] += D0*qa.data()[0] * qb.data()[1]; matrix.data()[2] += D0*qa.data()[0] * qb.data()[2];
					matrix.data()[3] += D0*qa.data()[1] * qb.data()[0]; matrix.data()[4] += D0*qa.data()[1] * qb.data()[1]; matrix.data()[5] += D0*qa.data()[1] * qb.data()[2];
					matrix.data()[6] += D0*qa.data()[2] * qb.data()[0]; matrix.data()[7] += D0*qa.data()[2] * qb.data()[1]; matrix.data()[8] += D0*qa.data()[2] * qb.data()[2];

					double D1 = 0.5*lambda*te->D[a][b][c][e] + mu*te->D[a][c][b][e];

					double dotpD = D1*(qa.data()[0] * qb.data()[0] + qa.data()[1] * qb.data()[1] + qa.data()[2] * qb.data()[2]);

					matrix.data()[0] += dotpD;
					matrix.data()[4] += dotpD;
					matrix.data()[8] += dotpD;
				}
			}

			int row = te->node_indices[c] * 3;
			int col = te->node_indices[e] * 3;

			for (int k = 0; k < 3; k++)
			{
				for (int l = 0; l < 3; l++)
				{
					int block_r = c * 3 + k;
					int block_c = e * 3 + l;

					elementmatrix->data()[3 * numElementVertices*block_c + block_r]
							+= matrix.data()[k * 3 + l];
				}
			}
		}
	}



}

void STVKModel::computeTetElementForce(TetElement *te, double* internalforce)
{
	double lambda = lambdaLame[te->index_];
	double mu = muLame[te->index_];

	for (int c = 0; c < 4; c++)
	{
		for (int a = 0; a < 4; a++)
		{
			Vector3d qa = volumetricMesh->getNodeDisplacement(te->node_indices[a]);

			Vector3d force = lambda*(te->A[c][a] * qa) + mu*te->B[a][c] * qa + mu*te->A[a][c] * qa;

			internalforce[c * 3 + 0] += force[0];
			internalforce[c * 3 + 1] += force[1];
			internalforce[c * 3 + 2] += force[2];
		}
	}

	for (int c = 0; c < 4; c++)
	{
		for (int a = 0; a < 4; a++)
		{
			for (int b = 0; b < 4; b++)
			{
				Vector3d qa = volumetricMesh->getNodeDisplacement(te->node_indices[a]);
				Vector3d qb = volumetricMesh->getNodeDisplacement(te->node_indices[b]);
				double dotab = qa.dot(qb);

				Vector3d forceterm1 = 0.5*lambda*dotab*te->C[c][a][b] + mu*dotab*te->C[a][b][c];

				Vector3d C = lambda*te->C[a][b][c] + mu*te->C[c][a][b] + mu*te->C[b][a][c];

				double dotCqa = C.dot(qa);

				internalforce[c * 3 + 0] += (forceterm1[0] + dotCqa*qb.data()[0]);
				internalforce[c * 3 + 1] += (forceterm1[1] + dotCqa*qb.data()[1]);
				internalforce[c * 3 + 2] += (forceterm1[2] + dotCqa*qb.data()[2]);

			}
		}
	}

	for (int c = 0; c < 4; c++)
	{
		for (int a = 0; a < 4; a++)
		{
			for (int b = 0; b < 4; b++)
			{
				for (int d = 0; d < 4; d++)
				{
					Vector3d qa = volumetricMesh->getNodeDisplacement(te->node_indices[a]);
					Vector3d qb = volumetricMesh->getNodeDisplacement(te->node_indices[b]);
					Vector3d qd = volumetricMesh->getNodeDisplacement(te->node_indices[d]);

					double dotp = qa.dot(qb);
					double scalar = dotp*(0.5*lambda*te->D[a][b][c][d] + mu*te->D[a][c][b][d]);

					internalforce[c * 3 + 0] += scalar*qd[0];
					internalforce[c * 3 + 1] += scalar*qd[1];
					internalforce[c * 3 + 2] += scalar*qd[2];
				}
			}
		}
	}

}

void STVKModel::addLinearTetElementMatrix(TetElement *te, SparseMatrix<double> * sparseMatrix, std::vector<Triplet<double>>* matirxcoef /*= NULL*/, double weights /*= 1*/)
{
	double lambda = lambdaLame[te->index_];
	double mu = muLame[te->index_];

	for (int c = 0; c < 4; c++)
	{
		for (int a = 0; a < 4; a++)
		{
			Matrix3d matrix;
			matrix.setIdentity();
			matrix *= mu*te->B[a][c];
			matrix += lambda*te->A[c][a] + mu*te->A[a][c];

			//cubature weight
			matrix *= weights;
			
			int row = te->node_indices[c] * 3;
			int col = te->node_indices[a] * 3;

			for (int r = 0; r < 3; r++)
			{
				for (int l = 0; l < 3; l++)
				{
					if (matirxcoef != NULL)
					{
						matirxcoef->push_back(EIGEN_TRI(row+r,col+l,matrix.data()[l*3+r]));

					}
					else
					{
						int block_r = c * 3 + r;
						int block_c = a * 3 + l;

						int index = column_[te->index_][3 * numElementVertices*block_c + block_r];

						sparseMatrix->valuePtr()[index] += matrix.data()[l * 3 + r];
					}
				}
			}
		}
	}
}


void STVKModel::addQuadraticTetElementMatrix(TetElement *te, SparseMatrix<double> * sparseMatrix, std::vector<Triplet<double>>* matirxcoef /*= NULL*/, double weights /*= 1*/ )
{
	double lambda = lambdaLame[te->index_];
	double mu = muLame[te->index_];

	for (int c = 0; c < 4; c++)
		{
			for (int e = 0; e < 4; e++)// compute contribution to block (c,e)
			{
				Matrix3d matrix;
				matrix.setZero();
				for (int a = 0; a < 4; a++)
				{
					Vector3d qa = volumetricMesh->getNodeDisplacement(te->node_indices[a]);
					Vector3d C0v = lambda*te->C[c][a][e] + mu*(te->C[e][a][c] + te->C[a][e][c]);
					//C0 tensor qa
					matrix.data()[0] += C0v.data()[0] * qa.data()[0]; matrix.data()[1] += C0v.data()[0] * qa.data()[1]; matrix.data()[2] += C0v.data()[0] * qa.data()[2];
					matrix.data()[3] += C0v.data()[1] * qa.data()[0]; matrix.data()[4] += C0v.data()[1] * qa.data()[1]; matrix.data()[5] += C0v.data()[1] * qa.data()[2];
					matrix.data()[6] += C0v.data()[2] * qa.data()[0]; matrix.data()[7] += C0v.data()[2] * qa.data()[1]; matrix.data()[8] += C0v.data()[2] * qa.data()[2];

					Vector3d C1v = lambda*te->C[e][a][c] + mu*(te->C[c][e][a] + te->C[a][e][c]);
					// qa tensor C1
					matrix.data()[0] += qa.data()[0] * C1v.data()[0]; matrix.data()[1] += qa.data()[0] * C1v.data()[1]; matrix.data()[2] += qa.data()[0] * C1v.data()[2];
					matrix.data()[3] += qa.data()[1] * C1v.data()[0]; matrix.data()[4] += qa.data()[1] * C1v.data()[1]; matrix.data()[5] += qa.data()[1] * C1v.data()[2];
					matrix.data()[6] += qa.data()[2] * C1v.data()[0]; matrix.data()[7] += qa.data()[2] * C1v.data()[1]; matrix.data()[8] += qa.data()[2] * C1v.data()[2];

					Vector3d C2v = lambda*te->C[a][e][c] + mu*(te->C[c][a][e] + te->C[e][a][c]);

					double dotp = qa.dot(C2v);

					matrix.data()[0] += dotp;
					matrix.data()[4] += dotp;
					matrix.data()[8] += dotp;
				}

				int row = te->node_indices[c] * 3;
				int col = te->node_indices[e] * 3;

				//cubature weight
				matrix *= weights;

				for (int k = 0; k < 3; k++)
				{
					for (int l = 0; l < 3; l++)
					{
						if (matirxcoef != NULL)
						{
							matirxcoef->push_back(EIGEN_TRI(row + k, col + l, matrix.data()[k * 3 + l]));

						}
						else
						{
							int block_r = c * 3 + k;
							int block_c = e * 3 + l;

							int index = column_[te->index_][3 * numElementVertices*block_c + block_r];
							sparseMatrix->valuePtr()[index] += matrix.data()[k * 3 + l];
						}
					}
				}
			}
		}

}

int STVKModel::getR()
{
	return volumetricMesh->getNumVertices() * 3;
}

void STVKModel::addCubicTetElementMatrix(TetElement *te, SparseMatrix<double> * sparseMatrix, std::vector<Triplet<double>>* matirxcoef /*= NULL*/, double weights/*=1*/ )
{
	double lambda = lambdaLame[te->index_];
	double mu = muLame[te->index_];

	for (int c = 0; c < 4; c++)
	{
		for (int e = 0; e < 4; e++) // compute contribution to block (c,e)
		{
			Matrix3d matrix;
			matrix.setZero();
			for (int a = 0; a < 4; a++)
			{
				Vector3d qa = volumetricMesh->getNodeDisplacement(te->node_indices[a]);

				for (int b = 0; b < 4; b++)
				{
					Vector3d qb = volumetricMesh->getNodeDisplacement(te->node_indices[b]);

					double D0 = lambda*te->D[a][c][b][e] + mu*(te->D[a][e][b][c] + te->D[a][b][c][e]);

					matrix.data()[0] += D0*qa.data()[0] * qb.data()[0]; matrix.data()[1] += D0*qa.data()[0] * qb.data()[1]; matrix.data()[2] += D0*qa.data()[0] * qb.data()[2];
					matrix.data()[3] += D0*qa.data()[1] * qb.data()[0]; matrix.data()[4] += D0*qa.data()[1] * qb.data()[1]; matrix.data()[5] += D0*qa.data()[1] * qb.data()[2];
					matrix.data()[6] += D0*qa.data()[2] * qb.data()[0]; matrix.data()[7] += D0*qa.data()[2] * qb.data()[1]; matrix.data()[8] += D0*qa.data()[2] * qb.data()[2];

					double D1 = 0.5*lambda*te->D[a][b][c][e] + mu*te->D[a][c][b][e];

					double dotpD = D1*(qa.data()[0] * qb.data()[0] + qa.data()[1] * qb.data()[1] + qa.data()[2] * qb.data()[2]);

					matrix.data()[0] += dotpD;
					matrix.data()[4] += dotpD;
					matrix.data()[8] += dotpD;
				}
			}

			//cubatrue weight
			matrix *= weights;

			int row = te->node_indices[c] * 3;
			int col = te->node_indices[e] * 3;

			for (int k = 0; k < 3; k++)
			{
				for (int l = 0; l < 3; l++)
				{
					if (matirxcoef != NULL)
					{
						matirxcoef->push_back(EIGEN_TRI(row + k, col + l, matrix.data()[k * 3 + l]));
					}
					else
					{

						int block_r = c * 3 + k;
						int block_c = e * 3 + l;

						int index = column_[te->index_][3 * numElementVertices*block_c + block_r];
						sparseMatrix->valuePtr()[index] += matrix.data()[k * 3 + l];

					}

				}
			}
		}
	}


}

