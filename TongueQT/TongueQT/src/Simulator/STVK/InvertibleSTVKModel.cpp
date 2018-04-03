#include "InvertibleSTVKModel.h"
#include "LoboVolumetricMesh/TetVolumetricMesh.h"
#include "SparseMatrix/SparseMatrixTopology.h"
#include <iostream>
#include "LoboVolumetricMesh/TetElement.h"
#include "LoboVolumetricMesh/LoboVolumetricMesh.h"
#include "LoboVolumetricMesh/ENuMaterial.h"
#include <fstream>
#include "Functions/computeSVDMatrix3d.h"
typedef Eigen::Triplet<double> EIGEN_TRI;



InvertibleSTVKModel::InvertibleSTVKModel(LoboVolumetricMesh* volumetricMesh, SparseMatrix<double>* massMatrix_, double invertible_threshold, int enableComparessionResistance /*= 0*/, double compressionResistance /*= 0*/) :STVKModel(volumetricMesh,massMatrix_)
{
	this->inversion_Threshold = invertible_threshold;
	this->enableCompressionResistance = enableComparessionResistance;
	this->compressionResistance = compressionResistance;

	int numElements = volumetricMesh->getNumElements();
	if (enableCompressionResistance)
	{
		EdivNuFactor = (double*)malloc(sizeof(double) * numElements);
	}
	else
	{
		EdivNuFactor = NULL;
	}

	if (enableCompressionResistance)
	for (int el = 0; el < numElements; el++)
	{
		LoboVolumetricMesh::Material * material = volumetricMesh->getElementMaterial(el);
		LoboVolumetricMesh::ENuMaterial * eNuMaterial = downcastENuMaterial(material);
		if (eNuMaterial != NULL)
		{
			EdivNuFactor[el] = compressionResistance * eNuMaterial->getE() / (1.0 - 2.0 * eNuMaterial->getNu());
		}
	}

}


InvertibleSTVKModel::~InvertibleSTVKModel()
{
	if (enableCompressionResistance)
	{
		free(EdivNuFactor);
	}
}

void InvertibleSTVKModel::updateMaterial()
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

	if (enableCompressionResistance)
		for (int el = 0; el < numElements; el++)
		{
			LoboVolumetricMesh::Material * material = volumetricMesh->getElementMaterial(el);
			LoboVolumetricMesh::ENuMaterial * eNuMaterial = downcastENuMaterial(material);
			if (eNuMaterial != NULL)
			{
				EdivNuFactor[el] = compressionResistance * eNuMaterial->getE() / (1.0 - 2.0 * eNuMaterial->getNu());
			}
		}
}

void InvertibleSTVKModel::precomputedFdU()
{
	int numElement = volumetricMesh->getNumElements();

	std::vector<Matrix3d> dF_tet(12);
	TetVolumetricMesh* tetmesh = downCastTetVolMesh(volumetricMesh);

	for (int i = 0; i < numElement; i++)
	{
		this->diffDeformationGradient(*(tetmesh->getTetElement(i)), dF_tet);
		dF.push_back(dF_tet);

		MatrixXd dF_du_tet(9, 12);
		for (size_t j = 0; j < 12; j++)
		{
			for (size_t i = 0; i < 9; i++)
			{
				dF_du_tet(i, j) = dF_tet[j].data()[i];
			}
		}
		dF_du.push_back(dF_du_tet);
	}

}

void InvertibleSTVKModel::computeForce(double * vertexDisplacement, double * internalForces, std::vector<int>* ele_indices /*= NULL*/, bool linear /*= true*/, bool quadratic /*= true*/, bool cubic /*= true*/)
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

	
	addContributionForce(vertexDisplacement, internalForces, ele_indices);


	if (addGravity)
	{
		for (int i = 0; i < numVertices * 3; i++)
		{
			internalForces[i] -= gravityForce.data()[i];
		}
	}
}

void InvertibleSTVKModel::computeStiffnessMatrix(double * vertexDisplacement, SparseMatrix<double>* sparseMatrix, bool accelerationindices /*= true*/, std::vector<int>* ele_indices /*= NULL*/, bool linear /*= true*/, bool quadratic /*= true*/, bool cubic /*= true*/)
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
		matirxcoef->reserve(numElement * 12 * 12);
	}

	addContributionMatrix(vertexDisplacement, sparseMatrix, ele_indices, matirxcoef);


	if (!accelerationindices)
	{
		sparseMatrix->setZero();
		sparseMatrix->setFromTriplets(matirxcoef->begin(), matirxcoef->end());
		delete matirxcoef;
	}

}



void InvertibleSTVKModel::computeStiffnessMatrix(std::vector<int> ele_indices, std::vector<MatrixXd*> elementmatrix, SparseMatrix<double>* stiffnessmatrix)
{

}

void InvertibleSTVKModel::computeForceAndMatrix(double* vertexDisplacement, double* internalForces, SparseMatrix<double>* sparseMatrix, std::vector<int>* ele_indices /*= NULL*/)
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
	resetSparseMatrix(sparseMatrix);

	addContributionForceAndMatrix(vertexDisplacement, internalForces, sparseMatrix, ele_indices);

	if (addGravity)
	{
		for (int i = 0; i < numVertices * 3; i++)
		{
			internalForces[i] -= gravityForce.data()[i];
		}
	}
}

void InvertibleSTVKModel::computeTetElementForce(TetElement* te, double* force)
{
	double lambda = lambdaLame[te->index_];
	double mu = muLame[te->index_];

	Matrix3d Ds, F, m_U, m_V, m_singularF, E, P;
	computeElementDeformationshapeMatrix(*te, Ds);
	F = Ds*te->Dm_inverse;

	computeSVD(F, m_U, m_V, m_singularF, 1e-8, 1);


	for (int i = 0; i < 3; i++)
	{
		if (m_singularF.data()[i * 3 + i] < inversion_Threshold)
		{
			m_singularF.data()[i * 3 + i] = inversion_Threshold;
		}
	}

	// compute element strain
	E = 0.5*(m_singularF.transpose()*m_singularF - Matrix3d().Identity());
	// compute PK1
	//P = m_singularF*(2.0 * mu*E + lambda*E.trace()*Matrix3d().Identity());

	double fhat[3], phat[3];
	fhat[0] = m_singularF.data()[0];
	fhat[1] = m_singularF.data()[4];
	fhat[2] = m_singularF.data()[8];

	ComputeDiagonalPFromStretches(*te, fhat, phat);
	P.setZero();
	P.data()[0] = phat[0];
	P.data()[4] = phat[1];
	P.data()[8] = phat[2];

	//compute real P
	P = m_U*P*m_V.transpose();



	//// compute element strain
	//E = 0.5*(m_singularF.transpose()*m_singularF - Matrix3d().Identity());
	//// compute PK1


	//P = m_singularF*(2.0 * mu*E + lambda*E.trace()*Matrix3d().Identity());

	////compute real P
	//P = m_U*P*m_V.transpose();

	//compute PK1

	for (int i = 0; i < 4; i++)
	{
		int n = te->node_indices[i];

		Vector3d f = P*te->nodes_norm_[i];

		force[i * 3] += f.data()[0];
		force[i * 3 + 1] += f.data()[1];
		force[i * 3 + 2] += f.data()[2];
	}

	
}

void InvertibleSTVKModel::computeTetElementMatrix(TetElement *te, MatrixXd* elementmatrix)
{
	double lambda = lambdaLame[te->index_];
	double mu = muLame[te->index_];

	Matrix3d Ds, F, m_U, m_V, m_singularF, E, P;
	Matrix3d dP_dF[9];
	MatrixXd m_12x9_dfdF(12, 9);
	computeElementDeformationshapeMatrix(*te, Ds);
	F = Ds*te->Dm_inverse;

	computeSVD(F, m_U, m_V, m_singularF, 1e-8, 1);

	for (int i = 0; i < 3; i++)
	{
		if (m_singularF.data()[i * 3 + i] < inversion_Threshold)
		{
			m_singularF.data()[i * 3 + i] = inversion_Threshold;
		}
	}

	// compute element strain
	E = 0.5*(m_singularF.transpose()*m_singularF - Matrix3d().Identity());
	// compute PK1
	//P = m_singularF*(2.0 * mu*E + lambda*E.trace()*Matrix3d().Identity());

	double fhat[3], phat[3];
	fhat[0] = m_singularF.data()[0];
	fhat[1] = m_singularF.data()[4];
	fhat[2] = m_singularF.data()[8];

	ComputeDiagonalPFromStretches(*te, fhat, phat);
	P.setZero();
	P.data()[0] = phat[0];
	P.data()[4] = phat[1];
	P.data()[8] = phat[2];

	//compute real P
	P = m_U*P*m_V.transpose();

	//// compute element strain
	//E = 0.5*(m_singularF.transpose()*m_singularF - Matrix3d().Identity());
	//// compute PK1
	//P = m_singularF*(2.0 * mu*E + lambda*E.trace()*Matrix3d().Identity());

	////compute real P
	//P = m_U*P*m_V.transpose();

	computedPdF(*te, F, m_U, m_V, m_singularF, E, dP_dF, mu, lambda);
	compute_dfdF(*te, dP_dF, m_12x9_dfdF, dF[te->index_]);

	//12*12
	MatrixXd stiffness = m_12x9_dfdF*dF_du[te->index_] * te->volume_;
	*elementmatrix = stiffness;

}


void InvertibleSTVKModel::addContributionForceAndMatrix(double *vertexDisplacement, double* force, SparseMatrix<double>*sparseMatrix, std::vector<int> *ele_indices /*= NULL*/)
{
	volumetricMesh->setDisplacementBuffer(vertexDisplacement);
	TetVolumetricMesh* tetmesh = downCastTetVolMesh(volumetricMesh);
	
	for (int i = 0; i < ele_indices->size(); i++)
	{
		int eleid = ele_indices->at(i);
		addTetElementForceAndMatrix(tetmesh->getTetElement(eleid), force, sparseMatrix);
	}

}

void InvertibleSTVKModel::addTetElementForceAndMatrix(TetElement*te, double*internalforce, SparseMatrix<double>* sparseMatrix, double weights /*= 1.0*/)
{
	double lambda = lambdaLame[te->index_];
	double mu = muLame[te->index_];

	Matrix3d Ds, F, m_U, m_V, m_singularF, E, P;
	Matrix3d dP_dF[9];
	MatrixXd m_12x9_dfdF(12, 9);
	computeElementDeformationshapeMatrix(*te, Ds);
	F = Ds*te->Dm_inverse;

	

	computeSVD(F, m_U, m_V, m_singularF, 1e-8, 1);



	for (int i = 0; i < 3; i++)
	{
		if (m_singularF.data()[i * 3 + i] < inversion_Threshold)
		{
			m_singularF.data()[i * 3 + i] = inversion_Threshold;
		}
	}

	// compute element strain
	E = 0.5*(m_singularF.transpose()*m_singularF - Matrix3d().Identity());
	// compute PK1
	//P = m_singularF*(2.0 * mu*E + lambda*E.trace()*Matrix3d().Identity());

	double fhat[3], phat[3];
	fhat[0] = m_singularF.data()[0];
	fhat[1] = m_singularF.data()[4];
	fhat[2] = m_singularF.data()[8];

	ComputeDiagonalPFromStretches(*te, fhat, phat);
	P.setZero();
	P.data()[0] = phat[0];
	P.data()[4] = phat[1];
	P.data()[8] = phat[2];

	//compute real P
	P = m_U*P*m_V.transpose();

	for (int i = 0; i < 4; i++)
	{
		int n = te->node_indices[i];

		Vector3d f = P*te->nodes_norm_[i];

		internalforce[n * 3] += f.data()[0];
		internalforce[n * 3 + 1] += f.data()[1];
		internalforce[n * 3 + 2] += f.data()[2];
	}

	
	computedPdF(*te, F, m_U, m_V, m_singularF, E, dP_dF, mu, lambda);
	compute_dfdF(*te, dP_dF, m_12x9_dfdF, dF[te->index_]);

	//12*12
	MatrixXd stiffness = m_12x9_dfdF*dF_du[te->index_] * te->volume_;

	for (int c = 0; c < 4; c++)
	{
		for (int a = 0; a < 4; a++)
		{
			int row = te->node_indices[c] * 3;
			int col = te->node_indices[a] * 3;

			for (int r = 0; r < 3; r++)
			{
				for (int l = 0; l < 3; l++)
				{

					int block_r = c * 3 + r;
					int block_c = a * 3 + l;
					
					int index = column_[te->index_][3 * numElementVertices*block_c + block_r];
					sparseMatrix->valuePtr()[index] += stiffness.data()[block_c * 12 + block_r];
				}
			}
		}
	}
}

void InvertibleSTVKModel::addContributionForce(double *vertexDisplacement, double* force, std::vector<int>* ele_indices)
{

	volumetricMesh->setDisplacementBuffer(vertexDisplacement);

	TetVolumetricMesh* tetmesh = downCastTetVolMesh(volumetricMesh);
	for (int i = 0; i < ele_indices->size(); i++)
	{
		int eleid = ele_indices->at(i);
		addTetElementForces(tetmesh->getTetElement(eleid), force);
	}
}

void InvertibleSTVKModel::addTetElementForces(TetElement *te, double *internalforce, double weight /*= 1.0*/)
{
	double lambda = lambdaLame[te->index_];
	double mu = muLame[te->index_];

	Matrix3d Ds,F,m_U,m_V,m_singularF,E,P;
	computeElementDeformationshapeMatrix(*te, Ds);
	F = Ds*te->Dm_inverse;

	computeSVD(F, m_U, m_V, m_singularF, 1e-8, 1);

	for (int i = 0; i < 3; i++)
	{
		if (m_singularF.data()[i * 3 + i] < inversion_Threshold)
		{
			m_singularF.data()[i * 3 + i] = inversion_Threshold;
		}
	}

	//// compute element strain
	//E = 0.5*(m_singularF.transpose()*m_singularF - Matrix3d().Identity());
	//// compute PK1
	//P = m_singularF*(2.0 * mu*E + lambda*E.trace()*Matrix3d().Identity());

	//if (te->index_ == 0)
	//{
	//	std::cout << "our method" << std::endl;
	//	std::cout << P << std::endl;
	//	std::cout << "vega" << std::endl;


	//	std::cout << phat[0] << std::endl;
	//	std::cout << phat[1] << std::endl;
	//	std::cout << phat[2] << std::endl;
	//}

	double fhat[3], phat[3];
	fhat[0] = m_singularF.data()[0];
	fhat[1] = m_singularF.data()[4];
	fhat[2] = m_singularF.data()[8];

	ComputeDiagonalPFromStretches(*te, fhat, phat);
	P.setZero();
	P.data()[0] = phat[0];
	P.data()[4] = phat[1];
	P.data()[8] = phat[2];

	//compute real P
	P = m_U*P*m_V.transpose();

	

	//compute PK1
	
	for (int i = 0; i < 4; i++)
	{
		int n = te->node_indices[i];
		
		Vector3d f = P*te->nodes_norm_[i];

		internalforce[n * 3] += f.data()[0];
		internalforce[n * 3+1] += f.data()[1];
		internalforce[n * 3+2] += f.data()[2];
	}
}

void InvertibleSTVKModel::addContributionMatrix(double *vertexDisplacment, SparseMatrix<double> *sparseMatrix, std::vector<int>* ele_indices /*= NULL*/, std::vector<Triplet<double>>* matirxcoef /*= NULL*/)
{
	volumetricMesh->setDisplacementBuffer(vertexDisplacment);
	TetVolumetricMesh* tetmesh = downCastTetVolMesh(volumetricMesh);


	for (int i = 0; i < ele_indices->size(); i++)
	{
		int ver = ele_indices->at(i);
		addTetElementMatrix(tetmesh->getTetElement(ver), sparseMatrix, matirxcoef);
	}
}

void InvertibleSTVKModel::addTetElementMatrix(TetElement *te, SparseMatrix<double> *sparseMatrix, std::vector<Triplet<double>>* matirxcoef /*= NULL*/, double weights /*= 1.0*/)
{
	double lambda = lambdaLame[te->index_];
	double mu = muLame[te->index_];

	Matrix3d Ds, F, m_U, m_V, m_singularF, E, P;
	Matrix3d dP_dF[9];
	MatrixXd m_12x9_dfdF(12,9);
	computeElementDeformationshapeMatrix(*te, Ds);
	F = Ds*te->Dm_inverse;

	computeSVD(F, m_U, m_V, m_singularF, 1e-8, 1);

	for (int i = 0; i < 3; i++)
	{
		if (m_singularF.data()[i * 3 + i] < inversion_Threshold)
		{
			m_singularF.data()[i * 3 + i] = inversion_Threshold;
		}
	}

	// compute element strain
	E = 0.5*(m_singularF.transpose()*m_singularF - Matrix3d().Identity());
	// compute PK1
	//P = m_singularF*(2.0 * mu*E + lambda*E.trace()*Matrix3d().Identity());

	double fhat[3], phat[3];
	fhat[0] = m_singularF.data()[0];
	fhat[1] = m_singularF.data()[4];
	fhat[2] = m_singularF.data()[8];

	ComputeDiagonalPFromStretches(*te, fhat, phat);
	P.setZero();
	P.data()[0] = phat[0];
	P.data()[4] = phat[1];
	P.data()[8] = phat[2];

	//compute real P
	P = m_U*P*m_V.transpose();

	computedPdF(*te, F, m_U, m_V, m_singularF, E, dP_dF,mu,lambda);
	compute_dfdF(*te, dP_dF, m_12x9_dfdF, dF[te->index_]);

	//12*12
	MatrixXd stiffness = m_12x9_dfdF*dF_du[te->index_]*te->volume_;

	for (int c = 0; c < 4; c++)
	{
		for (int a = 0; a < 4; a++)
		{
			int row = te->node_indices[c] * 3;
			int col = te->node_indices[a] * 3;

			for (int r = 0; r < 3; r++)
			{
				for (int l = 0; l < 3; l++)
				{

					int block_r = c * 3 + r;
					int block_c = a * 3 + l;

					if (matirxcoef != NULL)
					{
						matirxcoef->push_back(EIGEN_TRI(row+r,col+l,stiffness.data()[block_c*12+block_r]));
					}
					else
					{
						int index = column_[te->index_][3 * numElementVertices*block_c + block_r];
						sparseMatrix->valuePtr()[index] += stiffness.data()[block_c * 12 + block_r];
					}
				}
			}
		}
	}
	
}

void InvertibleSTVKModel::computeElementDeformationshapeMatrix(TetElement &te, Matrix3d &Ds)
{
	double* dis = volumetricMesh->getDisplacementRef();

	Ds.setZero();
	int ne[4];
	for (int i = 0; i < 4; i++)
	{
		ne[i] = te.node_indices[i];
	}

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			Ds.data()[j*3+i] = dis[ne[j] * 3 + i] - dis[ne[3] * 3 + i];
		}
	}

	Ds += te.Dm;
}

void InvertibleSTVKModel::diffDeformationGradient(TetElement &te, Matrix3d(&dF)[12])
{
	Matrix3d dDs[12];

	for (size_t i = 0; i < 9; i++)
	{
		dDs[i].setZero();
		dDs[i](i % 3, i / 3) = 1;
		dF[i] = dDs[i] * te.Dm_inverse;
	}

	for (size_t i = 0; i < 3; i++)
	{
		dDs[i + 9].setZero();
		dDs[i + 9](i, 0) = -1;
		dDs[i + 9](i, 1) = -1;
		dDs[i + 9](i, 2) = -1;
		dF[i + 9] = dDs[i + 9] * te.Dm_inverse;
	}
}

void InvertibleSTVKModel::diffDeformationGradient(TetElement &te, std::vector<Matrix3d> &dF)
{
	Matrix3d dDs[12];

	for (size_t i = 0; i < 9; i++)
	{
		dDs[i].setZero();
		dDs[i](i % 3, i / 3) = 1;
		dF[i] = dDs[i] * te.Dm_inverse;
	}

	for (size_t i = 0; i < 3; i++)
	{
		dDs[i + 9].setZero();
		dDs[i + 9](i, 0) = -1;
		dDs[i + 9](i, 1) = -1;
		dDs[i + 9](i, 2) = -1;
		dF[i + 9] = dDs[i + 9] * te.Dm_inverse;
	}
}

void InvertibleSTVKModel::computedPdF(TetElement &te, Matrix3d &F, Matrix3d &U, Matrix3d &V, Matrix3d &singularF, Matrix3d &singularE, Matrix3d(&dP_dF)[9], double mu /*= 1*/, double lambda/*=1*/)
{
	MatrixXd dP_dF9(9, 9);
	Matrix3d dP_dFij;
	Matrix3d diag_left;

	for (size_t i = 0; i < 3; i++)
	{
		for (size_t j = 0; j < 3; j++)
		{
			computedPdF_hatij(te, singularE, singularF, i, j, dP_dFij, mu,lambda);
			//dP_dF[i + j * 3] = dP_dFij;
			int col = (i + j * 3) * 9;
			dP_dF9.data()[col + 0] = dP_dFij.data()[0];
			dP_dF9.data()[col + 1] = dP_dFij.data()[1];
			dP_dF9.data()[col + 2] = dP_dFij.data()[2];
			dP_dF9.data()[col + 3] = dP_dFij.data()[3];
			dP_dF9.data()[col + 4] = dP_dFij.data()[4];
			dP_dF9.data()[col + 5] = dP_dFij.data()[5];
			dP_dF9.data()[col + 6] = dP_dFij.data()[6];
			dP_dF9.data()[col + 7] = dP_dFij.data()[7];
			dP_dF9.data()[col + 8] = dP_dFij.data()[8];
		}
	}


	for (size_t i = 0; i < 3; i++)
	{
		for (size_t j = 0; j < 3; j++)
		{
			diag_left.setZero();
			diag_left.data()[j * 3 + i] = 1;
			diag_left = U.transpose()*diag_left*V;

			for (int innerRow = 0; innerRow < 9; innerRow++)
			{
				double tempResult = 0.0;
				for (int innerColumn = 0; innerColumn < 9; innerColumn++)
				{
					tempResult += dP_dF9.data()[innerColumn * 9 + innerRow] * diag_left.data()[innerColumn];
				}
				dP_dF[i + j * 3].data()[innerRow] = tempResult;
			}
		}
	}

	for (size_t i = 0; i < 9; i++)
	{
		dP_dF[i] = U*dP_dF[i] * V.transpose();
	}
}

void InvertibleSTVKModel::computedPdF_hatij(TetElement &element, Matrix3d &E, Matrix3d &F, int i, int j, Matrix3d &dP_dFij, double mu, double lambda)
{
	Matrix3d dF_dFij, EFij;
	double tracEFij;
	dF_dFij.setZero();
	dF_dFij.data()[j * 3 + i] = 1;
	dP_dFij = dF_dFij*(2 * mu*E + lambda*E.trace()*Matrix3d().Identity());
	EFij = 0.5*(dF_dFij.transpose()*F + F.transpose()*dF_dFij);
	tracEFij = 0.5*(F(i, j) + F(j, i));
	dP_dFij += F*(2 * mu * EFij + lambda*tracEFij*Matrix3d().Identity());
}

void InvertibleSTVKModel::compute_dfdF(TetElement &element, Matrix3d(&dP_dF)[9], MatrixXd &dfdF, Matrix3d(&dF)[12])
{
	for (size_t i = 0; i < 9; i++)
	{
		for (size_t j = 0; j < 12; j++)
		{
			dfdF.data()[i * 12 + j] = (dF[j].transpose()*dP_dF[i]).trace();
		}
	}
}

void InvertibleSTVKModel::compute_dfdF(TetElement &element, Matrix3d(&dP_dF)[9], MatrixXd &dfdF, std::vector<Matrix3d> &dF)
{
	for (size_t i = 0; i < 9; i++)
	{
		for (size_t j = 0; j < 12; j++)
		{
			dfdF.data()[i * 12 + j] = (dF[j].transpose()*dP_dF[i]).trace();
		}
	}
}

void InvertibleSTVKModel::computeEnergyGradient(TetElement& element, double* invariants, double *gradient)
{
	int elementIndex = element.index_;
	double IC = invariants[0];
	gradient[0] = 0.25 * lambdaLame[elementIndex] * (IC - 3.0) - 0.5 * muLame[elementIndex];
	gradient[1] = 0.25 * muLame[elementIndex];
	gradient[2] = 0.0;

	//add compression ressistance gradient

	AddCompressionResistanceGradient(element, invariants, gradient);

}

void InvertibleSTVKModel::ComputeDiagonalPFromStretches(TetElement& element, double* lambda, double* PDiag)
{
	double invariants[3];

	double lambda2[3] = { lambda[0] * lambda[0], lambda[1] * lambda[1], lambda[2] * lambda[2] };
	double IC = lambda2[0] + lambda2[1] + lambda2[2];
	double IIC = lambda2[0] * lambda2[0] + lambda2[1] * lambda2[1] + lambda2[2] * lambda2[2];
	double IIIC = lambda2[0] * lambda2[1] * lambda2[2];

	invariants[0] = IC;
	invariants[1] = IIC;
	invariants[2] = IIIC;

	double dPsidI[3];

	computeEnergyGradient(element, invariants, dPsidI);

	Matrix3d mat;

	mat.data()[0] = 2.0 * lambda[0];
	mat.data()[1] = 2.0 * lambda[1];
	mat.data()[2] = 2.0 * lambda[2];
	mat.data()[3] = 4.0 * lambda[0] * lambda[0] * lambda[0];
	mat.data()[4] = 4.0 * lambda[1] * lambda[1] * lambda[1];
	mat.data()[5] = 4.0 * lambda[2] * lambda[2] * lambda[2];
	mat.data()[6] = 2.0 * lambda[0] * lambda2[1] * lambda2[2];
	mat.data()[7] = 2.0 * lambda[1] * lambda2[0] * lambda2[2];
	mat.data()[8] = 2.0 * lambda[2] * lambda2[0] * lambda2[1];


	Vector3d dPsidIV;
	dPsidIV.data()[0] = dPsidI[0];
	dPsidIV.data()[1] = dPsidI[1];
	dPsidIV.data()[2] = dPsidI[2];

	Vector3d result = mat*dPsidIV;
	PDiag[0] = result.data()[0];
	PDiag[1] = result.data()[1];
	PDiag[2] = result.data()[2];

}

void InvertibleSTVKModel::AddCompressionResistanceGradient(TetElement& element, double* invariants, double *gradient)
{
	if (enableCompressionResistance)
	{
		double IIIC = invariants[2];
		double J = sqrt(IIIC);


		if (J < 1)
		{
			double compressionResistanceFactor = GetCompressionResistanceFactor(element.index_);
			gradient[2] += -compressionResistanceFactor * (J - 1.0) * (J - 1.0) / (1728.0 * J);
		}
	}
}

double InvertibleSTVKModel::GetCompressionResistanceFactor(int elementIndex)
{
	return EdivNuFactor[elementIndex];
}
