#include "isotropicHyperelasticModel.h"
#include "SparseMatrix/SparseMatrixTopology.h"
#include "Functions/computeSVDMatrix3d.h"
#include "Simulator/IsotropicHyperelasticFEM/LoboIsotropicMaterial.h"
#include "polarDecomposition.h"
#include <fstream>
#include "Functions/SkewMatrix.h"

typedef Eigen::Triplet<double> EIGEN_TRI;

IsotropicHyperelasticModel::IsotropicHyperelasticModel(TetVolumetricMesh* tetMesh_, LoboIsotropicMaterial * isotropicMaterial_, SparseMatrix<double>* massMatrix_, double invertible_threshold_, bool enableComparessionResistance_ , double compressionResistance_ )
{
	implicitSpringForceK = 1500;
	implicitSpringOriLength = 0.25;
	this->setAddExternalSpring(false);

	this->inversion_Threshold = invertible_threshold_;
	this->compressionResistance = compressionResistance_;
	this->enableCompressionResistance = enableComparessionResistance_;
	this->isotropicMaterial = isotropicMaterial_;

	this->massMatrix = massMatrix_;
	this->volumetricMesh = tetMesh_;
	numElementVertices = this->volumetricMesh->getNumElementVertices();
	dofs_r = volumetricMesh->getNumVertices() * 3;

	stiffnessMatrixTopology = new SparseMatrix<double>;


	getSparseMatrixTopology(*stiffnessMatrixTopology);


	SparseMatrixTopology* sparseMatrixTopology = new SparseMatrixTopology(stiffnessMatrixTopology);

	int numElements = volumetricMesh->getNumElements();

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
						int block_r = i * 3 + k;
						int block_c = j * 3 + l;

						column_[el][3 * numElementVertices*block_c + block_r] = sparseMatrixTopology->getValueIndex(3 * row_[el][i] + k, 3 * row_[el][j] + l);
					}
				}
			}
	}

	initGravity();
	precomputedFdU();

	colMajorMatrixToTeran[0] = 0;
	colMajorMatrixToTeran[1] = 4;
	colMajorMatrixToTeran[2] = 6;
	colMajorMatrixToTeran[3] = 3;
	colMajorMatrixToTeran[4] = 1;
	colMajorMatrixToTeran[5] = 8;
	colMajorMatrixToTeran[6] = 5;
	colMajorMatrixToTeran[7] = 7;
	colMajorMatrixToTeran[8] = 2;

	for (int i = 0; i < 9; i++)
		teranToColMajorMatrix[colMajorMatrixToTeran[i]] = i;

	setIsInvertible(true);
}

IsotropicHyperelasticModel::IsotropicHyperelasticModel(TetVolumetricMesh* tetMesh_, LoboIsotropicMaterial * isotropicMaterial_, SparseMatrix<double>* massMatrix_, double invertible_threshold_, bool addExternalSpring_, std::vector<Vector2i> springPair_list_, bool enableComparessionResistance_ /*= 0*/, double compressionResistance_ /*= 0*/)
{
	implicitSpringForceK = 1500;
	implicitSpringOriLength = 0.25;

	this->setAddExternalSpring(addExternalSpring_);
	springPair_list = springPair_list_;

	this->inversion_Threshold = invertible_threshold_;
	this->compressionResistance = compressionResistance_;
	this->enableCompressionResistance = enableComparessionResistance_;
	this->isotropicMaterial = isotropicMaterial_;

	this->massMatrix = massMatrix_;
	this->volumetricMesh = tetMesh_;
	numElementVertices = this->volumetricMesh->getNumElementVertices();
	dofs_r = volumetricMesh->getNumVertices() * 3;

	stiffnessMatrixTopology = new SparseMatrix<double>;


	getSparseMatrixTopology(*stiffnessMatrixTopology);


	SparseMatrixTopology* sparseMatrixTopology = new SparseMatrixTopology(stiffnessMatrixTopology);

	int numElements = volumetricMesh->getNumElements();

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
						int block_r = i * 3 + k;
						int block_c = j * 3 + l;

						column_[el][3 * numElementVertices*block_c + block_r] = sparseMatrixTopology->getValueIndex(3 * row_[el][i] + k, 3 * row_[el][j] + l);
					}
				}
			}
	}

	spring_column_ = (int **)malloc(sizeof(int*)*springPair_list.size());
	for (int n = 0; n < springPair_list.size(); n++)
	{
		spring_column_[n] = (int*)malloc(sizeof(int) * 6 * 6);

		for (int i = 0; i < 2; i++)
		{
			for (int j = 0; j < 2; j++)
			{
				for (int k = 0; k < 3; k++)
				{
					for (int l = 0; l < 3; l++)
					{
						int block_r = i * 3 + k;
						int block_c = j * 3 + l;
						spring_column_[n][6 * block_c + block_r] = sparseMatrixTopology->getValueIndex(3 * springPair_list[n][i] + k, 3 * springPair_list[n][j]);
					}
				}
			}
		}
	}

	initGravity();
	precomputedFdU();

	colMajorMatrixToTeran[0] = 0;
	colMajorMatrixToTeran[1] = 4;
	colMajorMatrixToTeran[2] = 6;
	colMajorMatrixToTeran[3] = 3;
	colMajorMatrixToTeran[4] = 1;
	colMajorMatrixToTeran[5] = 8;
	colMajorMatrixToTeran[6] = 5;
	colMajorMatrixToTeran[7] = 7;
	colMajorMatrixToTeran[8] = 2;

	for (int i = 0; i < 9; i++)
		teranToColMajorMatrix[colMajorMatrixToTeran[i]] = i;

	setIsInvertible(true);
}

IsotropicHyperelasticModel::~IsotropicHyperelasticModel()
{
	int numElements = volumetricMesh->getNumElements();
	for (int i = 0; i < numElements; i++)
		free(row_[i]);
	free(row_);

	for (int i = 0; i < numElements; i++)
		free(column_[i]);
	free(column_);

}

void IsotropicHyperelasticModel::updateMaterial()
{
	isotropicMaterial->updateMaterial();
}

int IsotropicHyperelasticModel::getR()
{
	return volumetricMesh->getNumVertices() * 3;
}

void IsotropicHyperelasticModel::precomputedFdU()
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

void IsotropicHyperelasticModel::computeForce(double * vertexDisplacement, double * internalForces, std::vector<int>* ele_indices)
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
	
	if (getAddGravity())
	{
		for (int i = 0; i < numVertices * 3; i++)
		{
			internalForces[i] -= gravityForce.data()[i];
		}
	}
}

void IsotropicHyperelasticModel::computeStiffnessMatrix(double * vertexDisplacement, SparseMatrix<double>* sparseMatrix, bool accelerationindices /*= true*/, std::vector<int>* ele_indices /*= NULL*/)
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


	addContributionMatrix(vertexDisplacement, sparseMatrix, ele_indices, NULL);



}

void IsotropicHyperelasticModel::computeForceAndMatrix(double* vertexDisplacement, double* internalForces, SparseMatrix<double>* sparseMatrix, std::vector<int>* ele_indices /*= NULL*/)
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

	if (getAddExternalSpring())
	{
		addExtSpringForceAndMatrix(vertexDisplacement, internalForces, sparseMatrix);
	}

	if (getAddGravity())
	{
		for (int i = 0; i < numVertices * 3; i++)
		{
			internalForces[i] -= gravityForce.data()[i];
		}
	}
}

void IsotropicHyperelasticModel::computeTetElementForce(TetElement* te, double* internalforce)
{
	getTetElementForceAndMatrix(te, internalforce, NULL, ComputationMode::force);
}

void IsotropicHyperelasticModel::computeTetElementMatrix(TetElement* te, MatrixXd &stiffness)
{
	getTetElementForceAndMatrix(te, NULL, &stiffness, ComputationMode::matrix);
}

void IsotropicHyperelasticModel::computeTetForceAndMatrix(TetElement* te, double* internalforce, MatrixXd& stiffness)
{
	getTetElementForceAndMatrix(te, internalforce, &stiffness, 3);
}

double IsotropicHyperelasticModel::computeElementEnergy(int eleid)
{
	TetElement* te = volumetricMesh->getTetElement(eleid);

	Matrix3d Ds, F, m_U, m_V, m_singularF, E, P;
	Matrix3d dP_dF[9];
	MatrixXd m_12x9_dfdF(12, 9);
	computeElementDeformationshapeMatrix(*te, Ds);
	F = Ds*te->Dm_inverse;

	computeSVD(F, m_U, m_V, m_singularF, 1e-8, 1);

	if (getIsInvertible())
	for (int i = 0; i < 3; i++)
	{
		if (m_singularF.data()[i * 3 + i] < inversion_Threshold)
		{
			m_singularF.data()[i * 3 + i] = inversion_Threshold;
		}
	}

	double fhat[3], phat[3];
	fhat[0] = m_singularF.data()[0];
	fhat[1] = m_singularF.data()[4];
	fhat[2] = m_singularF.data()[8];

	double e = this->computeElementEnergyByI(*te, fhat);
	return e;
}

double IsotropicHyperelasticModel::computeCorotatedLinearEnergy(int eleid)
{
	TetElement* te = volumetricMesh->getTetElement(eleid);

	Matrix3d Ds, F, m_U, m_V, m_singularF, E, P;
	Matrix3d dP_dF[9];
	MatrixXd m_12x9_dfdF(12, 9);
	computeElementDeformationshapeMatrix(*te, Ds);
	F = Ds*te->Dm_inverse;

	//PolarDecomposition::
	double* M = (double*)malloc(sizeof(double) * 9);
	getRowMajorFromColMajor(F, M);

	double Q_d[9]; // output
	double S_d[9]; // output
	double tol = 1e-6;
	PolarDecomposition::Compute(M, Q_d, S_d, tol);
	Matrix3d R, S;

	rowMajorToColMajor(Q_d, R);
	rowMajorToColMajor(S_d, S);
	

	double lambda,mu;
	isotropicMaterial->getLambdaLame(eleid, lambda);
	isotropicMaterial->getMuLame(eleid, mu);
	S -= Matrix3d::Identity();

	double e = S.norm()*S.norm()*mu + lambda/2.0*S.trace()*S.trace();

	free(M);

	return e;

}

double IsotropicHyperelasticModel::computeLinearElasticityEnergy(int eleid)
{
	TetElement* te = volumetricMesh->getTetElement(eleid);

	Matrix3d Ds, F, m_U, m_V, m_singularF, E, P;
	Matrix3d dP_dF[9];
	MatrixXd m_12x9_dfdF(12, 9);
	computeElementDeformationshapeMatrix(*te, Ds);
	F = Ds*te->Dm_inverse;

	Matrix3d e = 0.5* (F.transpose() + F) - Matrix3d::Identity();

	double energy = 0;
	double lambda, mu;
	isotropicMaterial->getLambdaLame(eleid, lambda);
	isotropicMaterial->getMuLame(eleid, mu);

	for (int i = 0; i < 9; i++)
	{
		energy += e.data()[i] * e.data()[i] * mu;
	}

	energy += lambda*0.5*e.trace()*e.trace();
	return energy;
}

double IsotropicHyperelasticModel::computeElementStress(int eleid)
{
	TetElement* te = volumetricMesh->getTetElement(eleid);
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
	return P.norm();
}

double IsotropicHyperelasticModel::computeLinearElasticityStress(int eleid)
{
	TetElement* te = volumetricMesh->getTetElement(eleid);
	Matrix3d Ds, F, m_U, m_V, m_singularF, E, P;
	computeElementDeformationshapeMatrix(*te, Ds);
	F = Ds*te->Dm_inverse;
	double lambda, mu;
	isotropicMaterial->getLambdaLame(eleid, lambda);
	isotropicMaterial->getMuLame(eleid, mu);
	P = mu*(F + F.transpose() - 2 * Matrix3d::Identity()) + lambda*(F - Matrix3d::Identity()).trace()*Matrix3d::Identity();
	return P.trace();
}

double IsotropicHyperelasticModel::computeCorotatedLinearStress(int eleid)
{
	TetElement* te = volumetricMesh->getTetElement(eleid);

	Matrix3d Ds, F, m_U, m_V, m_singularF, E, P;
	Matrix3d dP_dF[9];
	MatrixXd m_12x9_dfdF(12, 9);
	computeElementDeformationshapeMatrix(*te, Ds);
	F = Ds*te->Dm_inverse;

	//PolarDecomposition::
	double* M = (double*)malloc(sizeof(double) * 9);
	getRowMajorFromColMajor(F, M);

	double Q_d[9]; // output
	double S_d[9]; // output
	double tol = 1e-6;
	PolarDecomposition::Compute(M, Q_d, S_d, tol);
	Matrix3d R, S;

	rowMajorToColMajor(Q_d, R);
	rowMajorToColMajor(S_d, S);


	double lambda, mu;
	isotropicMaterial->getLambdaLame(eleid, lambda);
	isotropicMaterial->getMuLame(eleid, mu);
	S -= Matrix3d::Identity();

	P = 2 * mu*S + lambda*S.trace()*Matrix3d::Identity();
	return P.norm();
}

void IsotropicHyperelasticModel::getSparseMatrixTopology(SparseMatrix<double>& sparesMatrixTopology)
{
	int numVertices = volumetricMesh->getNumVertices();
	int numElements = volumetricMesh->getNumElements();

	std::vector<int> vertices(numElementVertices);
	std::vector<EIGEN_TRI> entrys;

	for (int ele = 0; ele < numElements; ele++)
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

	if (getAddExternalSpring())
	{
		for (int n = 0; n < springPair_list.size(); n++)
		{

			for (int i = 0; i < 2; i++)
			{
				for (int j = 0; j < 2; j++)
				{
					for (int k = 0; k < 3; k++)
						for (int l = 0; l < 3; l++)
						{
							entrys.push_back(EIGEN_TRI(3 * springPair_list[n][i] + k, 3 * springPair_list[n][j] + l, 1.0));
						}
				}
			}
		}

	}

	sparesMatrixTopology.resize(numVertices * 3, numVertices * 3);
	sparesMatrixTopology.setFromTriplets(entrys.begin(), entrys.end());
}

void IsotropicHyperelasticModel::setGravityDirection(Vector3d direction)
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

void IsotropicHyperelasticModel::diffDeformationGradient(TetElement &te, std::vector<Matrix3d> &dF)
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

void IsotropicHyperelasticModel::addContributionForce(double *vertexDisplacement, double* force, std::vector<int>* ele_indices)
{
	volumetricMesh->setDisplacementBuffer(vertexDisplacement);
	for (int i = 0; i < ele_indices->size(); i++)
	{
		int eleid = ele_indices->at(i);
		//addTetElementForces(volumetricMesh->getTetElement(eleid), force);
		addTetElementForceAndMatrix(volumetricMesh->getTetElement(eleid), force, NULL, ComputationMode::force);
	}
}

void IsotropicHyperelasticModel::addTetElementForces(TetElement *te, double *internalforce, double weight /*= 1.0*/)
{
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
		internalforce[n * 3 + 1] += f.data()[1];
		internalforce[n * 3 + 2] += f.data()[2];
	}

}

void IsotropicHyperelasticModel::addContributionMatrix(double *vertexDisplacment, SparseMatrix<double> *sparseMatrix, std::vector<int>* ele_indices /*= NULL*/, std::vector<Triplet<double>>* matirxcoef /*= NULL*/)
{
	volumetricMesh->setDisplacementBuffer(vertexDisplacment);
	for (int i = 0; i < ele_indices->size(); i++)
	{
		int ver = ele_indices->at(i);
		addTetElementForceAndMatrix(volumetricMesh->getTetElement(ver), NULL, sparseMatrix, ComputationMode::matrix);
	}
}

void IsotropicHyperelasticModel::addTetElementMatrix(TetElement *te, SparseMatrix<double> *sparseMatrix, std::vector<Triplet<double>>* matirxcoef /*= NULL*/, double weights /*= 1.0*/)
{
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



}

void IsotropicHyperelasticModel::addContributionForceAndMatrix(double *vertexDisplacement, double* force, SparseMatrix<double>*sparseMatrix, std::vector<int> *ele_indices /*= NULL*/)
{
	volumetricMesh->setDisplacementBuffer(vertexDisplacement);
	for (int i = 0; i < ele_indices->size(); i++)
	{
		int eleid = ele_indices->at(i);
		addTetElementForceAndMatrix(volumetricMesh->getTetElement(eleid), force, sparseMatrix, 3);
	}

}

void IsotropicHyperelasticModel::addExtSpringForceAndMatrix(double *vertexDisplacement, double* force, SparseMatrix<double>*sparseMatrix)
{
	volumetricMesh->setDisplacementBuffer(vertexDisplacement);
	double k = implicitSpringForceK;

	for (int n = 0; n < springPair_list.size(); n++)
	{
		int nodei = springPair_list[n][0];
		int nodej = springPair_list[n][1];
		Vector3d ori_pi = volumetricMesh->getNodeRestPosition(nodei);
		Vector3d ori_pj = volumetricMesh->getNodeRestPosition(nodej);
		Vector3d cur_pi = volumetricMesh->getNodePosition(nodei);
		Vector3d cur_pj = volumetricMesh->getNodePosition(nodej);
		Vector3d forceDirection = cur_pi - cur_pj;

		double l = (ori_pi - ori_pj).norm();
		l = implicitSpringOriLength;

		if (forceDirection.norm() > l)
		{
			continue;
		}


		Vector3d forceoni = (k - (k*l) / forceDirection.norm())*forceDirection.normalized();
		Vector3d forceonj = -forceoni;

		force[nodei * 3 + 0] += forceoni.data()[0];
		force[nodei * 3 + 1] += forceoni.data()[1];
		force[nodei * 3 + 2] += forceoni.data()[2];

		force[nodej * 3 + 0] += forceonj.data()[0];
		force[nodej * 3 + 1] += forceonj.data()[1];
		force[nodej * 3 + 2] += forceonj.data()[2];

		MatrixXd stiffness(6, 6);
		stiffness.setZero();

		// k(|a-b|-L) \cdot (a-b)/|a-b|
		double identifyvalue = (k - (k*l) / forceDirection.norm());
		Matrix3d aa;
		aa.setIdentity();

		aa *= identifyvalue;
		
		double alpha = (cur_pi - cur_pj).norm();
		alpha = std::sqrt(alpha*alpha*alpha);
		alpha = 1.0 / alpha /2.0 * k*l;

		Vector3d left = alpha*(2 * (cur_pj - cur_pi));
		aa += (cur_pi - cur_pj) * left.transpose();

		Matrix3d ab;
		/*ab.setIdentity();
		ab *= -identifyvalue;

		ab += -(cur_pi - cur_pj) * left.transpose();*/
		ab = -aa;

		Matrix3d bb;
		bb = aa;
		Matrix3d ba = ab;
		stiffness.block(0, 0, 3, 3) = aa;
		stiffness.block(0, 3, 3, 3) = ab;
		stiffness.block(3, 0, 3, 3) = ba;
		stiffness.block(3, 3, 3, 3) = bb;


		for (int i = 0; i < 2; i++)
		{
			for (int j = 0; j < 2; j++)
			{
				for (int k = 0; k < 3; k++)
				{
					for (int l = 0; l < 3; l++)
					{
						int block_r = i * 3 + k;
						int block_c = j * 3 + l;
						 
						int index = spring_column_[n][6 * block_c + block_r];

						sparseMatrix->valuePtr()[index] += stiffness.data()[block_c * 6 + block_r];
					}
				}
			}
		}
	}
}

void IsotropicHyperelasticModel::addTetElementForceAndMatrix(TetElement*te, double*internalforce, SparseMatrix<double>* sparseMatrix, int forceormatrix_mode, double weights /*= 1.0*/)
{
	Matrix3d Ds, F, m_U, m_V, m_singularF, E, P;
	Matrix3d dP_dF[9];
	MatrixXd m_12x9_dfdF(12, 9);
	computeElementDeformationshapeMatrix(*te, Ds);
	F = Ds*te->Dm_inverse;

	computeSVD(F, m_U, m_V, m_singularF, 1e-8, 1);

	if (getIsInvertible())
	for (int i = 0; i < 3; i++)
	{
		if (m_singularF.data()[i * 3 + i] < inversion_Threshold)
		{
			m_singularF.data()[i * 3 + i] = inversion_Threshold;
		}
	}

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

	if (forceormatrix_mode&ComputationMode::force)
	{
		for (int i = 0; i < 4; i++)
		{
			int n = te->node_indices[i];

			Vector3d f = P*te->nodes_norm_[i];

			internalforce[n * 3] += f.data()[0];
			internalforce[n * 3 + 1] += f.data()[1];
			internalforce[n * 3 + 2] += f.data()[2];
		}
	}
	 
	if (forceormatrix_mode&ComputationMode::matrix)
	{
		compute_dPdF(*te, dP_dF, fhat, m_U, m_V);
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

}

void IsotropicHyperelasticModel::getTetElementForceAndMatrix(TetElement* te, double* internalforce, MatrixXd *stiffness, int forceormatrix_mode, double weights /*= 1.0*/)
{
	Matrix3d Ds, F, m_U, m_V, m_singularF, E, P;
	Matrix3d dP_dF[9];
	MatrixXd m_12x9_dfdF(12, 9);
	computeElementDeformationshapeMatrix(*te, Ds);
	F = Ds*te->Dm_inverse;

	computeSVD(F, m_U, m_V, m_singularF, 1e-8, 1);

	if (getIsInvertible())
		for (int i = 0; i < 3; i++)
		{
			if (m_singularF.data()[i * 3 + i] < inversion_Threshold)
			{
				m_singularF.data()[i * 3 + i] = inversion_Threshold;
			}
		}

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

	if (forceormatrix_mode&ComputationMode::force)
	{
		for (int i = 0; i < 4; i++)
		{
			Vector3d f = P*te->nodes_norm_[i];

			internalforce[i * 3] += f.data()[0];
			internalforce[i * 3 + 1] += f.data()[1];
			internalforce[i * 3 + 2] += f.data()[2];
		}
	}

	if (forceormatrix_mode&ComputationMode::matrix)
	{
		compute_dPdF(*te, dP_dF, fhat, m_U, m_V);
		compute_dfdF(*te, dP_dF, m_12x9_dfdF, dF[te->index_]);
		//12*12
		*stiffness = m_12x9_dfdF*dF_du[te->index_] * te->volume_;
	}
}

void IsotropicHyperelasticModel::compute_dPdF(TetElement& element, Matrix3d dPdF[9], double* fhat, Matrix3d &U, Matrix3d &V)
{
	double sigma1square = fhat[0] * fhat[0];
	double sigma2square = fhat[1] * fhat[1];
	double sigma3square = fhat[2] * fhat[2];

	double invariants[3];
	invariants[0] = sigma1square + sigma2square + sigma3square;
	invariants[1] = (sigma1square * sigma1square +
		sigma2square * sigma2square +
		sigma3square * sigma3square);
	invariants[2] = sigma1square * sigma2square * sigma3square;

	double gradient[3];
	isotropicMaterial->ComputeEnergyGradient(element.index_, invariants, gradient);

	double hessian[6];
	isotropicMaterial->ComputeEnergyHessian(element.index_, invariants, hessian);

	double alpha11 = 2.0 * gradient[0] + 8.0 * sigma1square * gradient[1];
	double alpha22 = 2.0 * gradient[0] + 8.0 * sigma2square * gradient[1];
	double alpha33 = 2.0 * gradient[0] + 8.0 * sigma3square * gradient[1];
	double alpha12 = 2.0 * gradient[0] + 4.0 * (sigma1square + sigma2square) * gradient[1];
	double alpha13 = 2.0 * gradient[0] + 4.0 * (sigma1square + sigma3square) * gradient[1];
	double alpha23 = 2.0 * gradient[0] + 4.0 * (sigma2square + sigma3square) * gradient[1];

	double beta11 = 4.0 * sigma1square * gradient[1] - (2.0 * invariants[2] * gradient[2]) / sigma1square;
	double beta22 = 4.0 * sigma2square * gradient[1] - (2.0 * invariants[2] * gradient[2]) / sigma2square;
	double beta33 = 4.0 * sigma3square * gradient[1] - (2.0 * invariants[2] * gradient[2]) / sigma3square;
	double beta12 = 4.0 * fhat[0] * fhat[1] * gradient[1] - (2.0 * invariants[2] * gradient[2]) / (fhat[0] * fhat[1]);
	double beta13 = 4.0 * fhat[0] * fhat[2] * gradient[1] - (2.0 * invariants[2] * gradient[2]) / (fhat[0] * fhat[2]);
	double beta23 = 4.0 * fhat[1] * fhat[2] * gradient[1] - (2.0 * invariants[2] * gradient[2]) / (fhat[1] * fhat[2]);

	double gamma11 = gammaValue(0, 0, fhat, invariants, gradient, hessian);
	double gamma22 = gammaValue(1, 1, fhat, invariants, gradient, hessian);
	double gamma33 = gammaValue(2, 2, fhat, invariants, gradient, hessian);
	double gamma12 = gammaValue(0, 1, fhat, invariants, gradient, hessian);
	double gamma13 = gammaValue(0, 2, fhat, invariants, gradient, hessian);
	double gamma23 = gammaValue(1, 2, fhat, invariants, gradient, hessian);

	double x1111, x2222, x3333;
	double x2211, x3311, x3322;
	double x2121, x3131, x3232;
	double x2112, x3113, x3223;

	x1111 = alpha11 + beta11 + gamma11;
	x2222 = alpha22 + beta22 + gamma22;
	x3333 = alpha33 + beta33 + gamma33;

	x2211 = gamma12;
	x3311 = gamma13;
	x3322 = gamma23;

	x2121 = alpha12;
	x3131 = alpha13;
	x3232 = alpha23;

	x2112 = beta12;
	x3113 = beta13;
	x3223 = beta23;

	double dPdF_atFhat[81];
	memset(dPdF_atFhat, 0, sizeof(double) * 81);
	dPdF_atFhat[tensor9x9Index(0, 0, 0, 0)] = x1111;
	dPdF_atFhat[tensor9x9Index(0, 0, 1, 1)] = x2211;
	dPdF_atFhat[tensor9x9Index(0, 0, 2, 2)] = x3311;

	dPdF_atFhat[tensor9x9Index(1, 1, 0, 0)] = x2211;
	dPdF_atFhat[tensor9x9Index(1, 1, 1, 1)] = x2222;
	dPdF_atFhat[tensor9x9Index(1, 1, 2, 2)] = x3322;

	dPdF_atFhat[tensor9x9Index(2, 2, 0, 0)] = x3311;
	dPdF_atFhat[tensor9x9Index(2, 2, 1, 1)] = x3322;
	dPdF_atFhat[tensor9x9Index(2, 2, 2, 2)] = x3333;

	dPdF_atFhat[tensor9x9Index(0, 1, 0, 1)] = x2121;
	dPdF_atFhat[tensor9x9Index(0, 1, 1, 0)] = x2112;

	dPdF_atFhat[tensor9x9Index(1, 0, 0, 1)] = x2112;
	dPdF_atFhat[tensor9x9Index(1, 0, 1, 0)] = x2121;

	dPdF_atFhat[tensor9x9Index(0, 2, 0, 2)] = x3131;
	dPdF_atFhat[tensor9x9Index(0, 2, 2, 0)] = x3113;

	dPdF_atFhat[tensor9x9Index(2, 0, 0, 2)] = x3113;
	dPdF_atFhat[tensor9x9Index(2, 0, 2, 0)] = x3131;

	dPdF_atFhat[tensor9x9Index(1, 2, 1, 2)] = x3232;
	dPdF_atFhat[tensor9x9Index(1, 2, 2, 1)] = x3223;

	dPdF_atFhat[tensor9x9Index(2, 1, 1, 2)] = x3223;
	dPdF_atFhat[tensor9x9Index(2, 1, 2, 1)] = x3232;



	double eiejVector[9];
	memset(eiejVector, 0, sizeof(double) * 9);
	memset(dPdF, 0, sizeof(double) * 81);
	Matrix3d elejmat;

	for (size_t i = 0; i < 3; i++)
	{
		for (size_t j = 0; j < 3; j++)
		{

			elejmat.setZero();
			elejmat.data()[j*3+i] = 1.0;

			Matrix3d ut_eiej_v = U.transpose()*elejmat*V;
			double ut_eiej_v_TeranVector[9]; //in Teran order
			for (int k = 0; k < 9; k++)
				ut_eiej_v_TeranVector[colMajorMatrixToTeran[k]] = ut_eiej_v.data()[k];

			double dPdF_resultVector[9]; // not in teran order

			for (int innerRow = 0; innerRow < 9; innerRow++)
			{
				double tempResult = 0.0;
				for (int innerColumn = 0; innerColumn < 9; innerColumn++)
				{
					tempResult += dPdF_atFhat[innerRow * 9 + innerColumn] * ut_eiej_v_TeranVector[innerColumn];
				}

				dPdF[j * 3 + i].data()[teranToColMajorMatrix[innerRow]] = tempResult;
			}
		}
	}

	for (size_t k = 0; k < 9; k++)
	{
		dPdF[k] = U*dPdF[k] * V.transpose();
	}
}

void IsotropicHyperelasticModel::compute_dfdF(TetElement &element, Matrix3d(&dP_dF)[9], MatrixXd &dfdF, std::vector<Matrix3d> &dF)
{
	for (size_t i = 0; i < 9; i++)
	{
		for (size_t j = 0; j < 12; j++)
		{
			dfdF.data()[i * 12 + j] = (dF[j].transpose()*dP_dF[i]).trace();
		}
	}
}

double IsotropicHyperelasticModel::gammaValue(int i, int j, double sigma[3], double invariants[3], double gradient[3], double hessian[6])
{
	/*
	The hessian is in order (11,12,13,22,23,33)
	| 11 12 13 |   | 0 1 2 |
	| 21 22 23 | = | 1 3 4 |
	| 31 32 33 |   | 2 4 5 |
	*/

	double tempGammaVec1[3];
	tempGammaVec1[0] = 2.0 * sigma[i];
	tempGammaVec1[1] = 4.0 * sigma[i] * sigma[i] * sigma[i];
	tempGammaVec1[2] = 2.0 * invariants[2] / sigma[i];
	double tempGammaVec2[3];
	tempGammaVec2[0] = 2.0 * sigma[j];
	tempGammaVec2[1] = 4.0 * sigma[j] * sigma[j] * sigma[j];
	tempGammaVec2[2] = 2.0 * invariants[2] / sigma[j];
	double productResult[3];
	productResult[0] = (tempGammaVec2[0] * hessian[0] + tempGammaVec2[1] * hessian[1] +
		tempGammaVec2[2] * hessian[2]);
	productResult[1] = (tempGammaVec2[0] * hessian[1] + tempGammaVec2[1] * hessian[3] +
		tempGammaVec2[2] * hessian[4]);
	productResult[2] = (tempGammaVec2[0] * hessian[2] + tempGammaVec2[1] * hessian[4] +
		tempGammaVec2[2] * hessian[5]);
	return (tempGammaVec1[0] * productResult[0] + tempGammaVec1[1] * productResult[1] +
		tempGammaVec1[2] * productResult[2] + 4.0 * invariants[2] * gradient[2] / (sigma[i] * sigma[j]));
}

int IsotropicHyperelasticModel::tensor9x9Index(int i, int j, int m, int n)
{
	/*
	|  dP_0/dF_0  dP_0/dF_4  dP_0/dF_8  ...  dP_0/dF_5  |
	|  dP_4/dF_0  dP_4/dF_4  dP_4/dF_8  ...  dP_4/dF_5  |
	|                         ...                       |
	|  dP_5/dF_0  dP_5/dF_4  dP_5/dF_8  ...  dP_5/dF_5  |
	*/

	int rowIndex_in9x9Matrix = colMajorMatrixToTeran[3 * j + i];
	int columnIndex_in9x9Matrix = colMajorMatrixToTeran[3 * n + m];
	return (9 * rowIndex_in9x9Matrix + columnIndex_in9x9Matrix);
}

void IsotropicHyperelasticModel::resetVector(double *vec)
{
	memset(vec, 0, sizeof(double) * 3 * volumetricMesh->getNumVertices());
}

void IsotropicHyperelasticModel::resetSparseMatrix(SparseMatrix<double>* sparseMatrix)
{
	for (int i = 0; i < sparseMatrix->outerSize(); ++i)
		for (SparseMatrix<double>::InnerIterator it(*sparseMatrix, i); it; ++it)
		{
			it.valueRef() = 0;
		}
}

void IsotropicHyperelasticModel::computeElementDeformationshapeMatrix(TetElement &te, Matrix3d &Ds)
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
			Ds.data()[j * 3 + i] = dis[ne[j] * 3 + i] - dis[ne[3] * 3 + i];
		}
	}

	Ds += te.Dm;
}

void IsotropicHyperelasticModel::ComputeDiagonalPFromStretches(TetElement& element, double* lambda, double* PDiag)
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

	isotropicMaterial->ComputeEnergyGradient(element.index_, invariants, dPsidI);

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

double IsotropicHyperelasticModel::computeElementEnergyByI(TetElement& element, double* lambda)
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
	double e = isotropicMaterial->ComputeEnergy(element.index_, invariants);
	return e;
}

void IsotropicHyperelasticModel::initGravity()
{
	int r = volumetricMesh->getNumVertices() * 3;
	this->gravityForce.resize(r);
	this->gravityForce.setZero();

	for (int i = 0; i < volumetricMesh->getNumVertices(); i++)
	{
		gravityForce.data()[i * 3 + 1] = -9.8;
	}
	gravityForce = (*massMatrix)*gravityForce;
}
