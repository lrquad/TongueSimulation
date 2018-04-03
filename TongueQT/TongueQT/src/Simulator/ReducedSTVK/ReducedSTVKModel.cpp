#include "ReducedSTVKModel.h"
#include "ReducedModesCoefficients.h"
#include "LoboVolumetricMesh/ENuMaterial.h"
#include "EigenMatrixIO/EigenMatrixIO.h"
#include <fstream>

typedef Eigen::Triplet<double> EIGEN_TRI;

ReducedSTVKModel::ReducedSTVKModel(LoboVolumetricMesh* volumetricMesh_, SparseMatrix<double>* massMatrix_, MatrixXd * reducedModes_)
{
	reducedModes = *reducedModes_;
	this->massMatrix = massMatrix_;
	this->volumetricMesh = volumetricMesh_;

	int numElements = volumetricMesh->getNumElements();
	numElementVertices = volumetricMesh->getNumElementVertices();

	lambdaLame = (double*)malloc(sizeof(double) * numElements);
	muLame = (double*)malloc(sizeof(double) * numElements);

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

	//
	stiffnessMatrixTopology = new SparseMatrix<double>;
	getSparseMatrixTopology(*stiffnessMatrixTopology);


	r = reducedModes.cols();
	r2 = r*r;

}

ReducedSTVKModel::ReducedSTVKModel()
{

}

void ReducedSTVKModel::computeReducedModesCoefficients()
{
	ReducedModesCoefficients* reducedmodesCoefficients
		= new ReducedModesCoefficients(&reducedModes, volumetricMesh, &linearCoef_, &quadraticCoef_, &cubicCoef_, lambdaLame, muLame, true);

	linearSize = reducedmodesCoefficients->getLinearSize();
	quadraticSize = reducedmodesCoefficients->getQuadraticSize();
	cubicSize = reducedmodesCoefficients->getCubicSize();
}

ReducedSTVKModel::~ReducedSTVKModel()
{
	delete (stiffnessMatrixTopology);
}


void ReducedSTVKModel::getSparseMatrixTopology(SparseMatrix<double>& sparesMatrixTopology)
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
	sparesMatrixTopology.resize(numVertices * 3, numVertices * 3);
	sparesMatrixTopology.setFromTriplets(entrys.begin(), entrys.end());
}

void ReducedSTVKModel::computeReducedMassMatrix(MatrixXd* massMatrix)
{
	*massMatrix = (reducedModes.transpose()*(*massMatrix)*(reducedModes));
}


void ReducedSTVKModel::computeGravity(VectorXd *gravity, SparseMatrix<double> *massMatrix, int R)
{
	VectorXd fullgravity(R);
	VectorXd fullacce(R);
	fullacce.setZero();
	for (int i = 0; i < R / 3; i++)
	{
		fullacce.data()[i * 3 + 1] = -9.8;
	}
	fullgravity = *massMatrix*fullacce;

	*gravity = reducedModes.transpose()*fullgravity;
}

void ReducedSTVKModel::setGravityForce(double * g)
{
	reducedGravityForce.resize(r);
	memcpy(reducedGravityForce.data(), g, r*sizeof(double));
}

void ReducedSTVKModel::saveCoefMatrix(const char* filename)
{
	std::ofstream out(filename, std::ios::out | std::ios::binary | std::ios::trunc);
	out.precision(outputprecision);
	EigenMatrixIO::write_binary(out, linearCoef_);
	EigenMatrixIO::write_binary(out, quadraticCoef_);
	EigenMatrixIO::write_binary(out, cubicCoef_);
	out.close();
}

void ReducedSTVKModel::loadCoefMatrix(const char* filename)
{
	std::ifstream in(filename, std::ios::in | std::ios::binary);
	if (!in.good())
	{
		std::cout << "coef matrix file not open" << std::endl;
		return;
	}
	EigenMatrixIO::read_binary(in, linearCoef_);
	EigenMatrixIO::read_binary(in, quadraticCoef_);
	EigenMatrixIO::read_binary(in, cubicCoef_);
	in.close();

	linearSize = linearCoef_.cols();
	quadraticSize = quadraticCoef_.cols();
	cubicSize = cubicCoef_.cols();
}

void ReducedSTVKModel::resetVector(double * vec)
{
	memset(vec, 0, sizeof(double)*r);
}

void ReducedSTVKModel::computeForce(double * reducedDisplacement, double * internalForces, std::vector<int>* ele_indices /*= NULL*/)
{
	int i, j, k, l, index;
	
	resetVector(internalForces);
	VectorXd internalforces(r);
	internalforces.setZero();
	//linear term
	for (int i = 0; i < r; i++)
	{
		internalforces += linearCoef_.col(i)*reducedDisplacement[i];
	}

	//
	for (int i = 0; i < r; i++)
		for (int j = 0; j < r; j++)
		{
			int pos = i*r + j;
			internalforces += quadraticCoef_.col(pos)*reducedDisplacement[i] * reducedDisplacement[j];
		}

	//
	for (int i = 0; i < r; i++)
		for (int j = 0; j < r; j++)
			for (int k = 0; k < r; k++)
			{
				int pos = i*r2 + j*r + k;
				internalforces += cubicCoef_.col(pos)*reducedDisplacement[i]*reducedDisplacement[j] * reducedDisplacement[k];
			}

	if (addGravity)
	{
		internalforces -= reducedGravityForce;
	}

	memcpy(internalForces, internalforces.data(), sizeof(double)*r);
}

void ReducedSTVKModel::computeStiffnessMatrix(double *reducedDisplacement, MatrixXd *matrix, bool accelerationindices /*= true*/, std::vector<int>* ele_indices /*= NULL*/)
{
	matrix->setZero();

	for (int l = 0; l < r; l++)
	{
		matrix->col(l) += linearCoef_.col(l);
	}

	for (int l = 0; l < r; l++)
	{
		for (int i = 0; i < r; i++)
		{
			int pos = l*r + i;
			matrix->col(l) += quadraticCoef_.col(pos)*reducedDisplacement[i];
			pos = i*r + l;
			matrix->col(l) += quadraticCoef_.col(pos)*reducedDisplacement[i];
		}
	}

	for (int l = 0; l < r; l++)
	{
		for (int i = 0; i < r; i++)
		{
			for (int j = 0; j < r; j++)
			{
				int pos = l*r2 + i*r + j;
				VectorXd cubicCoef = cubicCoef_.col(pos);
				pos = i*r2 + l*r + j;
				cubicCoef += cubicCoef_.col(pos);
				pos = i*r2 + j*r + l;
				cubicCoef += cubicCoef_.col(pos);
				matrix->col(l) += cubicCoef*reducedDisplacement[i] * reducedDisplacement[j];
			}
		}
	}

}