#include "LinearSTVkForceModel.h"
#include "Simulator/STVK/STVKModel.h"
#include <ctime>
#include <iostream>
#include <fstream>

LinearSTVkForceModel::LinearSTVkForceModel(STVKModel* stvkmodel_) :STVKForceModel(stvkmodel_)
{
	uselinearStiffnessMatrix = true;
}

LinearSTVkForceModel::~LinearSTVkForceModel()
{
}

void LinearSTVkForceModel::updateMaterial()
{
	stvkmodel->updateMaterial();
}

void LinearSTVkForceModel::computeLinearStiffnessMatrix(double *u, SparseMatrix<double> * tangentStiffnessMatrix)
{
	stvkmodel->computeStiffnessMatrix(u, tangentStiffnessMatrix, true, NULL, true,false,false);
	this->tangentStiffnessMatrix_rest = tangentStiffnessMatrix;

}

void LinearSTVkForceModel::getInternalForce(double *u, double* internalForces, bool linear /*= true*/, bool quadratic /*= true*/, bool cubic /*= true*/)
{
	VectorXd uv(r);
	memcpy(uv.data(), u, sizeof(double)*r);

	VectorXd internalForce_v = *this->tangentStiffnessMatrix_rest*uv;



	VectorXd gravity = this->stvkmodel->getGravity();

	if (stvkmodel->getUseGravity())
	internalForce_v -= gravity;

	memcpy(internalForces, internalForce_v.data(), sizeof(double)*r);
}

void LinearSTVkForceModel::getTangentStiffnessMatrix(double *u, SparseMatrix<double> * tangentStiffnessMatrix, bool linear /*= true*/, bool quadratic /*= true*/, bool cubic /*= true*/)
{
	//do nothing
}

void LinearSTVkForceModel::getTangentStiffnessMatrixTopology(SparseMatrix<double> * tangentStiffnessMatrix)
{
	stvkmodel->getSparseMatrixTopology(*tangentStiffnessMatrix);
}

