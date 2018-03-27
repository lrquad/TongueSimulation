#include "STVKForceModel.h"
#include "Simulator/STVK/STVKModel.h"
#include <ctime>
#include <iostream>
#include <fstream>

STVKForceModel::STVKForceModel(STVKModel* stvkmodel_)
{
	this->stvkmodel = stvkmodel_;
	this->r = stvkmodel->getR();
}


STVKForceModel::~STVKForceModel()
{
}

void STVKForceModel::getInternalForce(double *u, double* internalForces, bool linear, bool quadratic, bool cubic)
{
	stvkmodel->computeForce(u, internalForces,NULL,linear,quadratic,cubic);
}

void STVKForceModel::getTangentStiffnessMatrix(double *u, SparseMatrix<double> * tangentStiffnessMatrix, bool linear, bool quadratic, bool cubic)
{
	stvkmodel->computeStiffnessMatrix(u, tangentStiffnessMatrix, true,NULL, linear, quadratic, cubic);
}

void STVKForceModel::getTangentStiffnessMatrixTopology(SparseMatrix<double> * tangentStiffnessMatrix)
{
	stvkmodel->getSparseMatrixTopology(*tangentStiffnessMatrix);
}   