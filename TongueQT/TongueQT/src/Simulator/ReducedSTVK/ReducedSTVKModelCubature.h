#pragma once
#include "ReducedSTVKModel.h"
class STVKModel;

class ReducedSTVKModelCubature : public ReducedSTVKModel
{
public :
	ReducedSTVKModelCubature(LoboVolumetricMesh* volumetricMesh, STVKModel* stvkmodel, SparseMatrix<double>* massMatrix_, MatrixXd * reducedModes_);

	virtual void computeForce(double * reducedDisplacement, double * internalForces, std::vector<int>* ele_indices = NULL);

	virtual void computeStiffnessMatrix(double *reducedDisplacement, MatrixXd *matrix, bool accelerationindices = true, std::vector<int>* ele_indices = NULL);

protected:

	virtual void computeStiffnessMatrixNoCubature(double* fulldisplacement, MatrixXd* matrix);
	virtual void computeForceNoCubature(double* fulldisplacement, VectorXd& internalForces, VectorXd& reducedForces);

	void initElementPhi();

	STVKModel* stvkmodel;
	SparseMatrix<double>* fullstiffnessmatrix;

	std::vector<MatrixXd> ele_phi;
	std::vector<MatrixXd> ele_phi_T;

};