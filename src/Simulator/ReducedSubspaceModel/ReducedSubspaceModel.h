#pragma once
#include <Eigen/Dense>
#include <Eigen/Sparse>

using namespace Eigen;

class LoboVolumetricMesh;
class IsotropicHyperelasticModel;
class ReducedCubatureTraining;

class ReducedSubspaceModel
{
public:
	ReducedSubspaceModel(LoboVolumetricMesh* volumetricMesh, SparseMatrix<double>* massMatrix, IsotropicHyperelasticModel* coremodel, MatrixXd* phi);

	~ReducedSubspaceModel();

	virtual void computeForceAndMatrix(double* q, double * internalForces, MatrixXd *matrix, std::vector<int>* node_indice = NULL);

	virtual void computeForce(double * frameDisplacement, double * internalForces, std::vector<int>* ele_indice = NULL);

	virtual void computeForcebyfull(double* fulldisplacement, double* internalForces);

	virtual void computeStiffnessMatrixbyfull(double* fulldisplacement, MatrixXd* matrix);

	virtual void initElementPhi();

	virtual void computeGravity(VectorXd* gravity, SparseMatrix<double>* massMatrix);


	int getR();

	bool getUseCubature() const { return useCubature; }
	void setUseCubature(bool val) { useCubature = val; }
	bool getUsegravity() const { return usegravity; }
	void setUsegravity(bool val) { usegravity = val; }
	VectorXd getGravityForce() const { return gravityForce; }
	void setGravityForce(VectorXd val) { gravityForce = val; }

	ReducedCubatureTraining* getReducedCubatureTraining() const { return reducedCubatureTraining; }
	void setReducedCubatureTraining(ReducedCubatureTraining* val) { reducedCubatureTraining = val; }
protected:
	
	virtual void initGravity();




	virtual void computeForceCubature(double* fulldisplacement,VectorXd& internalforces,VectorXd& reducedForces);

	virtual void computeForceNoCubature(double* fulldisplacement, VectorXd& internalforces, VectorXd& reducedForces);

	virtual void computeStiffnessMatrixCubature(double* fulldisplacement, MatrixXd &matrix);

	virtual void computeStiffnessMatrixNoCubature(double* fulldisplacement, MatrixXd& matrix);

	virtual void computeForceAndMatrixCubature(double* fulldisplacement, double* reducediForces, MatrixXd& matrix);

	virtual void computeForceAndMatrixNoCubature(double* fulldisplacement, double* reducidiForces, MatrixXd& matrix);

	bool useCubature;
	MatrixXd phi;
	SparseMatrix<double>* sparseMassMatrix;
	SparseMatrix<double>* fullstiffnessmatrix;
	IsotropicHyperelasticModel* isotropicModel;
	LoboVolumetricMesh* volumetricMesh;
	ReducedCubatureTraining* reducedCubatureTraining;

	VectorXd gravityForce;

	int r;
	std::vector<MatrixXd> ele_phi;
	std::vector<MatrixXd> ele_phi_T;


	bool usegravity;

};

