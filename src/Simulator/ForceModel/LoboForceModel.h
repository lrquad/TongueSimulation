#pragma once
#include <Eigen/Dense>
#include <Eigen/Sparse>
using namespace Eigen;
class LoboForceModel
{
public:

	LoboForceModel();
	~LoboForceModel();

	int getr(){ return r; }

	virtual void updateMaterial(){};

	virtual void computeLinearStiffnessMatrix(double *u, SparseMatrix<double> * tangentStiffnessMatrix);

	virtual void getInternalForce(double *u, double* internalForces, bool linear = true, bool quadratic = true, bool cubic = true) = 0;

	virtual void getTangentStiffnessMatrix(double *u, SparseMatrix<double> * tangentStiffnessMatrix, bool linear = true, bool quadratic = true, bool cubic = true) = 0;
	virtual void getTangentStiffnessMatrixTopology(SparseMatrix<double> * tangentStiffnessMatrix) = 0;

	virtual void ResetToZero(){};
	virtual void Reset(double *q){};

	virtual void getForceAndMatrix(double * u, double * internalForce, SparseMatrix<double> * tangentStiffnessMatrix);


 	virtual double computeElementEnergy(int eleid);

	// test the stiffness matrix, using finite differences
	// q is the configuration to test, dq is a small delta
	// if the stiffness matrix is correct, f(q) - f(q + eps * dq) - eps * K(q) * dq should be O(eps^2)
	void TestStiffnessMatrix(double *q,double *dq);

	bool getUselinearStiffnessMatrix() const { return uselinearStiffnessMatrix; }
	void setUselinearStiffnessMatrix(bool val) { uselinearStiffnessMatrix = val; }
protected:

	int r;

	bool uselinearStiffnessMatrix;

	SparseMatrix<double> * tangentStiffnessMatrix_rest;

};

