#pragma once
#include "LoboVolumetricMesh/LoboVolumetricMesh.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>
using namespace Eigen;

class TetVolumetricMesh;
class TetElement;

class ReducedSTVKModel
{
public:
	typedef enum { ASCII, BINARY, NUM_FILE_FORMATS } fileFormatType;

	ReducedSTVKModel();
	ReducedSTVKModel(LoboVolumetricMesh* volumetricMesh, SparseMatrix<double>* massMatrix_,MatrixXd * reducedModes_);

	void computeReducedModesCoefficients();

	~ReducedSTVKModel();

	void getSparseMatrixTopology(SparseMatrix<double>& sparesMatrixTopology);

	virtual void computeForce(double * reducedDisplacement, double * internalForces, std::vector<int>* ele_indices = NULL);

	virtual void computeStiffnessMatrix(double *reducedDisplacement, MatrixXd *matrix, bool accelerationindices = true, std::vector<int>* ele_indices = NULL);

	virtual void computeReducedMassMatrix(MatrixXd* massMatrix);
	
	void computeGravity(VectorXd *gravity, SparseMatrix<double> *massMatrix,int R);
	inline void setGravity(bool b){ addGravity = b; };
	virtual void setGravityForce(double * g);

	void saveCoefMatrix(const char* filename);
	void loadCoefMatrix(const char* filename);

protected:

	void resetVector(double * vec);

	MatrixXd reducedModes;

	MatrixXd* reducedStiffnessMatrix;
	MatrixXd* reducedMassMatrix;

	SparseMatrix<double>* massMatrix;
	SparseMatrix<double>* stiffnessMatrixTopology;

	LoboVolumetricMesh* volumetricMesh;

	VectorXd reducedGravityForce;
	bool addGravity;
	double g;

	int numElementVertices;
	
	int r;
	int r2;

	int linearSize;
	int quadraticSize;
	int cubicSize;

	MatrixXd linearCoef_;
	MatrixXd quadraticCoef_;
	MatrixXd cubicCoef_;

	double *lambdaLame;
	double *muLame;

};

