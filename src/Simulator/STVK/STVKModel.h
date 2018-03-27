#pragma once
#include "LoboVolumetricMesh/LoboVolumetricMesh.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/PardisoSupport>
#include "Functions/LoboDefine.h"
//#define EIGEN_USE_MKL_ALL


using namespace Eigen;
class TetVolumetricMesh;
class TetElement;


class STVKModel
{
public:
	typedef enum { ASCII, BINARY, NUM_FILE_FORMATS } fileFormatType;
	
	STVKModel(LoboVolumetricMesh* volumetricMesh, SparseMatrix<double>* massMatrix_);
	~STVKModel();

	virtual void updateMaterial();

	inline virtual int getR();

	virtual void computeForce(double * vertexDisplacement, double * internalForces, std::vector<int>* ele_indices = NULL,bool linear = true,bool quadratic = true,bool cubic = true);

	virtual void computeForceCubature(double * vertexDisplacement, double * internalForces, std::vector<int>* ele_indices, std::vector<double>* ele_weights);

	void setGravity(bool addGravity);
	bool getUseGravity();

	void getSparseMatrixTopology(SparseMatrix<double>& sparesMatrixTopology);

	virtual void computeStiffnessMatrix(double * vertexDisplacement, SparseMatrix<double>* sparseMatrix, bool accelerationindices = true, std::vector<int>* ele_indices = NULL, bool linear = true, bool quadratic = true, bool cubic = true);

	virtual void computeStiffnessMatrixCubature(double * vertexDisplacement, SparseMatrix<double>* sparseMatrix, bool accelerationindices, std::vector<int>* ele_indices, std::vector<double>* ele_weights);
	
	virtual void computeForceAndMatrix(double* vertexDisplacement, double* internalForces, SparseMatrix<double>* sparseMatrix, std::vector<int>* ele_indices = NULL);

	void setDisplacement(double *displacemenet);

	// === the routines below are meant for advanced usage ===

	virtual void addLinearTermsContributionForce(double * vertexDisplacement,double * force, std::vector<int>* ele_indices = NULL);
	virtual void addQuadraticTermsContributionForce(double * vertexDisplacement, double * force, std::vector<int>* ele_indices = NULL);
	virtual void addCubicTermsContributionForce(double * vertexDisplacement, double * force, std::vector<int>* ele_indices = NULL);

	virtual void addLinearTermsContributionMatrix(double * vertexDisplacement, SparseMatrix<double> * sparseMatrix, std::vector<int>* ele_indices = NULL, std::vector<Triplet<double>>* matirxcoef = NULL);
	virtual void addQuadraticTermContributionMatrix(double * vertexDisplacement, SparseMatrix<double> * sparseMatrix, std::vector<int>* ele_indices = NULL, std::vector<Triplet<double>>* matirxcoef = NULL);
	virtual void addCubicTermsContributionMatrix(double * vertexDisplacement, SparseMatrix<double> * sparseMatrix, std::vector<int>* ele_indices = NULL, std::vector<Triplet<double>>* matirxcoef = NULL);

	// === I/O ===

	virtual void exportABCD(const char* filename, fileFormatType filetype);
	virtual void exportAccelerationIndices(const char* filename, fileFormatType filetype);

	inline SparseMatrix<double>* getSparseMatrixTopologyMatrix() const{ return stiffnessMatrixTopology; };

	//=== compute matrix use input element matirx ===
	virtual void computeTetElementMatrix(TetElement *te, MatrixXd* elementmatrix);

	/**
	 * @brief Put brief description here
	 *
	 * Put full description here
	 *
	 * @param[in,out] te Put argument desc here
	 * @param[in,out] force 12*1 array
	 * @return Put return information here
	 */
	 virtual void computeTetElementForce(TetElement *te, double* force);

	virtual void computeStiffnessMatrix(std::vector<int> ele_indices, std::vector<MatrixXd*> elementmatrix, SparseMatrix<double>* stiffnessmatrix);

	virtual void addTetElementMatrix(TetElement *te, SparseMatrix<double> * sparseMatrix, MatrixXd* elematrix);

	virtual void updateGravityDirection(Vector3d direction);
	VectorXd getGravity();
	void setGravityVector(VectorXd g);
	void scaleGravityAndMass(double scale);
	virtual void setGravityDirection(Vector3d direction);

protected:

	
	void initGravity(); // aux function
	void initABCD();

	void addLinearTetElementForces(TetElement *te, double *internalforce, double weight = 1);
	void addQuadraticTetElementForces(TetElement *te, double *internalforce, double weight=1);
	void addCubicTetElementForces(TetElement *te, double *internalforce, double weight = 1);

	void addLinearTetElementMatrix(TetElement *te, SparseMatrix<double> * sparseMatrix, std::vector<Triplet<double>>* matirxcoef = NULL, double weights = 1);
	void addQuadraticTetElementMatrix(TetElement *te, SparseMatrix<double> * sparseMatrix, std::vector<Triplet<double>>* matirxcoef = NULL, double weights = 1);
	void addCubicTetElementMatrix(TetElement *te, SparseMatrix<double> * sparseMatrix, std::vector<Triplet<double>>* matirxcoef = NULL, double weights=1);

	void resetVector(double * vec);
	void resetSparseMatrix(SparseMatrix<double>* sparseMatrix);

	SparseMatrix<double>* stiffnessMatrixTopology;
	SparseMatrix<double>* massMatrix;
	



	LoboVolumetricMesh* volumetricMesh;

	// acceleration indices
	int ** row_;
	int ** column_;

	double* currentdisplacement;

	VectorXd gravityForce;
	bool addGravity;
	double g;

	int numElementVertices;
	int dofs_r;

	double *lambdaLame;
	double *muLame;

};

