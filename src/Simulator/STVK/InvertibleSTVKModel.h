#pragma once
#include "LoboVolumetricMesh/LoboVolumetricMesh.h"
#include "STVKModel.h"

class InvertibleSTVKModel:public STVKModel
{
public:
	InvertibleSTVKModel(LoboVolumetricMesh* volumetricMesh, SparseMatrix<double>* massMatrix_, double invertible_threshold, int enableComparessionResistance = 0, double compressionResistance = 0);
	~InvertibleSTVKModel();

	virtual void updateMaterial();

	virtual void precomputedFdU();

	virtual void computeForce(double * vertexDisplacement, double * internalForces, std::vector<int>* ele_indices = NULL, bool linear = true, bool quadratic = true, bool cubic = true);

	virtual void computeStiffnessMatrix(double * vertexDisplacement, SparseMatrix<double>* sparseMatrix, bool accelerationindices = true, std::vector<int>* ele_indices = NULL, bool linear = true, bool quadratic = true, bool cubic = true);

	virtual void computeForceAndMatrix(double* vertexDisplacement, double* internalForces, SparseMatrix<double>* sparseMatrix, std::vector<int>* ele_indices = NULL);


	virtual void computeStiffnessMatrix(std::vector<int> ele_indices, std::vector<MatrixXd*> elementmatrix, SparseMatrix<double>* stiffnessmatrix);

	virtual void computeTetElementForce(TetElement* te, double* force);

	virtual void computeTetElementMatrix(TetElement *te, MatrixXd* elementmatrix);



	bool getEnableCompressionResistance() const { return enableCompressionResistance; }
	void setEnableCompressionResistance(bool val) { enableCompressionResistance = val; }
protected:

	virtual void addContributionForceAndMatrix(double *vertexDisplacement, double* force, SparseMatrix<double>*sparseMatrix, std::vector<int> *ele_indices = NULL);
	virtual void addTetElementForceAndMatrix(TetElement*te, double*internalforce, SparseMatrix<double>* sparseMatrix, double weights = 1.0);

	virtual void addContributionForce(double *vertexDisplacement, double* force, std::vector<int>* ele_indices);
	virtual void addTetElementForces(TetElement *te, double *internalforce, double weight = 1.0);
	
	virtual void addContributionMatrix(double *vertexDisplacment, SparseMatrix<double> *sparseMatrix, std::vector<int>* ele_indices = NULL, std::vector<Triplet<double>>* matirxcoef = NULL);
	virtual void addTetElementMatrix(TetElement *te, SparseMatrix<double> *sparseMatrix, std::vector<Triplet<double>>* matirxcoef = NULL, double weights = 1.0);

	//compute matrix

	virtual void computeElementDeformationshapeMatrix(TetElement &te, Matrix3d &Ds);
	virtual void diffDeformationGradient(TetElement &te, Matrix3d(&dF)[12]);
	virtual void diffDeformationGradient(TetElement &te, std::vector<Matrix3d> &dF);
	virtual void computedPdF(TetElement &te, Matrix3d &F, Matrix3d &U, Matrix3d &V, Matrix3d &singularF, Matrix3d &singularE, Matrix3d(&dP_dF)[9], double mu, double lambda);
	void computedPdF_hatij(TetElement &element,
		Matrix3d &E,
		Matrix3d &F,
		int i,
		int j,
		Matrix3d &dP_dF_hat_ij,double mu,double lambda);
	void compute_dfdF(TetElement &element, Matrix3d(&dP_dF)[9], MatrixXd &dfdF, Matrix3d(&dF)[12]);
	void compute_dfdF(TetElement &element, Matrix3d(&dP_dF)[9], MatrixXd &dfdF, std::vector<Matrix3d> &dF);


	//from vega
	void computeEnergyGradient(TetElement& element, double* invariants, double *gradient);
	void ComputeDiagonalPFromStretches(TetElement& element, double* lambda, double* PDiag);
	void AddCompressionResistanceGradient(TetElement& element, double* invariants, double *gradient);

	double inversion_Threshold;

	double compressionResistance;
	double * EdivNuFactor;
	virtual double GetCompressionResistanceFactor(int elementIndex);
	
	//precompute 
	std::vector<std::vector<Matrix3d>> dF; // 12* numelements
	std::vector<MatrixXd> dF_du; // 9X12

	bool enableCompressionResistance;

};

