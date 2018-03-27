#pragma once
#include "LoboVolumetricMesh/LoboVolumetriceMeshCore.h"
#include "Eigen/src/Geometry/AlignedBox.h"

class LoboIsotropicMaterial;

class IsotropicHyperelasticModel
{
public:
	typedef enum {force=1, matrix=2} ComputationMode;

	IsotropicHyperelasticModel(TetVolumetricMesh* tetMesh_, LoboIsotropicMaterial * isotropicMaterial_, SparseMatrix<double>* massMatrix_, double invertible_threshold,bool addExternalSpring_,std::vector<Vector2i> springPair_list_, bool enableComparessionResistance = 0, double compressionResistance = 0);
	IsotropicHyperelasticModel(TetVolumetricMesh* tetMesh_, LoboIsotropicMaterial * isotropicMaterial_, SparseMatrix<double>* massMatrix_, double invertible_threshold, bool enableComparessionResistance = 0, double compressionResistance = 0);

	~IsotropicHyperelasticModel();

	virtual void updateMaterial();

	inline virtual int getR();

	virtual void precomputedFdU();

	virtual void computeForce(double * vertexDisplacement, double * internalForces, std::vector<int>* ele_indices);

	virtual void computeStiffnessMatrix(double * vertexDisplacement, SparseMatrix<double>* sparseMatrix, bool accelerationindices = true, std::vector<int>* ele_indices = NULL);

	virtual void computeForceAndMatrix(double* vertexDisplacement, double* internalForces, SparseMatrix<double>* sparseMatrix, std::vector<int>* ele_indices = NULL);

	virtual void computeTetElementForce(TetElement* te, double* internalforce);
	virtual void computeTetElementMatrix(TetElement* te, MatrixXd &stiffness);

	virtual void computeTetForceAndMatrix(TetElement* te, double *internalforce, MatrixXd& stiffness);

	virtual double computeElementEnergy(int eleid);
	virtual double computeCorotatedLinearEnergy(int eleid);
	virtual double computeLinearElasticityEnergy(int eleid);

	virtual double computeElementStress(int eleid);
	virtual double computeCorotatedLinearStress(int eleid);
	virtual double computeLinearElasticityStress(int eleid);

	void getSparseMatrixTopology(SparseMatrix<double>& sparesMatrixTopology);

	bool getAddGravity() const { return addGravity; }
	void setAddGravity(bool val) { addGravity = val; }
	bool getIsInvertible() const { return isInvertible; }
	void setIsInvertible(bool val) { isInvertible = val; }
	virtual void setGravityDirection(Vector3d direction);

	bool getAddExternalSpring() const { return addExternalSpring; }
	void setAddExternalSpring(bool val) { addExternalSpring = val; }

	std::vector<Vector2i> getSpringPair_list() const { return springPair_list; }
	void setSpringPair_list(std::vector<Vector2i> val) { springPair_list = val; }
	
	double getImplicitSpringForceK() const { return implicitSpringForceK; }
	void setImplicitSpringForceK(double val) { implicitSpringForceK = val; }
	double getImplicitSpringOriLength() const { return implicitSpringOriLength; }
	void setImplicitSpringOriLength(double val) { implicitSpringOriLength = val; }
	Eigen::VectorXd getGravityForce() const { return gravityForce; }
	void setGravityForce(Eigen::VectorXd val) { gravityForce = val; }
protected:


	virtual void diffDeformationGradient(TetElement &te, std::vector<Matrix3d> &dF);
	
	//internal force
	virtual void addContributionForce(double *vertexDisplacement, double* force, std::vector<int>* ele_indices);
	virtual void addTetElementForces(TetElement *te, double *internalforce, double weight = 1.0);

	//stiffness matrix
	virtual void addContributionMatrix(double *vertexDisplacment, SparseMatrix<double> *sparseMatrix, std::vector<int>* ele_indices = NULL, std::vector<Triplet<double>>* matirxcoef = NULL);
	virtual void addTetElementMatrix(TetElement *te, SparseMatrix<double> *sparseMatrix, std::vector<Triplet<double>>* matirxcoef = NULL, double weights = 1.0);


	//matrix and force
	virtual void addContributionForceAndMatrix(double *vertexDisplacement, double* force, SparseMatrix<double>*sparseMatrix, std::vector<int> *ele_indices = NULL);
	virtual void addExtSpringForceAndMatrix(double *vertexDisplacement, double* force, SparseMatrix<double>*sparseMatrix);

	virtual void addTetElementForceAndMatrix(TetElement*te, double*internalforce, SparseMatrix<double>* sparseMatrix, int forceormatrix_mode, double weights = 1.0);

	virtual void getTetElementForceAndMatrix(TetElement* te, double* internalforce, MatrixXd *stiffness,int forceormatrix_mode, double weights = 1.0);

	virtual void compute_dPdF(TetElement& element,Matrix3d dPdF[9], double* fhat, Matrix3d &U, Matrix3d &V);
	void compute_dfdF(TetElement &element, Matrix3d(&dP_dF)[9], MatrixXd &dfdF, std::vector<Matrix3d> &dF);

	inline double gammaValue(int i, int j, double sigma[3], double invariants[3], double gradient[3], double hessian[6]);
	int tensor9x9Index(int i, int j, int m, int n);

	void resetVector(double *vec);
	void resetSparseMatrix(SparseMatrix<double>* sparseMatrix);

	virtual void computeElementDeformationshapeMatrix(TetElement &te, Matrix3d &Ds);

	void ComputeDiagonalPFromStretches(TetElement& element, double* lambda, double* PDiag);

	double computeElementEnergyByI(TetElement& element, double* lambda);

	void initGravity(); // aux function

	SparseMatrix<double>* stiffnessMatrixTopology;
	SparseMatrix<double>* massMatrix;

	TetVolumetricMesh* volumetricMesh;
	LoboIsotropicMaterial* isotropicMaterial;


	int numElementVertices;

	// acceleration indices
	int ** row_;
	int ** column_;

	//accleration for external spring
	int ** spring_column_;

	double* currentdisplacement;

	VectorXd gravityForce;
	
	bool addGravity;
	bool isInvertible;
	double inversion_Threshold;
	double compressionResistance;

	double g;

	int dofs_r;

	//precompute 
	std::vector<std::vector<Matrix3d>> dF; // 12* numelements
	std::vector<MatrixXd> dF_du; // 9X12

	bool enableCompressionResistance;


	int colMajorMatrixToTeran[9];
	int teranToColMajorMatrix[9];

	bool addExternalSpring;
	std::vector<Vector2i> springPair_list;

	double implicitSpringForceK;
	double implicitSpringOriLength;

};

