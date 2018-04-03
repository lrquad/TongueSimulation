#pragma once
#include <Eigen/Dense>
#include <Eigen/Sparse>
using namespace Eigen;


class LoboForceModel;
class SubspaceModesCreator
{
public:
	/// <summary>
	/// Initializes a new instance of the <see cref="SupspaceModesCreater"/> class.
	/// </summary>
	/// <param name="forceModel">The force model.</param>
	/// <param name="massMatrix">The mass matrix.</param>
	/// <param name="k">The k num of linear modes.</param>
	/// <param name="r">The final modes.</param>
	SubspaceModesCreator(LoboForceModel* forceModel, SparseMatrix<double>* massMatrix, int numConstrainedDOFs_, int* constrainedDOFs_, int k, int r);
	SubspaceModesCreator(LoboForceModel* forceModel, SparseMatrix<double>* massMatrix, int numConstrainedDOFs_, int* constrainedDOFs_, int k, int r, MatrixXd* linearmodes_);

	~SubspaceModesCreator();

	virtual void generateModes();
	virtual void generateLinearModes();
	virtual void generateModesFromInput(MatrixXd* linearModes);
	
	virtual void insertZeros(MatrixXd* modes,MatrixXd* output);

	virtual void buildFreeLMAMode(int k);
	virtual void initLinearModes(MatrixXd* linearModes);
	virtual void modalDerivaties();
	virtual void modesPCA(int r);



	typedef enum { ASCII, BINARY, NUM_FILE_FORMATS } fileFormatType;

	/* =============================
	export and load
	=============================*/

	void exportModes(const char* filename, fileFormatType filetype);
	void exportModesBinary(const char* filename);

	void loadModes(const char* filename, fileFormatType filetype);
	void loadModesBinary(const char* filename);

	/* =============================
	output
	=============================*/

	 MatrixXd*  getLinearModes();
	 MatrixXd*  getDerivativeModes();
	 MatrixXd* getNonRigidLinearModes();
	
	//mostly use this
	 MatrixXd*  getFinalModes();
	 MatrixXd*  getRigidModes();

	void constraintsInsert(SparseMatrix<double> &result, SparseMatrix<double> &K, MatrixXd &U);

	MatrixXd getLinearModesNULL() const { return linearModesNULL; }
	void setLinearModesNULL(MatrixXd val) { linearModesNULL = val; }
	VectorXd getEigen_values() const { return eigen_values; }
	void setEigen_values(VectorXd val) { eigen_values = val; }
protected:


	std::vector<int> mapOldNew;
	std::vector<int> mapSparseMatrixEntryOldNew;

	int rigid_modes_number;
	int numDerivatesModes;

	int numConstrainedDOFs; 
	int* constrainedDOFs;

	LoboForceModel* forceModel;

	SparseMatrix<double> * tangentStiffnessMatrix;
	SparseMatrix<double> * massMatrix;

	MatrixXd linearmodes;
	MatrixXd linearModesNULL;
	MatrixXd removedRigidLinearmodes;
	MatrixXd derivativemodes;
	MatrixXd removedRigid_modes;
	MatrixXd rigidmodes;

	VectorXd eigen_values;

	std::string filenamebase;

	int k;
	int r;
	int r2;

};

