#pragma once
#include "LoboVolumetricMesh.h"

class TetElement;

class TetVolumetricMesh: public LoboVolumetricMesh
{
public:
	TetVolumetricMesh();
	~TetVolumetricMesh();


	virtual void readElementList();
	

	virtual void readElementMeshOnly(const char* filenamebase, double* translate = NULL, double scale = 1.0);
	virtual void initTetVolumetricMesh();

	/// <summary>
	/// This will call parent function first then call read Element List();
	/// </summary>
	/// <param name="filenamebase">The filenamebase.</param>
	/// <param name="translate">The translate.</param>


	virtual void readElementMesh(const char* filenamebase, double* translate = NULL, double scale = 1.0);

	virtual void saveElementMesh(std::ofstream &outstream);
	virtual void readElementMesh(std::ifstream &instream);
	
	virtual void searchNeighborElement();
	virtual bool containsVertex(int element, const Vector3d &pos) const;

	virtual void computeMassMatrix(std::vector<LoboElement*>* elementlist, SparseMatrix<double>* mass) const;
	virtual void computeMassMatrix2(std::vector<LoboElement*>* elementlist, SparseMatrix<double>* mass) const;

	virtual void computeMassMatrix(SparseMatrix<double>* mass);

	virtual void computeBarycentricWeights(int elementid,const Vector3d &pos, Vector4d &weight) const;

	virtual void computeSurface();
	virtual void releaseSurface();

	virtual void saveSurfaceToObjAscii(const char* filename) const;



	//************************************
	// Method:    saveSubMeshAsscii
	// FullName:  TetVolumetricMesh::saveSubMeshAsscii
	// Access:    virtual public 
	// Returns:   void
	// Qualifier:
	// Parameter: const char * filenamebase
	// Parameter: std::vector<int> element_list
	// Parameter: std::vector<bool> subnodemark
	// Parameter: std::vector<int> submap
	//************************************
	virtual void saveSubMeshAsscii(const char* filenamebase, std::vector<int> element_list, std::vector<bool> subnodemark, std::vector<int> submap);

	virtual void cutAndExportByInsideTetMesh(TetVolumetricMesh* inputMesh,const char* filename);

	virtual void initVolumetricMesh();
	
	inline virtual VolumetricmeshType getMeshType() const{ return TET;};

	virtual void initMeshABCD();
	virtual void initElementShapeFunctionDerivate();


	/**
	 * @brief create sub volumtric mesh by elements list
	 *
	 * tetmesh should point to NULL before call this function
	 *
	 * @param[in,out] tetmesh 
	 * @param[in,out] elelist 
	 * @return Put return information here
	 */
	 virtual void createSubTetMesh(TetVolumetricMesh* tetmesh,std::vector<int> elelist,std::vector<int> &nodeindex,std::vector<int> &nodemap);

	//special routine
	virtual void computeDivergenceMatrix(MatrixXd &DIV);
	virtual void computeDivergenceMatrixCurrent(MatrixXd &DIV);



	/* =============================
	query
	=============================*/

	virtual void computeNodeVolume(double *nodevolume);
	virtual void computeCurrentNodeVolume(double *nodevolume);
	virtual double computeCurrentMeshVolume();


	virtual Vector3d computeMassCenter();
	virtual void computeMassCenterDrive(MatrixXd& jacobiMatrix);
	
	TetElement* getTetElement(int index);
	virtual void computeDeformationGradient(int elementid,Matrix3d &output);

	//G is a 9m x 3n sparse matrix 
	virtual void computeDeformationOperator(SparseMatrix<double>* G);
	//V is a 9m x 9m diagnoal matrix  m is number of elements
	virtual void computeElementVolumeMatrix(SparseMatrix<double>* V);

	//to compute a better transformation than modalwarping
	virtual void computeNodeTransformation(VectorXd &linearDisplacement,VectorXd &nonlinearDisplacement,std::vector<Matrix3d> &ourT);

	//geomtry query
	void computeCurrentFaceArea(double *elementarea);
	void computeCurrentFaceNorm(std::vector<Vector3d> &face_norm);
	void computeCurrentNodeNorm(std::vector<Vector3d> &node_norm);

	void computeElementVolume(TetElement* te);
	double computeCurElementVolume(TetElement* te);
	double computeOriElementVolume(TetElement* te);


protected:

	virtual void initElementABCD(TetElement* te);

	void correctElementNodeOrder(TetElement * element);
	void computeElementFaceArea(TetElement * element);
	void computeElementCenter(TetElement* element);

	void computeElementShapeFunctionDerivate(TetElement* te);
	
	void computeTetElementNodeNorm(TetElement* element);
	double computeTetVolumeABS(Vector3d &a,Vector3d &b,Vector3d &c,Vector3d &d);
	double computeTetVolume(Vector3d &a, Vector3d &b, Vector3d &c, Vector3d &d);

};

TetVolumetricMesh* downCastTetVolMesh(LoboVolumetricMesh* mesh);

