#pragma once
#include "LoboVolumetricMesh.h"

class CubeElement;
class LoboNodeBase;
class LoboElement;

class CubeVolumtricMesh:public LoboVolumetricMesh
{
public:
	CubeVolumtricMesh();
	~CubeVolumtricMesh();

	virtual void readElementList();
	virtual void readElementMesh(const char* filenamebase, double* translate = NULL, double scale = 1.0);
	virtual void saveElementMesh(std::ofstream &outstream);

	virtual void saveElementMeshAscii(const char* filenamebase, std::vector<LoboNodeBase*> &newNodeList, std::vector<LoboElement*> &newElementList);

	virtual void initVolumetricMesh();

	virtual void exportNewCubeVolumtricMeshByNode(const char* filenamebase);

	inline virtual VolumetricmeshType getMeshType() const{ return CUBE; };
	CubeElement* getCubeElement(int index);


	virtual void protrudingCube(int selectedCube,int type);


	/* =============================
	control
	=============================*/

	virtual void searchNeighborElement();
	virtual bool containsVertex(int element, const Vector3d &pos) const;
	virtual void computeMassMatrix(std::vector<LoboElement*>* elementlist, SparseMatrix<double>* mass) const;
	virtual void computeMassMatrix(SparseMatrix<double>* mass);
	virtual void computeBarycentricWeights(int elementid, const Vector3d &pos, Vector4d &weight) const;

	// this will compute surface info save to internal buffer
	virtual void computeSurface();
	virtual void releaseSurface();

	virtual void saveSurfaceToObjAscii(const char* filename) const ;
	virtual void computeNodeVolume(double *nodeweight);
	virtual double computeCurrentMeshVolume() ;
	virtual void computeDeformationGradient(int elementid, Matrix3d &output);

protected:

	void computeElementAttri(CubeElement* element);
	//************************************
	// Method:    computeNeighborDirection
	// FullName:  CubeVolumtricMesh::computeNeighborDirection
	// Access:    protected 
	// Returns:   the direction describes the position of centerj refer to centeri
	// Qualifier:
	// Parameter: Vector3d centeri
	// Parameter: Vector3d centerj
	//************************************
	int computeNeighborDirection(Vector3d centeri, Vector3d centerj);
	bool mergeNodePosition(Vector3d centeri, Vector3d centerj, double edge);

	int cubeNodeOrderMap[8];


};

CubeVolumtricMesh* downCastCubeVolMesh(LoboVolumetricMesh* mesh);

