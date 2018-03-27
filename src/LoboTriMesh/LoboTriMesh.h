#pragma once
#include "ObjLoader/tiny_obj_loader.h"
#include <Eigen/Dense>
using namespace Eigen;

class LoboObjMesh;

class LoboTriMesh
{
public:
	LoboTriMesh();
	/**
	 * @brief read from file
	 *
	 * @param[in,out] objfilename Put argument desc here
	 * @return Put return information here
	 */
	 LoboTriMesh(const char* objfilename);

	 LoboTriMesh(const char* objfilename,bool centered);


	/**
	 * @brief can created from old class
	 *
	 * @param[in,out] objmesh Put argument desc here
	 * @return Put return information here
	 */
	 LoboTriMesh(LoboObjMesh* objmesh);

	~LoboTriMesh();

	/* ===========================================
	Obj query and control
	=========================================== */
	
	/**
	 * @brief this function will reset the original position of mesh to current position.
	 * Thus will release info about the position from file.
	 */
	void finishEditMesh();

	void translateMesh(double x, double y, double z);
	void translateMesh(Vector3d translate);
	void transformMesh(Vector3d translate, Matrix3d rotation);
	void moveMesh(Vector3d speed);
	void updateTranslate();
	void scaleMeshAroundMeshcenter(double scale);

	void shapeMatching(LoboTriMesh* targettrimesh);

	/**
	 * @brief scale the centered mesh then store in original 
	 */
	 void scaleOriginalAtCenter(double scale);

	 void scaleAtCurrent(double scale);
	 void scaleAtCurrentUpdate(double scale);

	 void transformAtcurrentUpdate(Matrix3d transform,Vector3d translate);

	const char* getMeshName(){ return objFilename.c_str();}

	Eigen::Vector3d getTranslateFromOri() const { return translateFromOri; }
	void setTranslateFromOri(Eigen::Vector3d val) { translateFromOri = val; }
	Eigen::Vector3d getOriginalToCenter() const { return originalToCenter; }
	void setOriginalToCenter(Eigen::Vector3d val) { originalToCenter = val; }

	double getScaleFromCenter() const { return scaleFromCenter; }
	void setScaleFromCenter(double val) { scaleFromCenter = val; }

	int getNumShapes(){ return numShapes; };
	int getNumVertex();

	tinyobj::attrib_t* getAttributeRef();
	std::vector<tinyobj::shape_t>* getShapesRef();
	std::vector<tinyobj::material_t>* getMaterialRef();

	void updateTriDisplacement(double * displacement);

	Vector3d getOriPosition(int vertexid);
	Vector3d getCurPosition(int vertexid);
	VectorXf getCurrentPositionFloat();
	VectorXd getCurrentPositionDouble();
	VectorXi getIndices();

	void exportMTL(std::ofstream &outputStream);
	void exportObj(std::ofstream &outputstream, int &vertexOffset, int &texCoordOffset);
	void exportObj(const char* filename);

	std::vector<std::vector<int>>* getMapFromTinyObjShape();

	int getNumMeshVertes() const { return numMeshVertes; }
	void setNumMeshVertes(int val) { numMeshVertes = val; }

	
	void setVertexColorData3D(double * weights);
	void setVertexTexCoor(double *coordinate);
	float* getVertexColorData();

	Eigen::Matrix4d getAffineTransform() const { return affineTransform; }
	void setAffineTransform(Eigen::Matrix4d val) { affineTransform = val; }

	bool getMeshUpdated() const { return meshUpdated; }
	void setMeshUpdated(bool val) { meshUpdated = val; }

	int collision_id_;
	int tri_id;

	VectorXd vertexData;

protected:

	Matrix4d affineTransform;

	/**
	 * @brief call this function when orishapechanged
	 */
	void updateEigenVector();
	void initEigenVector();
	void initShapeInfo();
	void initConstructor();

	bool readObjModel(const char* filename, bool centered = true);
	void readObjVertexOnly(const char* filename);
	
	int numShapes;
	int numVertex;
	int numFaces;

	VectorXd globalDisPlacement;
	VectorXd globalOriPosition;
	VectorXi globalIndices;

	//use to render vertex data info, for example vertex weight.
	VectorXf globalVertexColor;
	

	std::string mtlFilename;
	std::string objFilename;

	std::vector<tinyobj::shape_t> meshShapes;

	tinyobj::attrib_t currentVertexAttribute;
	tinyobj::attrib_t originalVertexAttribute;
	tinyobj::attrib_t centeredAttribute;

	std::vector<tinyobj::material_t> materials;
	std::vector<Vector3f> mesh_vertex;
	int numMeshVertes;
	std::vector<std::vector<int>> mapFromTinyObjShape;

	Vector3d translateFromOri;
	Vector3d originalToCenter;
	double scaleFromCenter;
	bool meshUpdated;


};

