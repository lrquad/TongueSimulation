#pragma once
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>
#include <QOpenGLTexture>
#include <QOpenGLVertexArrayObject>
#include "ObjLoader/tiny_obj_loader.h"
#include "LoboFunctions.h"

struct VertexData
{
	QVector3D postion;
	QVector2D texCoord;
};

struct RenderBufferObj
{
	QOpenGLTexture *texture;
	QOpenGLBuffer *arrayBuf;
	QOpenGLBuffer *textureBuf;
	QOpenGLBuffer *indexBuf;
	int material_id;
	bool bool_texture;
};

class LoboObjMesh: protected QOpenGLFunctions
{
public:
	LoboObjMesh();
	LoboObjMesh(const char* filename);
	~LoboObjMesh();

	/* ===========================================
	Obj query and control
	=========================================== */
	const char* getMeshName();

	bool readObjModel(const char* filename,bool centered = true);
	void translateObjModel(double x,double y,double z);
	void scaleObjModel(double scale);
	void scaleObjAtCenter(double scale);

	
	/**
	 * @brief Draw this model
	 *
	 * This class need to refactor in the future.
	 *
	 * @param[in,out] program Put argument desc here
	 * @param[in,out] alpha Put argument desc here
	 * @return Put return information here
	 */
	 void drawModel(QOpenGLShaderProgram *program, double alpha =1.0);
	
	void updateBuffer();

	void setDiffuseColor(double R,double G,double B);

	void objUniformMotion(Vector3d step_moving);
	void updateDisplacement(const double* displacement);
	void initCollisionShape();
	void updateCollisionShape();
	void updateAABBtree();
	void selfCollisionDetection();

	void saveShape(std::ofstream &outStream,int &nodeoffset,int &texoffset,bool matchfunnel = false);
	void saveDisplacement(int totalcols, int cols, const char* filename,bool initMatrix);

	// collision
	inline void setCollisionFlag(bool b){ collision_flag_ = b; };
	inline bool getCollisionFlag(){ return collision_flag_; };
	inline void setSelfCollisionFlag(bool b){ selfcollision_flag_ = b; };
	inline bool getSelfCollisionFlag(){ return selfcollision_flag_; };
	inline void setOutputFlag(bool b){ output_flat_ = b; };
	inline bool getOutputFlag() { return output_flat_; };

	inline void setTexture(bool b) {texture_flag_ = b;}
	inline void setAABBupdateFlag(bool b){ aabbupdate_flag_ = b; };
	inline bool getAABBupdateFlag(){ return aabbupdate_flag_; };
	inline void setCollisionRender(bool b){ this->collision_render_ = b; };
	inline void setDeformFlag(bool b){deform_flag_ = b; };
	inline void setMovingFlag(bool b){ this->moving_obj_ = b; };
	inline bool getMovingFlag(){ return moving_obj_; };

	lobo::Tri_Mesh* getTri_Mesh(){return &tri_mesh;}
	
	double funnel_radius;

	int getNumberofVertex() const { return displacement.rows()/3;}
	Vector3d getOriPosition(int positionid);

	double getAlpha() const { return alpha_; }
	void setAlpha(double val) { alpha_ = val; }



protected:

	void init();
	void initShapes();
	void initTexture();
	void initDisplacement();

	void printInfo(const std::vector<tinyobj::shape_t>& shapes, const std::vector<tinyobj::material_t>& materials, bool triangulate = true);
	void drawShape(QOpenGLShaderProgram *program, RenderBufferObj &renderobj, double alpha=1.0);

	int AmbientProduct_loc;
	int DiffuseProduct_loc;
	int SpecularProduct_loc;
	int Shininess_loc;

	double alpha_;

	std::string mtl_filename;
	std::string obj_filename;
	lobo::Tri_Mesh tri_mesh;

	std::vector<tinyobj::shape_t> shapes;
	std::vector<tinyobj::shape_t> ori_shapes;
	std::vector<tinyobj::shape_t> centered_shapes;

	std::vector<tinyobj::material_t> materials;
	std::vector<RenderBufferObj> shape_render;

	ozcollide::Vector<ozcollide::Polygon> tris;
	ozcollide::Vector<ozcollide::Vec3f> verts;

	//for output 
	MatrixXd displacement_list;

	//model control
	Vector3d modeltranslate;
	Vector3d translate;
	double scale;
	VectorXd displacement;
	VectorXd oriposition;
	VectorXi indices;
	

	bool texture_flag_;
	bool collision_flag_;
	bool selfcollision_flag_;
	bool aabbupdate_flag_;
	bool deform_flag_;
	bool collision_render_;
	bool output_flat_;
	bool moving_obj_;
};

