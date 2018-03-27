#pragma once
#include <QPainter>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLTexture>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>
#include "ObjLoader/tiny_obj_loader.h"

struct RenderBufferShape
{
	QOpenGLTexture *texture;
	QOpenGLBuffer *arrayBuf;
	QOpenGLBuffer *textureBuf;
	QOpenGLBuffer *indexBuf;
	QOpenGLBuffer *normlBuf;
	QOpenGLBuffer *vertexColorBuf;

	std::vector<float> vb;

	int sizePerVertex;
	int material_id;
	bool bool_texture;
};

class SphereRender;
class LoboTriMesh;
class TriMeshCollisionDetector;

class LoboTriMeshRender : protected QOpenGLFunctions
{
public:
	
	LoboTriMeshRender(LoboTriMesh* trimesh);
	LoboTriMeshRender(LoboTriMesh* trimesh, bool isspecial);

	~LoboTriMeshRender();

	virtual void renderModel(QOpenGLShaderProgram *program);

	void renderCollisionInfo(QOpenGLShaderProgram *program);

	virtual void updateBuffer();
	void setDiffuesColor(double R, double G, double B);

	virtual void registerCollisionDetection(TriMeshCollisionDetector* collisiondetection);

	double getAlphaChannel() const { return alphaChannel; }
	void setAlphaChannel(double val) { alphaChannel = val; }

	bool getUseTexutre() const { return useTexutre; }
	void setUseTexutre(bool val) { useTexutre = val; }
	bool getRenderCollision() const { return ifrenderCollision; }
	void setRenderCollision(bool val);
	bool getSpecial_mesh() const { return special_mesh; }
	void setSpecial_mesh(bool val) { special_mesh = val; }
	bool getSmoothshading() const { return smoothshading; }
	void setSmoothshading(bool val) { smoothshading = val; }

	bool getVisible() const { return visible; }
	void setVisible(bool val) { visible = val; }
	bool getIsTransparent() const { return isTransparent; }
	void setIsTransparent(bool val) { isTransparent = val; }
	bool getRenderVertexColor() const { return renderVertexColor; }
	void setRenderVertexColor(bool val) { renderVertexColor = val; }
	bool getAcceptShadow() const { return acceptShadow; }
	void setAcceptShadow(bool val) { acceptShadow = val; }
	bool getAcceptwireframe() const { return acceptwireframe; }
	void setAcceptwireframe(bool val) { acceptwireframe = val; }
protected:
	
	void updateNorms();
	virtual void drawShape(QOpenGLShaderProgram *program, RenderBufferShape &shapebuffer, double alpha = 1.0);
	void drawShapeFlat(QOpenGLShaderProgram *program,tinyobj::shape_t* shape);

	virtual void initShapeBuffer();

	virtual void updateShapeArrayBuffer(int i, int sizePerVertex);

	void initTexture();
	void initRender();

	int AmbientProduct_loc;
	int DiffuseProduct_loc;
	int SpecularProduct_loc;
	int Shininess_loc;

	std::vector<tinyobj::material_t>* materials;
	std::vector<tinyobj::shape_t>* shapes;
	std::vector<RenderBufferShape> shapeBuffer;
	tinyobj::attrib_t* tri_attributes;

	QOpenGLBuffer *arrayBuf;
	QOpenGLBuffer *textureBuf;
	QOpenGLBuffer *normlBuf;


	TriMeshCollisionDetector* collisiondetection;

	double alphaChannel;
	LoboTriMesh* trimesh;
	
	//render sphere
	SphereRender* sphere1;

	bool useTexutre;
	bool ifrenderCollision;
	
	bool special_mesh;
	bool smoothshading;
	bool visible;
	bool isTransparent;
	bool renderVertexColor;
	bool acceptShadow;
	bool acceptwireframe;
};

