#pragma once
#include <iostream>
#include <Eigen/dense>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>

using namespace Eigen;

class LoboTriMesh;
class LoboTriMeshRender;

class SphereRender
{
public:
	SphereRender(const char* filepath,bool smoothing = true);
	~SphereRender();

	virtual void drawMesh(QOpenGLShaderProgram *program);
	virtual void drawMesh(QOpenGLShaderProgram *program,Vector3d position,double scale,int colorindex = 0);
	virtual void drawMesh(QOpenGLShaderProgram *program, Vector3d position, Matrix3d rotation, double scale, int colorindex = 0);
	virtual void drawMeshScaleGlobal(QOpenGLShaderProgram *program, Vector3d position, Matrix3d rotation, double scale, int colorindex = 0);
	virtual void updateMeshBuffer(Vector3d position, Matrix3d rotation, double scale);

	virtual void exportObj(const char* filename);

protected:

	double scale;
	Vector3d position;
	LoboTriMesh* spheretriobj;
	LoboTriMeshRender* trimeshrender;
};

