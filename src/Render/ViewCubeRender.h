#pragma once
#include <iostream>
#include <Eigen/dense>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>

using namespace Eigen;

class LoboTriMesh;
class LoboTriMeshRender;

class ViewCubeRender
{
public:
	ViewCubeRender(const char* filepath);
	~ViewCubeRender();

	virtual void drawMesh(QOpenGLShaderProgram *program, QMatrix4x4 m_project, QQuaternion rotation,int width,int height, Vector2d position, double scale);

protected:

	void projectScreenToWorld(QMatrix4x4 MVP, int width, int height, Vector2d screenpos, double depth, Vector3d &resultposition);

	double scale;
	Vector3d position;
	LoboTriMesh* viewcubeobj;
	LoboTriMeshRender* trimeshrender;
};

