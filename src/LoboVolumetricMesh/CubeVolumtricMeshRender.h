#pragma once
#include "CubeVolumtricMesh.h"
#include <QPainter>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include "CubeElement.h"

class CubeVolumtricMeshRender
{
public:
	CubeVolumtricMeshRender();
	~CubeVolumtricMeshRender();

	virtual void renderAllVolumtricMesh(QOpenGLShaderProgram *program, CubeVolumtricMesh* volumetricMes, Vector3d translate, double scale = 1, int colorindex = 0);

protected:

	virtual void drawElement(QOpenGLShaderProgram *program, CubeVolumtricMesh* volumetricMes, CubeElement* element, Vector3d translate, double scale = 1);
};

