#pragma once
#include <QPainter>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include "LoboVolumetricMesh/LoboVolumetricMesh.h"
#include "LoboElement.h"

class VolumetricMeshRender
{
public:
	VolumetricMeshRender();
	~VolumetricMeshRender();

	virtual void renderVolumetricMesh(QOpenGLShaderProgram *program, LoboVolumetricMesh* volumetricMesh);
	virtual void paintSimulator(QPainter &painter, LoboVolumetricMesh* volumetricMesh);

	inline void setOnlySurface(bool b){ only_render_surface = b; }
	inline bool isOnlySurface(){ return only_render_surface;}

	virtual void renderRegionNodes(QOpenGLShaderProgram *program, LoboVolumetricMesh* volumetricMes, std::vector<int> subsetNodes, int colorindex = 0);

protected:
	

	virtual void renderAllVolumetricMesh(QOpenGLShaderProgram *program, LoboVolumetricMesh* volumetricMes);
	virtual void renderSurfaceMesh(QOpenGLShaderProgram *program, LoboVolumetricMesh* volumetricMes);
	virtual void drawElement(QOpenGLShaderProgram *program, LoboElement* element);

	bool only_render_surface;


};

