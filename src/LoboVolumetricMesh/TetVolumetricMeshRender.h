#pragma once
#include "TetVolumetricMesh.h"
#include <QPainter>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include "TetElement.h"
class WeightFunction;
class TetVolumetricMeshRender
{
public:
	TetVolumetricMeshRender();
	~TetVolumetricMeshRender();

	virtual void renderAllVolumetricMesh(QOpenGLShaderProgram *program, TetVolumetricMesh* volumetricMes,int colorindex = 0);
	virtual void renderAllVolumetricMesh(QOpenGLShaderProgram *program, TetVolumetricMesh* volumetricMes,Vector3d translate, int colorindex = 0);
	virtual void renderAllVolumetricMesh(QOpenGLShaderProgram *program, TetVolumetricMesh* volumetricMes, Vector3d translate,double scale, int colorindex = 0);

	virtual void renderSurfaceVolumstricMesh(QOpenGLShaderProgram *program, TetVolumetricMesh* volumetricMes, int colorindex = 0);

	virtual void renderRegionVolumetricMesh(QOpenGLShaderProgram *program, TetVolumetricMesh* volumetricMes, std::vector<int> subsetElement, int colorindex = 0);
	virtual void renderRegionNodes(QOpenGLShaderProgram *program, TetVolumetricMesh* volumetricMes, std::vector<int> subsetNodes,int colorindex = 0);
	
	virtual void renderAllVolumetricMeshWeight(double* weight, QOpenGLShaderProgram *program, TetVolumetricMesh* volumetricMes, double * color = NULL);

	virtual void renderNodeWeight(double* weights, QOpenGLShaderProgram *program, TetVolumetricMesh* volumetricMes);
	virtual void renderNodeWeightGradient(double* weightGradient, QOpenGLShaderProgram *program, TetVolumetricMesh* volumetricMes);

protected:

	virtual void drawElement(QOpenGLShaderProgram *program, TetVolumetricMesh* volumetricMes, TetElement* element);
	virtual void drawElement(QOpenGLShaderProgram *program, TetVolumetricMesh* volumetricMes, TetElement* element,Vector3d translate);
	virtual void drawElement(QOpenGLShaderProgram *program, TetVolumetricMesh* volumetricMes, TetElement* element, Vector3d translate,double scale);

};

