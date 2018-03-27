#include "VolumetricMeshRender.h"
#include "TetElement.h"
#include "TetVolumetricMeshRender.h"
#include "ColorTable/Colortable.h"
#include <iostream>

VolumetricMeshRender::VolumetricMeshRender()
{
}


VolumetricMeshRender::~VolumetricMeshRender()
{
}

void VolumetricMeshRender::renderVolumetricMesh(QOpenGLShaderProgram *program, LoboVolumetricMesh* volumetricMesh)
{
	if (volumetricMesh == NULL)
	{
		return;
	}

	if (volumetricMesh->getSurfaceReady())
	{
		//render surface only
		renderSurfaceMesh(program, volumetricMesh);
	}
	else
	{
		//render all element(including inside elements)
		renderAllVolumetricMesh(program, volumetricMesh);
	}

}

void VolumetricMeshRender::paintSimulator(QPainter &painter, LoboVolumetricMesh* volumetricMesh)
{

}

void VolumetricMeshRender::renderRegionNodes(QOpenGLShaderProgram *program, LoboVolumetricMesh* volumetricMes, std::vector<int> subsetNodes, int colorindex /*= 0*/)
{
	int DiffuseProduct_loc = program->uniformLocation("DiffuseProduct");
	int BoolPoint_loc = program->uniformLocation("point");
	program->setUniformValue(BoolPoint_loc, 1);

	//use default color
	double defaultcolor[3];
	getColor(colorindex, defaultcolor);

	QVector4D color = QVector4D(defaultcolor[0], defaultcolor[1], defaultcolor[2], 0.5);
	program->setUniformValue(DiffuseProduct_loc, color);

	glPointSize(20);
	glBegin(GL_POINTS);
	for (int i = 0; i < subsetNodes.size(); i++)
	{
		int nodeid = subsetNodes[i];
		if (nodeid >= volumetricMes->getNumVertices()||nodeid<0)
		{
			break;
		}
		Vector3d position = volumetricMes->getNodeRestPosition(nodeid);
		Vector3d displacement = volumetricMes->getNodeDisplacement(nodeid);
		Vector3d curposition = position + displacement;
		glVertex3f(curposition.x(), curposition.y(), curposition.z());
	}
	glEnd();
	glPointSize(1);
	program->setUniformValue(BoolPoint_loc, 0);

}

void VolumetricMeshRender::renderAllVolumetricMesh(QOpenGLShaderProgram *program, LoboVolumetricMesh* volumetricMes)
{
	if (volumetricMes->getMeshType() == LoboVolumetricMesh::VolumetricmeshType::TET)
	{
		TetVolumetricMeshRender render;

		render.renderAllVolumetricMesh(program, downCastTetVolMesh(volumetricMes));
	}
}


void VolumetricMeshRender::renderSurfaceMesh(QOpenGLShaderProgram *program, LoboVolumetricMesh* volumetricMes)
{

}

void VolumetricMeshRender::drawElement(QOpenGLShaderProgram *program, LoboElement* element)
{
	

}