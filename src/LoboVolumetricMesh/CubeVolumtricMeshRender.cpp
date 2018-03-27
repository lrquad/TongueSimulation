#include "CubeVolumtricMeshRender.h"
#include "ColorTable/Colortable.h"


CubeVolumtricMeshRender::CubeVolumtricMeshRender()
{
}


CubeVolumtricMeshRender::~CubeVolumtricMeshRender()
{
}

void CubeVolumtricMeshRender::renderAllVolumtricMesh(QOpenGLShaderProgram *program, CubeVolumtricMesh* volumetricMes, Vector3d translate, double scale /*= 1*/, int colorindex /*= 0*/)
{
	int DiffuseProduct_loc = program->uniformLocation("DiffuseProduct");
	int BoolPoint_loc = program->uniformLocation("point");
	int AmbientProduct_loc = program->uniformLocation("AmbientProduct");
	int SpecularProduct_loc = program->uniformLocation("SpecularProduct");

	//use default color
	double defaultcolor[3];
	getColor(colorindex, defaultcolor);

	QVector4D color = QVector4D(defaultcolor[0], defaultcolor[1], defaultcolor[2], 0.4);

	program->setUniformValue(DiffuseProduct_loc, color);
	program->setUniformValue(AmbientProduct_loc, color);
	program->setUniformValue(SpecularProduct_loc, QVector4D(0.1,0.1,0.1, 0.4));



	int num_elements = volumetricMes->getNumElements();

	for (int i = 0; i < num_elements; i++)
	{
		CubeElement* element = volumetricMes->getCubeElement(i);
		drawElement(program, volumetricMes, element, translate,scale);
	}

}

void CubeVolumtricMeshRender::drawElement(QOpenGLShaderProgram *program, CubeVolumtricMesh* volumetricMes, CubeElement* element, Vector3d translate, double scale /*= 1*/)
{
	glBegin(GL_QUADS);
	for (int i = 0; i < 6; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			int nindex = element->node_indices[element->face_indices[i * 4 + j]];
			Vector3d nv = volumetricMes->getNodePosition(nindex) + translate;
			nv *= scale;
			glVertex3f(nv.x(), nv.y(), nv.z());
		}
	}
	glEnd();
}
