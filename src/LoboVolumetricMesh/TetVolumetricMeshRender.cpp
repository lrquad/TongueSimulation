#include "TetVolumetricMeshRender.h"
#include "ColorTable/Colortable.h"

TetVolumetricMeshRender::TetVolumetricMeshRender()
{
}


TetVolumetricMeshRender::~TetVolumetricMeshRender()
{
}

void TetVolumetricMeshRender::renderAllVolumetricMesh(QOpenGLShaderProgram *program, TetVolumetricMesh* volumetricMesh, int colorindex)
{
	int DiffuseProduct_loc = program->uniformLocation("DiffuseProduct");
	int BoolPoint_loc = program->uniformLocation("point");
	
		//use default color
		double defaultcolor[3];
		getColor(colorindex, defaultcolor);

	QVector4D color = QVector4D(defaultcolor[0], defaultcolor[1], defaultcolor[2], 0.1);
	program->setUniformValue(DiffuseProduct_loc, color);

	int num_elements = volumetricMesh->getNumElements();
	
	for (int i = 0; i < num_elements; i++)
	{
		TetElement* element = volumetricMesh->getTetElement(i);
		drawElement(program, volumetricMesh, element);
	}

}

void TetVolumetricMeshRender::renderAllVolumetricMesh(QOpenGLShaderProgram *program, TetVolumetricMesh* volumetricMesh, Vector3d translate, int colorindex /*= 0*/)
{
	int DiffuseProduct_loc = program->uniformLocation("DiffuseProduct");
	int BoolPoint_loc = program->uniformLocation("point");

	//use default color
	double defaultcolor[3];
	getColor(colorindex, defaultcolor);

	QVector4D color = QVector4D(defaultcolor[0], defaultcolor[1], defaultcolor[2], 0.1);
	program->setUniformValue(DiffuseProduct_loc, color);

	int num_elements = volumetricMesh->getNumElements();

	for (int i = 0; i < num_elements; i++)
	{
		TetElement* element = volumetricMesh->getTetElement(i);
		drawElement(program, volumetricMesh, element,translate);
	}
}

void TetVolumetricMeshRender::renderAllVolumetricMesh(QOpenGLShaderProgram *program, TetVolumetricMesh* volumetricMesh, Vector3d translate, double scale, int colorindex /*= 0*/)
{
	int DiffuseProduct_loc = program->uniformLocation("DiffuseProduct");
	int BoolPoint_loc = program->uniformLocation("point");

	//use default color
	double defaultcolor[3];
	getColor(colorindex, defaultcolor);

	QVector4D color = QVector4D(defaultcolor[0], defaultcolor[1], defaultcolor[2], 0.1);
	program->setUniformValue(DiffuseProduct_loc, color);

	int num_elements = volumetricMesh->getNumElements();

	for (int i = 0; i < num_elements; i++)
	{
		TetElement* element = volumetricMesh->getTetElement(i);
		drawElement(program, volumetricMesh, element, translate,scale);
	}
}

void TetVolumetricMeshRender::renderSurfaceVolumstricMesh(QOpenGLShaderProgram *program, TetVolumetricMesh* volumetricMes, int colorindex /*= 0*/)
{
	int DiffuseProduct_loc = program->uniformLocation("DiffuseProduct");
	int BoolPoint_loc = program->uniformLocation("point");

	//use default color
	double defaultcolor[3];
	getColor(colorindex, defaultcolor);

	QVector4D color = QVector4D(defaultcolor[0], defaultcolor[1], defaultcolor[2], 0.1);
	program->setUniformValue(DiffuseProduct_loc, color);

	std::vector<int> eles = volumetricMes->getSurface_ele();

	for (int i = 0; i < eles.size(); i++)
	{
		int eleid = eles[i];
		TetElement* element = volumetricMes->getTetElement(eleid);
		drawElement(program, volumetricMes, element);
	}

}

void TetVolumetricMeshRender::renderRegionNodes(QOpenGLShaderProgram *program, TetVolumetricMesh* volumetricMes, std::vector<int> subsetNodes, int colorindex)
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
		Vector3d position = volumetricMes->getNodeRestPosition(nodeid);
		Vector3d displacement = volumetricMes->getNodeDisplacement(nodeid);
		Vector3d curposition = position + displacement;
		glVertex3f(curposition.x(), curposition.y(), curposition.z());
	}
	glEnd();
	glPointSize(1);
	program->setUniformValue(BoolPoint_loc, 0);

}

void TetVolumetricMeshRender::renderRegionVolumetricMesh(QOpenGLShaderProgram *program, TetVolumetricMesh* volumetricMes, std::vector<int> subsetElement, int colorindex)
{
	int DiffuseProduct_loc = program->uniformLocation("DiffuseProduct");
	int BoolPoint_loc = program->uniformLocation("point");
	
		//use default color
	double defaultcolor[3];
	getColor(colorindex, defaultcolor);

	QVector4D color = QVector4D(defaultcolor[0], defaultcolor[1], defaultcolor[2], 0.5);
	program->setUniformValue(DiffuseProduct_loc, color);

	int num_elements = subsetElement.size();

	for (int i = 0; i < num_elements; i++)
	{
		TetElement* element = volumetricMes->getTetElement(subsetElement[i]);
		drawElement(program, volumetricMes, element);
	}
}

void TetVolumetricMeshRender::renderAllVolumetricMeshWeight(double* weight, QOpenGLShaderProgram *program, TetVolumetricMesh* volumetricMesh, double * color)
{
	
}

void TetVolumetricMeshRender::renderNodeWeightGradient(double* weightGradient, QOpenGLShaderProgram *program, TetVolumetricMesh* volumetricMes)
{
	int DiffuseProduct_loc = program->uniformLocation("DiffuseProduct");
	int BoolPoint_loc = program->uniformLocation("point");
	program->setUniformValue(BoolPoint_loc, 1);

	int numVertex = volumetricMes->getNumVertices();

	for (int i = 0; i < numVertex; i++)
	{
		Vector3d gradient;
		gradient.data()[0] = weightGradient[i * 3 + 0];
		gradient.data()[1] = weightGradient[i * 3 + 1];
		gradient.data()[2] = weightGradient[i * 3 + 2];

		double wi = gradient.norm();
		double defaultcolor[3];
		getColor(0, defaultcolor);
		QVector4D color = QVector4D(defaultcolor[0] * wi, defaultcolor[1] * wi, defaultcolor[2] * wi, 1.0);
		program->setUniformValue(DiffuseProduct_loc, color);
		glPointSize(10);
		glBegin(GL_POINTS);
		Vector3d position = volumetricMes->getNodeRestPosition(i);
		Vector3d displacement = volumetricMes->getNodeDisplacement(i);

		Vector3d curposition = position + displacement;
		glVertex3f(curposition.x(), curposition.y(), curposition.z());
		glEnd();
	}

	glPointSize(1);
	program->setUniformValue(BoolPoint_loc, 0);
}

void TetVolumetricMeshRender::renderNodeWeight(double* weights, QOpenGLShaderProgram *program, TetVolumetricMesh* volumetricMes)
{
	int DiffuseProduct_loc = program->uniformLocation("DiffuseProduct");
	int BoolPoint_loc = program->uniformLocation("point");
	program->setUniformValue(BoolPoint_loc, 1);

	int numVertex = volumetricMes->getNumVertices();
	
	for (int i = 0; i < numVertex; i++)
	{
		double wi = 1.0;
		if (weights!=NULL)
		wi = std::abs(weights[i]);
		double defaultcolor[3];
		getColor(0, defaultcolor);
		QVector4D color = QVector4D(defaultcolor[0] * wi, defaultcolor[1] * wi, defaultcolor[2] * wi, 1.0);
		program->setUniformValue(DiffuseProduct_loc, color);
		glPointSize(10);
		glBegin(GL_POINTS);
		Vector3d position = volumetricMes->getNodeRestPosition(i);
		Vector3d displacement = volumetricMes->getNodeDisplacement(i);

		Vector3d curposition = position + displacement;
		glVertex3f(curposition.x(), curposition.y(), curposition.z());
		glEnd();
	}
	
	glPointSize(1);
	program->setUniformValue(BoolPoint_loc, 0);
}

void TetVolumetricMeshRender::drawElement(QOpenGLShaderProgram *program, TetVolumetricMesh* volumetricMes, TetElement* element)
{
	glBegin(GL_TRIANGLES);
	for (int i = 0; i < 4; i++)
	{
		int n1i = element->node_indices[element->face_indices[i * 3 + 0]];
		int n2i = element->node_indices[element->face_indices[i * 3 + 1]];
		int n3i = element->node_indices[element->face_indices[i * 3 + 2]];

		Vector3d n1v = volumetricMes->getNodePosition(n1i);
		Vector3d n2v = volumetricMes->getNodePosition(n2i);
		Vector3d n3v = volumetricMes->getNodePosition(n3i);

		glVertex3f(n1v.x(), n1v.y(), n1v.z());
		glVertex3f(n2v.x(), n2v.y(), n2v.z());
		glVertex3f(n3v.x(), n3v.y(), n3v.z());
	}
	glEnd();
}

void TetVolumetricMeshRender::drawElement(QOpenGLShaderProgram *program, TetVolumetricMesh* volumetricMes, TetElement* element, Vector3d translate)
{
	glBegin(GL_TRIANGLES);
	for (int i = 0; i < 4; i++)
	{
		int n1i = element->node_indices[element->face_indices[i * 3 + 0]];
		int n2i = element->node_indices[element->face_indices[i * 3 + 1]];
		int n3i = element->node_indices[element->face_indices[i * 3 + 2]];

		Vector3d n1v = volumetricMes->getNodePosition(n1i) + translate;
		Vector3d n2v = volumetricMes->getNodePosition(n2i) + translate;
		Vector3d n3v = volumetricMes->getNodePosition(n3i) + translate;

		glVertex3f(n1v.x(), n1v.y(), n1v.z());
		glVertex3f(n2v.x(), n2v.y(), n2v.z());
		glVertex3f(n3v.x(), n3v.y(), n3v.z());
	}
	glEnd();
}

void TetVolumetricMeshRender::drawElement(QOpenGLShaderProgram *program, TetVolumetricMesh* volumetricMes, TetElement* element, Vector3d translate, double scale)
{
	glBegin(GL_TRIANGLES);
	for (int i = 0; i < 4; i++)
	{
		int n1i = element->node_indices[element->face_indices[i * 3 + 0]];
		int n2i = element->node_indices[element->face_indices[i * 3 + 1]];
		int n3i = element->node_indices[element->face_indices[i * 3 + 2]];

		Vector3d n1v = volumetricMes->getNodePosition(n1i) + translate;
		Vector3d n2v = volumetricMes->getNodePosition(n2i) + translate;
		Vector3d n3v = volumetricMes->getNodePosition(n3i) + translate;

		n1v *= scale;
		n2v *= scale;
		n3v *= scale;


		glVertex3f(n1v.x(), n1v.y(), n1v.z());
		glVertex3f(n2v.x(), n2v.y(), n2v.z());
		glVertex3f(n3v.x(), n3v.y(), n3v.z());
	}
	glEnd();
}
