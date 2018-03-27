#include "ViewCubeRender.h"
#include "LoboTriMesh/LoboTriMesh.h"
#include "LoboTriMesh/LoboTriMeshRender.h"


ViewCubeRender::ViewCubeRender(const char* filepath)
{
	viewcubeobj = new LoboTriMesh(filepath);
	trimeshrender = new LoboTriMeshRender(viewcubeobj,true);
	trimeshrender->setAcceptShadow(false);
}

ViewCubeRender::~ViewCubeRender()
{
	delete viewcubeobj;
	delete trimeshrender;
}

void ViewCubeRender::drawMesh(QOpenGLShaderProgram *program, QMatrix4x4 m_project, QQuaternion rotation, int width, int height, Vector2d position, double scale)
{
	glDisable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);

	int m_mvMatrixLoc = program->uniformLocation("mvMatrix");
	int m_projMatrixLoc = program->uniformLocation("projMatrix");

	double x = (position.x()- width/2)/(double)(width/2);
	double y = -(position.y() - height / 2) / (double)(height / 2);


	QMatrix4x4 MV, MVP;
	MV.setToIdentity();
	//MV.translate(x*10, y*10, 0);
	MV.translate(0, 0, -20/scale);
	MV.rotate(rotation);

	QMatrix4x4 test;
	test.setToIdentity();
	test.translate(x, y, 0);
	m_project = test*m_project;

	//Vector3d position_3d;
	//projectScreenToWorld(MVP, width, height, position, 10, position_3d);
	//MV = MV.inverted();
	
	program->setUniformValue(m_projMatrixLoc, m_project);
	program->setUniformValue(m_mvMatrixLoc, MV);

	//viewcubeobj->scaleOriginalAtCenter(scale);
	//viewcubeobj->translateMesh(position_3d.data()[0], position_3d.data()[1], position_3d.data()[2]);
	trimeshrender->updateBuffer();
	trimeshrender->renderModel(program);

	glDisable(GL_CULL_FACE);
	glEnable(GL_DEPTH_TEST);
}

void ViewCubeRender::projectScreenToWorld(QMatrix4x4 MVP, int width, int height, Vector2d screenpos, double depth, Vector3d &resultposition)
{
	resultposition.setZero();

	QVector4D vector;
	vector[0] = screenpos.data()[0] * 2.0 / width - 1;
	vector[1] = (height - screenpos.data()[1])*2.0 / height - 1;
	vector[2] = depth;
	vector[3] = 1;

	vector = MVP.inverted()*vector;
	resultposition.data()[0] = vector[0] / vector[3];
	resultposition.data()[1] = vector[1] / vector[3];
	resultposition.data()[2] = vector[2] / vector[3];
}
