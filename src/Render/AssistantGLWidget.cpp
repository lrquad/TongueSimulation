#include "AssistantGLWidget.h"
#include <QOpenGLShaderProgram>
#include <QOpenGLFunctions>
#include <QCoreApplication>
#include <QtWidgets/QApplication>
#include <QtWidgets/qdesktopwidget.h>

AssistantGLWidget::AssistantGLWidget(QWidget* parent /*= 0*/) :GLWidget(parent)
{
	//no need init Scene;
	scene = NULL;
	scene_render = NULL;
	mouseprojection = NULL;


}

AssistantGLWidget::~AssistantGLWidget()
{

}

void AssistantGLWidget::createNewSceneRender(MyScene* input_scene)
{
	delete scene_render;

	if (input_scene!=NULL)
	scene_render = new LoboSceneRender(input_scene);
}

void AssistantGLWidget::initializeGL()
{
	initializeOpenGLFunctions();

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	glEnable(GL_MULTISAMPLE);

	glClearColor(0.9, 0.9, 0.9, 0.9);

	//init shader program

	initShaders();
	initShadowMap();
	// Enable depth buffer
	glEnable(GL_DEPTH_TEST);

	// Enable back face culling
	glEnable(GL_CULL_FACE);

	vao_ = new QOpenGLVertexArrayObject(this);

	camera.init();
	camera.camera_position = QVector3D(0, 0, -10);
	camera.update();
	loadCameraInfo("config/cameraSensor.txt");

	program.setUniformValue(m_lightPosLoc, QVector3D(0, 30, 0));
	shadowmap.light_pos = QVector3D(0, 30, 0);
	program.release();

	t1 = 0;

	connect(&timer, SIGNAL(timeout()), this, SLOT(repaint()));

	timer.setInterval(timer_spf);
	timer.start();
}

void AssistantGLWidget::paintGL()
{
	if (!this->isVisible())
	{
		return;
	}

	painter.begin(this);
	painter.beginNativePainting();

	glEnable(GL_MULTISAMPLE);

	glDisable(GL_CULL_FACE);

	glEnable(GL_DEPTH_TEST);

	
	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);
	glEnable(GL_BLEND); //Enable blending.
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);


	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	program.bind();

	QMatrix4x4 m_world;
	m_world.setToIdentity();

	program.setUniformValue(m_projMatrixLoc, m_proj);
	program.setUniformValue(m_mvMatrixLoc, camera.m_camera * m_world);
	QMatrix3x3 normalMatrix = m_world.normalMatrix();
	program.setUniformValue(m_normalMatrixLoc, normalMatrix);

	//loboScene->drawScene(&program);

	if (scene_render != NULL)
	{
		scene_render->setIsAssistantView(true);
		scene_render->drawScene(&program);
		scene_render->setIsAssistantView(false);
	}
	program.release();

	glDisable(GL_DEPTH_TEST);
	glDisable(GL_BLEND);
	glDisable(GL_CULL_FACE);
	glDisable(GL_BLEND);

	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	painter.endNativePainting();

	painter.setRenderHints(QPainter::Antialiasing, true);
	QPen pen(QColor(255, 100, 0, 150), 4);
	painter.setPen(pen);
	painter.drawRect(0, 0, width(), height());

	painter.end();
}

void AssistantGLWidget::mousePressEvent(QMouseEvent *event) 
{

}

void AssistantGLWidget::mouseMoveEvent(QMouseEvent *event) 
{

}

void AssistantGLWidget::mouseReleaseEvent(QMouseEvent *event) 
{

}

void AssistantGLWidget::keyPressEvent(QKeyEvent *event) 
{

}

void AssistantGLWidget::keyReleaseEvent(QKeyEvent *event) 
{

}
