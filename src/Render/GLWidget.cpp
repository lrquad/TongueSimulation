#include "GLWidget.h"
#include <QMouseEvent>
#include <QOpenGLShaderProgram>
#include <QCoreApplication>
#include <math.h>
#include <iostream>
#include <QtWidgets/QApplication>
#include <QtWidgets/qdesktopwidget.h>
#include <QScreen>
#include "Render\MouseProjection.h"
#include <QWindow>
#include "ViewCubeRender.h"

GLWidget::GLWidget(QWidget *parent)
	: QOpenGLWidget(parent),
	timer_spf(0),
	loboScene(0),
	startsimulator(0),
	fps_(0),
	t1(0),
	scene(0),
	scene_render(0)
{
	swtichPaint = false;
	swtichRenderViewCube = false;
	useCameraLight = false;
	useTexture = true;
	useShadow = true;
	useVertexColor = false;
	renderWireFrame = true;
	useDeepWarping = false;

	setSwtichSaveAnimation(false);
	setFocus();
	
}

GLWidget::~GLWidget()
{
	cleanup();
}

static void qNormalizeAngle(int &angle)
{
	while (angle < 0)
		angle += 360 * 16;
	while (angle > 360 * 16)
		angle -= 360 * 16;
}

void GLWidget::cleanup()
{
	makeCurrent();
	delete loboScene;
	delete scene_render;
	delete scene;
	delete mouseprojection;
	doneCurrent();
}


void GLWidget::timerEvent(QTimerEvent *e)
{
	//not use now
}

void GLWidget::focusOutEvent(QFocusEvent * event)
{
	camera.resetKey();
}

void GLWidget::saveAnimation()
{
	std::ostringstream frame;
	frame << "results/screenshot_" << scene->getSimulation_step() << ".png";
	std::string framefilename = frame.str();
	QScreen *QSCREEN = QGuiApplication::primaryScreen();
	QPixmap originalPixmap;
	QPoint pos = this->parentWidget()->mapToGlobal(this->pos());
	QWidget* root = this->parentWidget()->parentWidget();
	pos = root->mapToGlobal(this->parentWidget()->pos());

	originalPixmap = QSCREEN->grabWindow(QApplication::desktop()->winId(),
		pos.x(), pos.y(),
		this->parentWidget()->width(), this->parentWidget()->height());
	originalPixmap.save(framefilename.c_str(), "PNG", 100);
}

void GLWidget::saveAnimationWithUi()
{
	std::ostringstream frame;
	frame << "results/screenshot_" << scene->getSimulation_step() << ".png";
	std::string framefilename = frame.str();
	QScreen *QSCREEN = QGuiApplication::primaryScreen();
	QPixmap originalPixmap;

	QPoint pos = parentWidget()->parentWidget()->pos();
	QWidget* root = this->parentWidget()->parentWidget();
	pos = root->pos();

	originalPixmap = QSCREEN->grabWindow(QApplication::desktop()->winId(),
		pos.x() + 10, pos.y(),
		root->width() - 2, root->height() + 30);

	originalPixmap.save(framefilename.c_str(), "PNG", 100);
}

void GLWidget::startSimulate()
{
	loboScene->config.b_start_simulate = true;
	scene->setStartSimulation(true);
}

void GLWidget::stopSimulate()
{
	loboScene->config.b_start_simulate = false;
	scene->setStartSimulation(false);
}

void GLWidget::toggelBlend()
{
	renderconfig.is_blend = !renderconfig.is_blend;
	scene_render->setIfBlend(renderconfig.is_blend);
}

void GLWidget::saveScreenShot()
{
	std::ostringstream frame;
	frame << "results/screenshot_single_"<<scene->getSimulation_step()<< ".png";
	std::string framefilename = frame.str();
	QScreen *QSCREEN = QGuiApplication::primaryScreen();
	QPixmap originalPixmap;
	QPoint pos = this->parentWidget()->mapToGlobal(this->pos());

	QWidget* root = this->parentWidget()->parentWidget();
	pos = root->mapToGlobal(this->parentWidget()->pos());

	originalPixmap = QSCREEN->grabWindow(QApplication::desktop()->winId(),
		pos.x(), pos.y(),
		this->parentWidget()->width(), this->parentWidget()->height());
	originalPixmap.save(framefilename.c_str(), "PNG", 100);
}

void GLWidget::saveScreenShotUi()
{
	std::ostringstream frame;
	frame << "results/screenshotUi_temp" << ".jpg";
	std::string framefilename = frame.str();
	QScreen *QSCREEN = QGuiApplication::primaryScreen();
	QPixmap originalPixmap;

	QPoint pos = parentWidget()->parentWidget()->pos();
	pos = QApplication::activeWindow()->pos();

	originalPixmap = QSCREEN->grabWindow(QApplication::desktop()->winId(),
		pos.x()+10, pos.y(),
		QApplication::activeWindow()->width()-2, QApplication::activeWindow()->height()+30);

	originalPixmap.save(framefilename.c_str(), 0, 100);
}

void GLWidget::saveCameraInfo()
{
	std::ofstream output("config/camera.txt");
	//camer position
	output << camera.camera_position.x() << std::endl;
	output << camera.camera_position.y() << std::endl;
	output << camera.camera_position.z() << std::endl;

	float x, y, z, angle;
	camera.rotation.getAxisAndAngle(&x,&y,&z,&angle);

	output << x << std::endl;
	output << y << std::endl;
	output << z << std::endl;
	output << angle << std::endl;

	output.close();
}

void GLWidget::saveCameraInfo(const char* filename)
{
	std::ofstream output(filename);
	//camer position
	output << camera.camera_position.x() << std::endl;
	output << camera.camera_position.y() << std::endl;
	output << camera.camera_position.z() << std::endl;

	float x, y, z, angle;
	camera.rotation.getAxisAndAngle(&x, &y, &z, &angle);

	output << x << std::endl;
	output << y << std::endl;
	output << z << std::endl;
	output << angle << std::endl;

	output.close();
}

void GLWidget::loadCameraInfo(const char* filename /*= NULL*/)
{
	std::ifstream intput;
	if (filename != NULL)
	{
		intput.open(filename);
	}
	else
	{
		intput.open("config/camera.txt");
	}

	if (intput.is_open())
	{
		float camerx, camery, camerz;
		intput >> camerx;
		intput >> camery;
		intput >> camerz;

		camera.camera_position = QVector3D(camerx, camery, camerz);
		float x, y, z, angle;
		intput >> x;
		intput >> y;
		intput >> z;
		intput >> angle;
		camera.rotation = QQuaternion::fromAxisAndAngle(QVector3D(x, y, z), angle);
		camera.update();
	}
	intput.close();
	//camera.camera_offset = QVector3D(0, 0, 0);
}

void GLWidget::setShadowMapBias(double val)
{
	shadowmap.bias = val;
}

void GLWidget::initializeGL()
{
	initializeOpenGLFunctions();

	viewcube = new ViewCubeRender("3dmodel/ViewCube.obj");


	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	glEnable(GL_MULTISAMPLE);

	glClearColor(1.0, 1.0, 1.0, 1.0);

	//init shader program
	
	initShaders();
	initShadowMap();
	// Enable depth buffer
	glEnable(GL_DEPTH_TEST);

	// Enable back face culling
	glEnable(GL_CULL_FACE);

	//vao_ = new QOpenGLVertexArrayObject(this);
	
	//init Scene
	loboScene = new LoboScene();
	scene = new MyScene();
	scene_render = new LoboSceneRender(scene);
	mouseprojection = new MouseProjection();
	scene->setMouseProjection(mouseprojection);

	//camera position
	camera.init();
	camera.camera_position = QVector3D(0,-5,-10);
	camera.update();

	//load default
	loadCameraInfo();

	//light position
	program.setUniformValue(m_lightPosLoc, QVector3D(1, 4, 1));
	
	program.release();

	t1 = 0;

	connect(&timer, SIGNAL(timeout()), this, SLOT(updateWidget()));
	//connect(&timer, SIGNAL(timeout()), this, SLOT(repaint()));

	timer.setInterval(timer_spf);
	timer.start();

	//timer.start(timer_spf, this);
}

void GLWidget::initShaders()
{
	std::cout << "init shaders" << std::endl;

	if (!shadowmap.program.addShaderFromSourceFile(QOpenGLShader::Vertex, ":glsl/shader/vshadowshader.glsl"))
	{
		close();
		std::cout << "not open shader" << std::endl;;
	}
	// Compile fragment shader
	if (!shadowmap.program.addShaderFromSourceFile(QOpenGLShader::Fragment, ":glsl/shader/fshadowshader.glsl"))
		close();

	// Link shader pipeline
	if (!shadowmap.program.link())
		close();

	shadowmap.program.bind();
	QMatrix4x4 depthProjectionMatrix, depthViewMatrix;
	depthProjectionMatrix.ortho(-10, 10, -10, 10, 1.0, 100.0);
	//depthProjectionMatrix.scale(0.5);
	//depthProjectionMatrix.perspective(45.0, 1.0, 2.0, 1000);

	shadowmap.light_pos = QVector3D(1, 4, 1);
	depthViewMatrix.lookAt(shadowmap.light_pos, QVector3D(0, 0, 0), QVector3D(0, 1, 0));
	shadowmap.mvpShadow = depthProjectionMatrix*depthViewMatrix;
	
	shadowmap.m_MVPMatrixLoc = shadowmap.program.uniformLocation("depthMVP");
	shadowmap.program.setUniformValue(shadowmap.m_MVPMatrixLoc, shadowmap.mvpShadow);
	
	QMatrix4x4 biasMatirx(
		0.5, 0.0, 0.0, 0.0,
		0.0, 0.5, 0.0, 0.0,
		0.0, 0.0, 0.5, 0.0,
		0.5, 0.5, 0.5, 1.0
		);

	shadowmap.depthBiasMVP = biasMatirx.transposed()*shadowmap.mvpShadow;
	
	shadowmap.program.release();


	// Compile vertex shader
	if (!program.addShaderFromSourceFile(QOpenGLShader::Vertex, ":glsl/shader/vshader.glsl"))
	{
		close();
		std::cout << "not open shader" << std::endl;;
	}

	// Compile fragment shader
	if (!program.addShaderFromSourceFile(QOpenGLShader::Fragment, ":glsl/shader/fshader.glsl"))
		close();

	// Link shader pipeline
	if (!program.link())
		close();

	// Bind shader pipeline for use
	if (!program.bind())
		close();

	m_projMatrixLoc = program.uniformLocation("projMatrix");
	m_mvMatrixLoc = program.uniformLocation("mvMatrix");
	m_normalMatrixLoc = program.uniformLocation("normalMatrix");
	m_lightPosLoc = program.uniformLocation("lightPos");
	m_useCameraLightLoc = program.uniformLocation("usecameralight");
	m_useDeepWarping = program.uniformLocation("deepwarp");
	m_useTextureLoc = program.uniformLocation("usetexture");
	m_useShadowLoc = program.uniformLocation("useShadow");
	m_useVertexColorLoc = program.uniformLocation("useinputcolor");

	shadowmap.depthTextureLoc = program.uniformLocation("shadowMap");
	shadowmap.depthBiasLoc = program.uniformLocation("DepthBiasMVP");
	shadowmap.biasLoc = program.uniformLocation("bias");

	//program.release();
}

void GLWidget::initShadowMap()
{
	shadowmap.FramebufferName = 0;
	glGenFramebuffers(1, &shadowmap.FramebufferName);
	glBindFramebuffer(GL_FRAMEBUFFER, shadowmap.FramebufferName);
	
	// Depth texture. Slower than a depth buffer, but you can sample it later in your shader
	glGenTextures(1, &shadowmap.depthTexture);
	glBindTexture(GL_TEXTURE_2D, shadowmap.depthTexture);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT,
		4096, 4096, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
	GLfloat borderColor[] = { 1.0, 1.0, 1.0, 1.0 };
	glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, borderColor);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_R_TO_TEXTURE);

	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, shadowmap.depthTexture, 0);

	glDrawBuffer(GL_NONE); // No color buffer is drawn to.
	glReadBuffer(GL_NONE);

	if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
	std::cout << "false frame buffer" << std::endl;
	
	glBindTexture(GL_TEXTURE_2D, 0);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);

}

void GLWidget::drawBackGround(QVector4D color1, QVector4D color2)
{
	glDisable(GL_DEPTH_TEST);
	//int DiffuseProduct_loc = program.uniformLocation("DiffuseProduct");
	//int BoolPoint_loc = program.uniformLocation("point");
	//program.setUniformValue(BoolPoint_loc, 1);

	//program.setUniformValue(m_projMatrixLoc, m_world);
	//program.setUniformValue(m_mvMatrixLoc, m_world);
	//program.setUniformValue(m_normalMatrixLoc, m_world);

	glDisable(GL_LIGHTING);
	glBegin(GL_QUADS);

	//red color
	glColor3f(color1.x(), color1.y(), color1.z());
	glVertex3f(-1.0, -1.0,0);
	glVertex3f(1.0, -1.0,0);
	//blue color
	glColor3f(color2.x(), color2.y(), color2.z());
	glVertex3f(1.0, 1.0,0);
	glVertex3f(-1.0, 1.0,0);
	glEnd();

	//program.setUniformValue(BoolPoint_loc, 0);
	glEnable(GL_LIGHTING);
	glEnable(GL_DEPTH_TEST);
}

void GLWidget::renderShadowMap()
{
	//glEnable(GL_CULL_FACE);
	//glCullFace(GL_FRONT);
	shadowmap.program.bind();
	glBindFramebuffer(GL_FRAMEBUFFER, shadowmap.FramebufferName);
	glViewport(0, 0, 4096, 4096);

	glClear(GL_DEPTH_BUFFER_BIT);

	//glBindTexture(GL_TEXTURE_2D, shadowmap.depthTexture);
	scene_render->drawSceneShadowMap(&shadowmap.program);

	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glViewport(0, 0, width(), height());
	shadowmap.program.release();
	//glCullFace(GL_BACK);
	//glDisable(GL_CULL_FACE);
}

void GLWidget::paintGL()
{	
	painter.begin(this);
	painter.beginNativePainting();

	glEnable(GL_MULTISAMPLE);
	glEnable(GL_DEPTH_TEST);
	

	switch (scene_render->getRendermode())
	{
	case LoboSceneRender::fill:
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		break;
	case LoboSceneRender::wireframe:
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		break;
	case LoboSceneRender::point:
		glPointSize(5);
		glPolygonMode(GL_FRONT_AND_BACK, GL_POINT);
		break;
	default:
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		break;
	}
	
	//render map
	renderShadowMap();


	glDisable(GL_CULL_FACE);

	if (scene_render->getIfBlend())
	{
		glEnable(GL_CULL_FACE);
		glCullFace(GL_BACK);
		glEnable(GL_BLEND); //Enable blending.
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		//glDisable(GL_DEPTH_TEST);
	}

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//bottom color
	QVector4D color1(1.0, 1.0, 1.0, 1.0);
	//top color
	QVector4D color2(1.0, 1.0, 1.0, 1.0);
	drawBackGround(color1, color2);

	program.bind();

	QMatrix4x4 m_world;
	m_world.setToIdentity();

	program.setUniformValue(m_projMatrixLoc, m_proj);
	program.setUniformValue(m_mvMatrixLoc, camera.m_camera * m_world);
	QMatrix3x3 normalMatrix = m_world.normalMatrix();
	program.setUniformValue(m_normalMatrixLoc, normalMatrix);
	program.setUniformValue(shadowmap.depthBiasLoc, shadowmap.depthBiasMVP);
	program.setUniformValue(m_useCameraLightLoc, useCameraLight);
	program.setUniformValue(m_useDeepWarping, useDeepWarping);
	program.setUniformValue(m_useTextureLoc, useTexture);
	program.setUniformValue(m_useShadowLoc, useShadow);
	program.setUniformValue(m_useVertexColorLoc, useVertexColor);

	glUniform1f(shadowmap.biasLoc, shadowmap.bias);
	
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, shadowmap.depthTexture);
	glUniform1i(shadowmap.depthTextureLoc, 1);


	trender1 = clock_t();
	scene_render->drawScene(&program);
	trender2 = clock_t();

	scene_render->setRenderWireframe(getRenderWireFrame());
	/*if (getRenderWireFrame())
	{
	int BoolPoint_loc = program.uniformLocation("wireframe");
	program.setUniformValue(BoolPoint_loc, 1);
	glLineWidth(1);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

	scene_render->drawSceneWireFrame(&program);

	glLineWidth(1);

	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	program.setUniformValue(BoolPoint_loc, 0);
	}*/

	//renderQuad();

	//this will change view matrix

	if (getSwtichRenderViewCube())
	{
		viewcube->drawMesh(&program, m_proj, camera.rotation, width(), height(), Vector2d(70, height() - 70), 2);
	}

	program.release();

	glDisable(GL_DEPTH_TEST);
	glDisable(GL_BLEND);
	glDisable(GL_CULL_FACE);
	glDisable(GL_TEXTURE_2D);

	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	painter.endNativePainting();

	painter.setRenderHints(QPainter::Antialiasing,true);
	painter.setPen(QColor(255, 100, 0, 100));

	if (camera.K_shift)
	drawRect(QRect(0, 0, 0, 0));

	if (getSwtichPaint())
	{
		painter.setPen(QColor(255, 100, 0, 255));
		painter.setFont(QFont("Times", 15));
		std::ostringstream oss;
		oss.precision(3);
		oss << "FPS: " << fps_;
		std::string var = oss.str();
		painter.drawText(10, 20, var.c_str());

		painter.setPen(QColor(10, 10, 10, 255));
		painter.setFont(QFont("Times", 12));
		scene_render->paintScene(painter);
	}
	painter.end();
}

void GLWidget::shadowMap()
{
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	shadowmap.program.bind();
	glBindFramebuffer(GL_FRAMEBUFFER, shadowmap.FramebufferName);
	shadowmap.program.setUniformValue(shadowmap.m_MVPMatrixLoc, shadowmap.mvpShadow);
	
	scene_render->drawScene(&shadowmap.program);

	glReadBuffer(shadowmap.FramebufferName);
	GLfloat* pixels = new GLfloat[2048 * 2048];
	//glReadPixels(0, 0, 1024, 1024, GL_RED, GL_FLOAT, pixels);
	QImage image((uchar*)pixels, 2048, 2048, QImage::Format_ARGB32);
	image.save("test.png");
	delete[] pixels;

	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	shadowmap.program.release();
	
	glDisable(GL_CULL_FACE);

	
	//system("pause");
}

void GLWidget::renderQuad()
{

	GLfloat quadVertices[] = {
		// Positions        // Texture Coords
		-1.0f, 1.0f, 0.0f, 
		-1.0f, -1.0f, 0.0f, 
		1.0f, 1.0f, 0.0f,
		1.0f, -1.0f, 0.0f
	};

	GLfloat textcoor[] = {
		0.0f, 1.0f,
		0.0f, 0.0f,
		1.0f, 1.0f,
		1.0f, 0.0f,
	};

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, shadowmap.depthTexture);
	int texLoc = program.uniformLocation("colortexture");
	glUniform1i(texLoc, 0);

	int vertexLocation = program.attributeLocation("a_position");
	glEnableVertexAttribArray(vertexLocation);
	glVertexAttribPointer(vertexLocation, 3, GL_FLOAT, GL_FALSE, 0 * sizeof(GLfloat), quadVertices);

	int textueLocation = program.attributeLocation("a_texcoord");
	glEnableVertexAttribArray(textueLocation);
	glVertexAttribPointer(textueLocation, 2, GL_FLOAT, GL_FALSE, 0 * sizeof(GLfloat), textcoor);

	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

	
	glDisableVertexAttribArray(vertexLocation);
	glDisableVertexAttribArray(textueLocation);

}

//void GLWidget::paintEvent(QTimerEvent *e)
//{
//	update();
//}

void GLWidget::resizeGL(int w, int h)
{
	qreal aspect = qreal(w) / qreal(h ? h : 1);

	// Set near plane to 1.0, far plane to 500.0, field of view 60.0 degrees
	const qreal zNear = 0.02, zFar = 1500.0, fov = 60.0;

	// Reset projection
	m_proj.setToIdentity();

	// Set perspective projection
	if (!renderconfig.is_ortho)
	m_proj.perspective(fov, aspect, zNear, zFar);
	else
	{
		m_proj.ortho(-aspect, aspect, -1, 1, -1.0, 1500.0);
		//m_proj.scale(0.2);
	}
}

void GLWidget::keyPressEvent(QKeyEvent *e)
{
	if (e->key() == Qt::Key_W)
	{
		camera.K_W = true;
	}
	if (e->key() == Qt::Key_S)
	{
		camera.K_S = true;
	}
	if (e->key() == Qt::Key_A)
	{
		camera.K_A = true;
	}
	if (e->key() == Qt::Key_D)
	{
		camera.K_D = true;
	}
	if (e->key() == Qt::Key_Space)
	{
		camera.K_space = true;
	}
	if (e->key() == Qt::Key_Control)
	{
		camera.K_ctr = true;
	}

	if (e->key() == Qt::Key_Shift)
	{
		camera.K_shift = true;
	}

	if (e->key() == Qt::Key_P)
	{
		if (loboScene->config.b_start_simulate)
		{
			stopSimulate();
		}
		else
		{
			startSimulate();
		}

		if (scene->getStartSimulation())
		{
			scene->setStartSimulation(false);
		}
		else
		{
			scene->setStartSimulation(true);
		}

	}

	loboScene->keyPressEvent(e);
}

void GLWidget::keyReleaseEvent(QKeyEvent *e)
{
	if (e->key() == Qt::Key_W)
	{
		camera.K_W = false;
	}
	if (e->key() == Qt::Key_S)
	{
		camera.K_S = false;
	}
	if (e->key() == Qt::Key_A)
	{
		camera.K_A = false;
	}
	if (e->key() == Qt::Key_D)
	{
		camera.K_D = false;
	}
	if (e->key() == Qt::Key_Space)
	{
		camera.K_space = false;
	}
	if (e->key() == Qt::Key_Control)
	{
		camera.K_ctr = false;
	}

	if (e->key() == Qt::Key_Shift)
	{
		camera.K_shift = false;
	}

	loboScene->keyReleaseEvent(e);
}

void GLWidget::mousePressEvent(QMouseEvent *e)
{
	setFocus();
	camera.mousePressPosition = QVector2D(e->globalPos());
	camera.mousePressPosition_loc = QVector2D(e->localPos());

	if (e->button() == Qt::RightButton)
	{
		setCursor(Qt::BlankCursor);
	}
	else if(e->button() == Qt::LeftButton)
	{
		QMatrix4x4 mvp = m_proj*camera.m_camera;
		std::copy(mvp.data(), mvp.data() + 16, (select.MVP).data());
		select.width = this->width();
		select.height = this->height();
		select.setRect(camera.mousePressPosition_loc.x(), camera.mouseReleasePosition_loc.x(), camera.mousePressPosition_loc.y(), camera.mouseReleasePosition_loc.y());
		select.ortho = false;

		this->mouseprojection->setMVPmatrix(select.MVP);
		this->mouseprojection->setWidth(this->width());
		this->mouseprojection->setHeight(this->height());

		this->mouseprojection->setMouseRect(camera.mousePressPosition_loc.x(), camera.mouseReleasePosition_loc.x(), camera.mousePressPosition_loc.y(), camera.mouseReleasePosition_loc.y());
		this->mouseprojection->setMouseXY(camera.mouseReleasePosition_loc.x(), camera.mouseReleasePosition_loc.y());
		this->mouseprojection->setPreMouseXY(camera.mousePressPosition_loc.x(), camera.mousePressPosition_loc.y());
		scene->singleClickEvent();

		loboScene->setMouseStruct(select);

		if (camera.K_shift)
		{
			camera.mousePressPosition_loc = QVector2D(e->localPos());
			camera.mouseReleasePosition_loc = QVector2D(e->localPos());
		}
	}
	loboScene->mousePressEvent(e);

}

void GLWidget::mouseMoveEvent(QMouseEvent *e)
{
	qreal aspect = qreal(width()) / qreal(height() ? height() : 1);
	camera.mouseReleasePosition = QVector2D(e->globalPos());
	camera.mouseReleasePosition_loc = QVector2D(e->localPos());

	if (e->buttons() == Qt::RightButton)
	{
		QVector2D diff = QVector2D(e->globalPos()) - camera.mousePressPosition;
		camera.mousePressPosition = QVector2D(e->globalPos());
		// Rotation axis is perpendicular to the mouse position difference
		// vector
		QVector3D n = QVector3D(diff.y(), diff.x(), 0.0).normalized();

		// Accelerate angular speed relative to the length of the mouse sweep
		qreal acc = diff.length() / (width())*100;

		// Calculate new rotation axis as weighted sum
		camera.rotationAxis = (n * acc).normalized();

		camera.rotation = QQuaternion::fromAxisAndAngle(camera.rotationAxis, acc) * camera.rotation;
		// Increase angular speed
		camera.update();
	}
	else if(e->buttons() == Qt::LeftButton)
	{
		QMatrix4x4 mvp = m_proj*camera.m_camera;
		std::copy(mvp.data(), mvp.data() + 16, (select.MVP).data());
		select.width = this->width();
		select.height = this->height();
		select.setRect(camera.mousePressPosition_loc.x(), camera.mouseReleasePosition_loc.x(), camera.mousePressPosition_loc.y(), camera.mouseReleasePosition_loc.y());

		select.ortho = false;

		this->mouseprojection->setMVPmatrix(select.MVP);
		this->mouseprojection->setWidth(this->width());
		this->mouseprojection->setHeight(this->height());
		
		this->mouseprojection->setMouseRect(camera.mousePressPosition_loc.x(), camera.mouseReleasePosition_loc.x(), camera.mousePressPosition_loc.y(), camera.mouseReleasePosition_loc.y());
		this->mouseprojection->setMouseXY(camera.mouseReleasePosition_loc.x(), camera.mouseReleasePosition_loc.y());
		this->mouseprojection->setPreMouseXY(camera.mousePressPosition_loc.x(), camera.mousePressPosition_loc.y());

		loboScene->setMouseStruct(select);
		this->scene->mouseMoveEvent();

		if (camera.K_shift)
		{
			this->scene->rectSelectEvent();

			camera.mouseReleasePosition_loc = QVector2D(e->localPos());
			// check if objects inside selected area

			
			std::copy(mvp.data(), mvp.data() + 16, (select.MVP).data());
			select.width = this->width();
			select.height = this->height();
			select.setRect(camera.mousePressPosition_loc.x(), camera.mouseReleasePosition_loc.x(), camera.mousePressPosition_loc.y(), camera.mouseReleasePosition_loc.y());
			select.ortho = false;
			loboScene->reciveSelectedInfo(select);
			
			/*Vector3d point = Vector3d::Zero();
			Vector2d result;
			lobo::mapToScreenCoordinate(select.MVP, width(), height(), point, result,true);
			std::cout << result << std::endl;*/
		}

		loboScene->mouseMoveEvent(e);
	}

	
}

void GLWidget::mouseReleaseEvent(QMouseEvent* e)
{
	camera.mouseReleasePosition = QVector2D(e->globalPos());
	if (e->button() == Qt::RightButton)
	{
		QCursor c = cursor();
		c.setPos(mapToGlobal(QPoint(width() / 2, height() / 2)));
		c.setShape(Qt::ArrowCursor);
		setCursor(c);
	}
	else if (e->button() == Qt::LeftButton)
	{
		if (camera.K_shift)
		{
			camera.mouseReleasePosition_loc = QVector2D(e->localPos());
		}
	}
	camera.clearMouse();
	loboScene->mouseReleaseEvent(e);
	this->scene->mouseReleaseEvent();
}

void GLWidget::setLoboScene(const char* filename)
{
	cleanLoboScene();
	t2 = clock();

	//loboScene->readConfig(filename);
	scene->readConfigAscii(filename);
	
	this->loadCameraInfo(scene->getScenecameraConfig().c_str());

	t1 = clock();
	fps_ = (double)CLOCKS_PER_SEC / ((double)(t1 - t2) == 0 ? 0.0001 : (double)(t1 - t2));
	std::cout << "init scene using time: " << 1 / fps_ << std::endl;
}

MyScene* GLWidget::getLoboScene()
{
	return scene;
}

void GLWidget::cleanLoboScene()
{
	if (loboScene != NULL)
	{
		delete loboScene;
	}

	if (scene_render != NULL)
	{
		delete scene_render;
	}

	if (scene != NULL)
	{
		delete scene;
	}

	

	scene = new MyScene();
	scene->setMouseProjection(mouseprojection);
	scene_render = new LoboSceneRender(scene);

	loboScene = new LoboScene();
}

void GLWidget::drawRect(QRect rect)
{
	rect.setX(MIN(camera.mousePressPosition_loc.x(), camera.mouseReleasePosition_loc.x()));
	rect.setY(MIN(camera.mousePressPosition_loc.y(), camera.mouseReleasePosition_loc.y()));
	rect.setWidth(std::abs(camera.mousePressPosition_loc.x() -
		camera.mouseReleasePosition_loc.x()
		));
	rect.setHeight(std::abs(camera.mousePressPosition_loc.y() -
		camera.mouseReleasePosition_loc.y()
		));

	painter.drawRect(rect);
	painter.fillRect(rect, QColor(255, 100, 0, 100));
}

void GLWidget::updateWidget()
{
	camera.camera_offset = QVector3D(0, 0, 0);
	if (camera.K_W)
	{
		camera.camera_offset[2] += camera.move_speed;
	}
	if (camera.K_S)
	{
		camera.camera_offset[2] -= camera.move_speed;
	}
	if (camera.K_A)
	{
		camera.camera_offset[0] += camera.move_speed;
	}
	if (camera.K_D)
	{
		camera.camera_offset[0] -= camera.move_speed;
	}
	if (camera.K_space)
	{
		camera.camera_offset[1] -= camera.move_speed;
	}
	if (camera.K_ctr)
	{
		camera.camera_offset[1] += camera.move_speed;
	}

	camera.update();

	
	//loboScene->simulate();
	
	if (getSwtichSaveAnimation())
	{
		saveAnimation();
		//saveAnimationWithUi();
	}

	t1 = clock();
	scene->updateScene();
	t2 = clock();

	fps_ = (double)CLOCKS_PER_SEC / ((double)(t2 - t1) <= 1 ? 1 : (double)(t2 - t1));

	//fps_ = (double)(t2 - t1) / (double)CLOCKS_PER_SEC;

	if (fps_ > 2000)
	{
		fps_ = 2000;
	}

	repaint();
}
