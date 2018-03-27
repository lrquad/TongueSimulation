#pragma once
#define CAMERA_SPEED 0.03


#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLFunctions_4_3_Core>
#include <QMatrix4x4>
#include <QQuaternion>
#include <QVector2D>
#include <QBasicTimer>
#include <QTimer>
#include <QOpenGLShaderProgram>
#include <QOpenGLTexture>
#include "LoboScene.h"
#include <QPainter>
#include <QOpenGLFramebufferObject>
#include "Scene/LoboSceneRender.h"
#include "Scene/myScene.h"


struct RenderConfig{
	bool is_blend;
	bool is_ortho;
	RenderConfig()
	{
		is_blend = false;
		is_ortho = false;
	}
};

struct ShadowMap{
	GLuint FramebufferName;
	GLuint depthTexture;
	int depthTextureLoc;
	QOpenGLFramebufferObject *framebufferobjet;
	QOpenGLTexture *texture;
	QOpenGLShaderProgram program;

	QVector3D light_pos;
	QMatrix4x4 mvpShadow;
	QMatrix4x4 depthBiasMVP;
	int m_MVPMatrixLoc;
	int depthBiasLoc;

	float bias;
	int biasLoc;

	ShadowMap()
	{
		framebufferobjet = NULL;
		texture = NULL;
		bias = 0.0004;
	}
	~ShadowMap()
	{
		if (framebufferobjet!=NULL)
		delete framebufferobjet;
		if (texture != NULL)
		delete texture;
	}
};

struct Camera
{
	QVector3D camera_position;
	QVector3D camera_offset;

	QVector2D mousePressPosition;
	QVector2D mouseReleasePosition;

	QVector2D mousePressPosition_loc;
	QVector2D mouseReleasePosition_loc;


	qreal angularSpeed;
	QQuaternion rotation;
	QVector3D rotationAxis;
	QMatrix4x4 m_camera;
	double move_speed;

	bool K_W;
	bool K_S;
	bool K_A;
	bool K_D;
	bool K_space;
	bool K_ctr;
	bool K_shift;

	Camera()
	{
		init();
	}

	void init()
	{
		K_W = false;
		K_S = false;
		K_A = false;
		K_D = false;
		K_space = false;
		K_ctr = false;
		K_shift = false;
		camera_offset = QVector3D(0, 0, 0);
		mousePressPosition = QVector2D(0, 0);
		mouseReleasePosition = QVector2D(0, 0);
		mousePressPosition_loc = QVector2D(0, 0);
		mouseReleasePosition_loc = QVector2D(0, 0);

		rotation = QQuaternion::fromAxisAndAngle(QVector3D(0,0,0), 0);
		move_speed = CAMERA_SPEED;
	}

	void update()
	{
		m_camera.setToIdentity();
		m_camera.rotate(rotation);
		camera_offset = m_camera.inverted()*camera_offset;
		camera_position += camera_offset;
		m_camera.translate(camera_position);
		camera_offset = QVector3D(0, 0, 0);
	}

	void clearMouse()
	{
		mousePressPosition = QVector2D(0, 0);
		mouseReleasePosition = QVector2D(0, 0);
		mousePressPosition_loc = QVector2D(0, 0);
		mouseReleasePosition_loc = QVector2D(0, 0);
	}

	void resetKey()
	{
		K_W = false;
		K_S = false;
		K_A = false;
		K_D = false;
		K_space = false;
		K_ctr = false;
		K_shift = false;
	}
};

QT_FORWARD_DECLARE_CLASS(QOpenGLShaderProgram)


class MouseProjection;
class ViewCubeRender;

class GLWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
	Q_OBJECT
public:
	GLWidget(QWidget *parent = 0);
	~GLWidget();
	void setLoboScene(const char* file);
	void cleanLoboScene();
	MyScene* getLoboScene();
	void startSimulate();
	void stopSimulate();

	void toggelBlend();
	void togelOrtho(){
		renderconfig.is_ortho = !renderconfig.is_ortho;
		resizeGL(width(), height());
	};

	void saveScreenShot();
	void saveScreenShotUi();
	void saveCameraInfo();
	void saveCameraInfo(const char* filename);
	void loadCameraInfo(const char* filename = NULL);

	LoboSceneRender * getScene_render() const { return scene_render; }
	void setScene_render(LoboSceneRender * render){ scene_render = render; }

	bool getSwtichPaint() const { return swtichPaint; }
	void setSwtichPaint(bool val) { swtichPaint = val; }
	bool getSwtichSaveAnimation() const { return swtichSaveAnimation; }
	void setSwtichSaveAnimation(bool val) { swtichSaveAnimation = val; }
	bool getSwtichRenderViewCube() const { return swtichRenderViewCube; }
	void setSwtichRenderViewCube(bool val) { swtichRenderViewCube = val; }


	void setShadowMapBias(double val);

	bool getUseCameraLight() const { return useCameraLight; }
	void setUseCameraLight(bool val) { useCameraLight = val; }
	bool getUseTexture() const { return useTexture; }
	void setUseTexture(bool val) { useTexture = val; }
	bool getUseShadow() const { return useShadow; }
	void setUseShadow(bool val) { useShadow = val; }
	bool getUseVertexColor() const { return useVertexColor; }
	void setUseVertexColor(bool val) { useVertexColor = val; }

	double getCameraMoveSpeed() const { return camera.move_speed; }
	void setCameraMoveSpeed(double val) { camera.move_speed = val; }

	bool getRenderWireFrame() const { return renderWireFrame; }
	void setRenderWireFrame(bool val) { renderWireFrame = val; }

	bool getUseDeepWarping() const { return useDeepWarping; }
	void setUseDeepWarping(bool val) { useDeepWarping = val; }
protected:

	void initializeGL() Q_DECL_OVERRIDE;
	void paintGL() Q_DECL_OVERRIDE;

	void shadowMap();
	void renderQuad();

	void resizeGL(int width, int height) Q_DECL_OVERRIDE;
	
	void renderShadowMap();
	void mousePressEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
	void mouseMoveEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
	void mouseReleaseEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
	void keyPressEvent(QKeyEvent *event) Q_DECL_OVERRIDE;
	void keyReleaseEvent(QKeyEvent *event) Q_DECL_OVERRIDE;
	void focusOutEvent(QFocusEvent * event) Q_DECL_OVERRIDE;
	
	//void paintEvent(QTimerEvent *e);
	void timerEvent(QTimerEvent *e);
	void cleanup();
	void initShaders();
	void saveAnimation();
	void saveAnimationWithUi();
	void initShadowMap();
	void drawBackGround(QVector4D color1, QVector4D color2);

	void drawRect(QRect rect);
	
	ShadowMap shadowmap;

private slots:
	void updateWidget();

protected:

	ViewCubeRender* viewcube;
	MouseProjection* mouseprojection;
	QTimer timer;
	QOpenGLShaderProgram program;
	QOpenGLVertexArrayObject* vao_;
	LoboScene *loboScene; // old one
	MyScene * scene; //new scene class
	LoboSceneRender * scene_render;

	QPainter painter;
	bool startsimulator;
	clock_t t1, t2;
	clock_t trender1, trender2;

	double fps_;
	int timer_spf;
	int m_projMatrixLoc;
	int m_mvMatrixLoc;
	int m_normalMatrixLoc;
	int m_lightPosLoc;
	int m_useCameraLightLoc;
	int m_useDeepWarping;
	int m_useTextureLoc;
	int m_useShadowLoc;
	int m_useVertexColorLoc;

	QMatrix4x4 m_proj;
	QMatrix4x4 m_camera;
	QMatrix4x4 m_world;

	Camera camera;

	lobo::MouseDescribe select;
	RenderConfig renderconfig;

	bool swtichPaint;
	bool swtichSaveAnimation;
	bool swtichRenderViewCube;
	bool useCameraLight;
	bool useTexture;
	bool useShadow;
	bool useVertexColor;
	bool renderWireFrame;
	bool useDeepWarping;

};

