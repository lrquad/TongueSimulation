#pragma once
#include "GLWidget.h"

class AssistantGLWidget : public GLWidget
{
	Q_OBJECT
public:
	AssistantGLWidget(QWidget* parent = 0);
	~AssistantGLWidget();

	virtual void createNewSceneRender(MyScene* input_scene);

protected:
	void initializeGL() Q_DECL_OVERRIDE;
	void paintGL() Q_DECL_OVERRIDE;


	void mousePressEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
	void mouseMoveEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
	void mouseReleaseEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
	void keyPressEvent(QKeyEvent *event) Q_DECL_OVERRIDE;
	void keyReleaseEvent(QKeyEvent *event) Q_DECL_OVERRIDE;

};

