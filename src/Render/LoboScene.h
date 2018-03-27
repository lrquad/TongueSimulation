#pragma once
#include "LoboObjMesh.h"
#include "Simulator/LoboSimulator.h"
#include <qevent.h>
#include <QPainter>

enum Rendermode {wireframe, fill,point};
struct SceneConfig
{
	bool b_save_screen_image;
	bool b_save_animation;
	Rendermode rendermode;
	bool b_start_simulate;
	bool b_render_model;
	bool b_render_simulator;
	int number_of_frames_to_save;
	SceneConfig()
	{
		b_render_model = true;
		b_save_screen_image = false;
		b_start_simulate = false;
		b_save_animation = false;
		rendermode = fill;
	}
};

class LoboScene
{
public:
	LoboScene();
	~LoboScene();
	bool addObjModel(const char* filename);
	void moveObjModel(double x,double y,double z);
	void scaleObjModel(double scale);
	void updateObjCollisionShape();
	void paintScene(QPainter &painter);
	void drawScene(QOpenGLShaderProgram *program);
	void readConfig(const char* filename);
	void simulate();
	void saveAnimation(int skipframe = 1);

	void keyPressEvent(QKeyEvent *key_event);
	void keyReleaseEvent(QKeyEvent *key_event);

	void mousePressEvent(QMouseEvent *mouse_event);
	void mouseMoveEvent(QMouseEvent *mouse_event);
	void mouseReleaseEvent(QMouseEvent *mouse_event);

	void setMouseStruct(lobo::MouseDescribe select);
	void setMVPMatrix(Matrix4d &MVP);
	void reciveSelectedInfo(lobo::MouseDescribe select);

	void initCollisionWorld();
	void checkCollision();
	void checkCollisionTT(LoboObjMesh *mode0, LoboObjMesh *mode1);


	SceneConfig config;
	std::vector<LoboSimulator*> simulator_list_;
	std::vector<LoboObjMesh*> model_list_;
	btCollisionWorld* bt_collision_world;
	int simulation_step;
	int skip_steps;
	int when_nail_back_;
	
	clock_t t1, t2;
};

