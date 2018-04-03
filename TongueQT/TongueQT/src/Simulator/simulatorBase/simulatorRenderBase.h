#pragma once
#include <QPainter>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include "LoboSimulatorBase.h"
#include "Render/SphereRender.h"

class VolumetricMeshRender;
class SphereRender;

class SimulatorRenderBase
{
public:
	SimulatorRenderBase(LoboSimulatorBase* simulator_);
	~SimulatorRenderBase();

	virtual void renderSimulator(QOpenGLShaderProgram *program);
	virtual void paintSimulator(QPainter &painter);

	typedef enum { ASCII, BINARY, NUM_FILE_FORMATS } fileFormatType;
	
	virtual void saveRenderInfo(const char* filename, fileFormatType filetype) const; // save all render info like frame's position or picked node position and so on.

	int getMesh_color_index() const { return mesh_color_index; }
	void setMesh_color_index(int val) { mesh_color_index = val; }

	bool getRender_volume_mesh() const { return b_render_volume_mesh; }
	void setRender_volume_mesh(bool val) { b_render_volume_mesh = val; }

	bool getRender_nodes() const { return b_render_nodes; }
	void setRender_nodes(bool val) { b_render_nodes = val; }
protected:

	virtual void drawForce(QOpenGLShaderProgram* program);
	virtual void drawAllExForce(QOpenGLShaderProgram* program);

	void drawArrow(QOpenGLShaderProgram* program, Vector3d start, Vector3d end, int colorindex = 0);
	void drawMeshSelected(QOpenGLShaderProgram *program);
	void drawSingleSelected(QOpenGLShaderProgram *program);
	void drawMeshNodes(QOpenGLShaderProgram* program);

	void drawMeshConstrainedNodes(QOpenGLShaderProgram *program);

	LoboSimulatorBase* simulator;
	VolumetricMeshRender* volumetricMeshRender;

	bool b_render_volume_mesh;
	bool b_render_nodes;
	bool b_paint_info;

	bool renderselected;
	bool renderConstrained;

	//render sphere
	SphereRender* sphere1;
	SphereRender* spherelow;


	int mesh_color_index;
};

