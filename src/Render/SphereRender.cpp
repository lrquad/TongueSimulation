#include "SphereRender.h"
#include "ColorTable/Colortable.h"
#include "LoboTriMesh/LoboTriMesh.h"
#include "LoboTriMesh/LoboTriMeshRender.h"

SphereRender::SphereRender(const char* filepath, bool smoothing)
{
	spheretriobj = new LoboTriMesh(filepath,false);
	trimeshrender = new LoboTriMeshRender(spheretriobj,true);
	trimeshrender->setSmoothshading(smoothing);
	trimeshrender->setAlphaChannel(0.4);
	trimeshrender->setIsTransparent(true);
	trimeshrender->setAcceptwireframe(false);
}

SphereRender::~SphereRender()
{
	delete spheretriobj;
	delete trimeshrender;
}

void SphereRender::drawMesh(QOpenGLShaderProgram *program, Vector3d position, double scale, int colorindex /*= 0*/)
{
	double defaultcolor[3];
	getColor(colorindex, defaultcolor);
	trimeshrender->setDiffuesColor(defaultcolor[0], defaultcolor[1], defaultcolor[2]);
	spheretriobj->scaleOriginalAtCenter(scale);
	spheretriobj->translateMesh(position.x(), position.y(), position.z());
	trimeshrender->updateBuffer();
	trimeshrender->renderModel(program);
}

void SphereRender::drawMesh(QOpenGLShaderProgram *program, Vector3d position, Matrix3d rotation, double scale, int colorindex /*= 0*/)
{
	double defaultcolor[3];
	getColor(colorindex, defaultcolor);
	trimeshrender->setDiffuesColor(defaultcolor[0], defaultcolor[1], defaultcolor[2]);
	spheretriobj->scaleOriginalAtCenter(scale);
	spheretriobj->transformMesh(position,rotation);
	trimeshrender->updateBuffer();
	trimeshrender->renderModel(program);
}

void SphereRender::drawMesh(QOpenGLShaderProgram *program)
{
	trimeshrender->renderModel(program);
}

void SphereRender::drawMeshScaleGlobal(QOpenGLShaderProgram *program, Vector3d position, Matrix3d rotation, double scale, int colorindex /*= 0*/)
{
	double defaultcolor[3];
	getColor(colorindex, defaultcolor);
	trimeshrender->setDiffuesColor(defaultcolor[0], defaultcolor[1], defaultcolor[2]);
	//spheretriobj->scaleOriginalAtCenter(scale);
	spheretriobj->transformMesh(position, rotation);
	spheretriobj->scaleAtCurrent(scale);
	trimeshrender->updateBuffer();
	trimeshrender->renderModel(program);
}

void SphereRender::updateMeshBuffer(Vector3d position, Matrix3d rotation, double scale)
{
	spheretriobj->transformMesh(position, rotation);
	spheretriobj->scaleAtCurrent(scale);
	trimeshrender->updateBuffer();
}

void SphereRender::exportObj(const char* filename)
{
	spheretriobj->exportObj(filename);
}
