#include "ReducedTetSTVKSimulatorRender.h"
#include "Simulator/ReducedSTVK/ReducedTetSTVKSimulator.h"
#include "LoboVolumetricMesh/TetVolumetricMeshRender.h"


ReducedTetSTVKSimulatorRender::ReducedTetSTVKSimulatorRender(ReducedTetSTVKSimulator* simulator) :SimulatorRenderBase(simulator)
{
	this->reducedtetSTVKSimulator = simulator;
	tetvolumetricRender = new TetVolumetricMeshRender();
	
}


ReducedTetSTVKSimulatorRender::~ReducedTetSTVKSimulatorRender()
{
	delete tetvolumetricRender;
}

void ReducedTetSTVKSimulatorRender::renderSimulator(QOpenGLShaderProgram *program)
{
	if (simulator->getVolumetricMesh() == NULL)
	{
		return;
	}

	if (renderConstrained)
	{
		drawMeshConstrainedNodes(program);
	}

	if (renderselected)
	{
		drawMeshSelected(program);
	}

	drawForce(program);

	if (getRender_volume_mesh())
	this->tetvolumetricRender->renderAllVolumetricMesh(program, downCastTetVolMesh(simulator->getVolumetricMesh()), 1);

}

void ReducedTetSTVKSimulatorRender::paintSimulator(QPainter &painter)
{

}

ReducedTetSTVKSimulatorRender* downCastRedcuedSTVKSimulatoRender(SimulatorRenderBase* render)
{
	return (ReducedTetSTVKSimulatorRender*)render;
}
