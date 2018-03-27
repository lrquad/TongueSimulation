#include "LoboIsotropicHyperElasticSimulatorRender.h"
#include "LoboVolumetricMesh/LoboVolumetriceMeshCore.h"
#include "Simulator/IsotropicHyperelasticFEM/Simulator/LoboIsotropicHyperlasticSimulator.h"

LoboIsotropicHyperElasticSimulatorRender::LoboIsotropicHyperElasticSimulatorRender(LoboIsotropicHyperlasticSimulator* simulator) :SimulatorRenderBase(simulator)
{
	this->mysimulator = simulator;
	tetvolumetricRender = new TetVolumetricMeshRender();
}

LoboIsotropicHyperElasticSimulatorRender::~LoboIsotropicHyperElasticSimulatorRender()
{
	delete tetvolumetricRender;
}

void LoboIsotropicHyperElasticSimulatorRender::renderSimulator(QOpenGLShaderProgram *program)
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

	

	if (getRender_volume_mesh())
		this->tetvolumetricRender->renderAllVolumetricMesh(program, downCastTetVolMesh(simulator->getVolumetricMesh()), 5);

	drawForce(program);
}

void LoboIsotropicHyperElasticSimulatorRender::paintSimulator(QPainter &painter)
{
	int width = painter.window().width();
	int height = painter.window().height();
	int start_x = width - 100;
	int start_y = height - 50;
	int offset_y = 0;

	std::ostringstream timestep;
	timestep << "Steps: " << this->mysimulator->getSimulation_steps();
	painter.drawText(10, 40, timestep.str().c_str());

	std::ostringstream oss;
	oss.precision(3);
}

LoboIsotropicHyperElasticSimulatorRender* downCastIsotropicHyperElasticRender(SimulatorRenderBase* render)
{
	return (LoboIsotropicHyperElasticSimulatorRender*)render;
}
