#include "FloatingMDSTVKSimulatorRender.h"
#include "LoboVolumetricMesh/TetVolumetricMeshRender.h"
#include "Simulator/ReducedSTVK/Freefloating/FloatingMDSTVKSimulator.h"
#include "Reduced/Modalderivtives/SubspaceModalDerivativesModule.h"
#include <iomanip>

FloatingMDSTVKSimulatorRender::FloatingMDSTVKSimulatorRender(FloatingMDSTVKSimulator* simualtor) :SimulatorRenderBase(simualtor)
{
	this->floatingMDSTVKSimulator = simualtor;
	tetvolumetricRender = new TetVolumetricMeshRender();
}

FloatingMDSTVKSimulatorRender::~FloatingMDSTVKSimulatorRender()
{
	delete tetvolumetricRender;
}

void FloatingMDSTVKSimulatorRender::renderSimulator(QOpenGLShaderProgram *program)
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
		this->tetvolumetricRender->renderAllVolumetricMesh(program, downCastTetVolMesh(simulator->getVolumetricMesh()), 4);

	
}

void FloatingMDSTVKSimulatorRender::paintSimulator(QPainter &painter)
{
	int width = painter.window().width();
	int height = painter.window().height();
	int start_x = width - 100;
	int start_y = height - 50;
	int offset_y = 0;

	std::ostringstream timestep;
	timestep << "Steps: " << this->floatingMDSTVKSimulator->getSimulation_steps();
	painter.drawText(10, 40, timestep.str().c_str());
	
	std::ostringstream oss;
	oss.precision(3);

	if (floatingMDSTVKSimulator->getSubspaceModuleRef() != NULL)
	{
		oss << std::fixed << std::setprecision(2) << "Modes: " << floatingMDSTVKSimulator->getSubspaceModuleRef()->r;
		painter.drawText(start_x, start_y + offset_y, oss.str().c_str());
	}

}

FloatingMDSTVKSimulatorRender* downCastFloatingMDSTVKSimulatorRender(SimulatorRenderBase* render)
{
	return (FloatingMDSTVKSimulatorRender*)render;
}
