#include "HyperElasticTongueSimulatorRender.h"
#include "HyperelasticTongueSimulator.h"
#include "ColorTable/Colortable.h"
#include "LoboVolumetricMesh/LoboVolumetriceMeshCore.h"



HyperElasticTongueSimulatorRender::HyperElasticTongueSimulatorRender(HyperelasticTongueSimulator* simulator) :SimulatorRenderBase(simulator)
{
	this->hyperTongueSImulator = simulator;
	tetvolumetricRender = new TetVolumetricMeshRender();
	mesh_color_index = 0;
}

HyperElasticTongueSimulatorRender::~HyperElasticTongueSimulatorRender()
{
}

void HyperElasticTongueSimulatorRender::renderSimulator(QOpenGLShaderProgram *program)
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
	{
		if (simulator->getVolumetricMesh() != NULL)
			this->tetvolumetricRender->renderAllVolumetricMesh(program, downCastTetVolMesh(simulator->getVolumetricMesh()), mesh_color_index);
	}

	drawSensorDataSphere(program, 1);

	drawForce(program);
}

void HyperElasticTongueSimulatorRender::paintSimulator(QPainter &painter)
{
	int width = painter.window().width();
	int height = painter.window().height();
	int start_x = width - 100;
	int start_y = height - 50;
	int offset_y = 0;

	std::ostringstream timestep;
	timestep << "Steps: " << this->hyperTongueSImulator->getSimulation_steps();
	painter.drawText(10, 40, timestep.str().c_str());

	std::ostringstream oss;
	oss.precision(3);
}

void HyperElasticTongueSimulatorRender::drawSensorDataSphere(QOpenGLShaderProgram* program, int colorindex)
{
	if (hyperTongueSImulator->getSimulatorState() == LoboSimulatorBase::beforeinit)
	{
		return;
	}

	TongueModel * tonguemode = hyperTongueSImulator->getTonguemodelRef();
	int numSensors = tonguemode->getNumSensor();
	//only draw first 4 sensors(on tongue); 
	numSensors = 4;
	for (int i = 0; i < numSensors; i++)
	{
		//Vector3d position = tonguemode->getSensorPosition(i, getRenderSensorDataTimeid());
		Vector3d position = tonguemode->getCurSensorNodePosition(i);
		sphere1->drawMesh(program, position, 0.01, i);
	}

}

HyperElasticTongueSimulatorRender* downCastHyperTongueSimulatorRender(SimulatorRenderBase* render)
{
	return (HyperElasticTongueSimulatorRender*)render;
}
