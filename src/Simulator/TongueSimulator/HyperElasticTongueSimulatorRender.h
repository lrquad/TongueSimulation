#pragma once
#include "simulator/simulatorBase/simulatorRenderBase.h"

class HyperelasticTongueSimulator;

class TetVolumetricMeshRender;

class HyperElasticTongueSimulatorRender: public SimulatorRenderBase
{
public:
	HyperElasticTongueSimulatorRender(HyperelasticTongueSimulator* simulator);
	~HyperElasticTongueSimulatorRender();

	virtual void renderSimulator(QOpenGLShaderProgram *program);
	virtual void paintSimulator(QPainter &painter);


protected:

	void drawSensorDataSphere(QOpenGLShaderProgram* program, int colorindex);

	HyperelasticTongueSimulator* hyperTongueSImulator;
	TetVolumetricMeshRender* tetvolumetricRender;

};


HyperElasticTongueSimulatorRender* downCastHyperTongueSimulatorRender(SimulatorRenderBase* render);