#pragma once
#include "Simulator/simulatorBase/simulatorRenderBase.h"

class TetVolumetricMeshRender;
class FloatingMDSTVKSimulator;

class FloatingMDSTVKSimulatorRender:public SimulatorRenderBase
{
public:
	FloatingMDSTVKSimulatorRender(FloatingMDSTVKSimulator* simualtor);
	~FloatingMDSTVKSimulatorRender();

	virtual void renderSimulator(QOpenGLShaderProgram *program);
	virtual void paintSimulator(QPainter &painter);


protected:
	FloatingMDSTVKSimulator* floatingMDSTVKSimulator;
	TetVolumetricMeshRender* tetvolumetricRender;

};

FloatingMDSTVKSimulatorRender* downCastFloatingMDSTVKSimulatorRender(SimulatorRenderBase* render);