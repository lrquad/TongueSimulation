#pragma once
#include "Simulator/simulatorBase/simulatorRenderBase.h"

class TetVolumetricMeshRender;
class ReducedTetSTVKSimulator;

class ReducedTetSTVKSimulatorRender:public SimulatorRenderBase
{
public:

	 ReducedTetSTVKSimulatorRender(ReducedTetSTVKSimulator* simulator);
	~ReducedTetSTVKSimulatorRender();

	virtual void renderSimulator(QOpenGLShaderProgram *program);
	virtual void paintSimulator(QPainter &painter);

protected:

	ReducedTetSTVKSimulator* reducedtetSTVKSimulator;
	TetVolumetricMeshRender* tetvolumetricRender;


};


ReducedTetSTVKSimulatorRender* downCastRedcuedSTVKSimulatoRender(SimulatorRenderBase* render);
