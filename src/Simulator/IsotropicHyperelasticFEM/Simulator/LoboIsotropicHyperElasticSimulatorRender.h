#pragma once
#include "Simulator/simulatorBase/simulatorRenderBase.h"


class TetVolumetricMeshRender;
class LoboIsotropicHyperlasticSimulator;

class LoboIsotropicHyperElasticSimulatorRender:public SimulatorRenderBase
{
public:
	LoboIsotropicHyperElasticSimulatorRender(LoboIsotropicHyperlasticSimulator* simulator);
	~LoboIsotropicHyperElasticSimulatorRender();

	virtual void renderSimulator(QOpenGLShaderProgram *program);
	virtual void paintSimulator(QPainter &painter);


protected:
	LoboIsotropicHyperlasticSimulator* mysimulator;
	TetVolumetricMeshRender* tetvolumetricRender;

};


LoboIsotropicHyperElasticSimulatorRender* downCastIsotropicHyperElasticRender(SimulatorRenderBase* render);