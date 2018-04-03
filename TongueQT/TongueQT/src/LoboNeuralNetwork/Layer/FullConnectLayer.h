#pragma once
#include "LoboNeuralNetwork/Layer/LoboNeuralLayer.h"

class FullConnectLayer :public LoboNeuralLayer
{
public:
	FullConnectLayer(MatrixXd weights_,VectorXd bias_);
	~FullConnectLayer(); 

	virtual void layerRun(VectorXd &input, VectorXd &output);

	virtual void convertToGLSL(std::ofstream &outputstream, int layerid);

	MatrixXd weights;
	VectorXd bias;

protected:

	

};

