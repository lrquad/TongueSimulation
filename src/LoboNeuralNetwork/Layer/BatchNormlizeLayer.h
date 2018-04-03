#pragma once
#include "LoboNeuralNetwork/Layer/LoboNeuralLayer.h"


class BatchNormlizeLayer :public LoboNeuralLayer
{
public:
	BatchNormlizeLayer(VectorXd Mean_, VectorXd Var_, VectorXd Gamma_, VectorXd Beta_, double Epsilon_);
	~BatchNormlizeLayer();

	virtual void layerRun(VectorXd &input, VectorXd &output);

protected:

	VectorXd mean_;
	VectorXd var_;
	VectorXd gamma_;
	VectorXd beta_;
	double epsilon_;
};

