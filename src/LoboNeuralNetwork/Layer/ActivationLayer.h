#pragma once
#include "LoboNeuralNetwork/Layer/LoboNeuralLayer.h"



class ActivationLayer:public LoboNeuralLayer
{
public:
	ActivationLayer(const char* activationtype_);

	~ActivationLayer();

	typedef enum { tanh,relu } ActivationType;

	virtual void layerRun(VectorXd &input, VectorXd &output);




protected:

	double tanhFunction(double input);
	double reluFunction(double input);


	ActivationType activationtype;


};

