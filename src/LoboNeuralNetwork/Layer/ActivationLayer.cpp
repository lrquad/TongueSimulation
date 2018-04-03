#include "ActivationLayer.h"
#include <iostream>
ActivationLayer::ActivationLayer(const char* activationtype_)
{
	if (strcmp(activationtype_, "tanh") == 0)
	{
		activationtype = ActivationLayer::tanh;
	}
	else if (strcmp(activationtype_,"relu") == 0)
	{
		activationtype = ActivationLayer::relu;
	}
	std::cout << "activation => " << activationtype_ << std::endl;
}

ActivationLayer::~ActivationLayer()
{
}

void ActivationLayer::layerRun(VectorXd &input, VectorXd &output)
{
	output.resize(input.rows());
	for (int i = 0; i < input.rows(); i++)
	{
		switch (activationtype)
		{
		case ActivationLayer::tanh:
			//output.data()[i] = tanhFunction(input.data()[i]);
			output.data()[i] = std::tanh(input.data()[i]);
			break;
		case ActivationLayer::relu:
			//output.data()[i] = reluFunction(input.data()[i]);
			output.data()[i] = input.data()[i] > 0 ? input.data()[i] : 0;
		default:
			break;
		}
	}

}

double ActivationLayer::tanhFunction(double input)
{
	double temp = std::tanh(input);
	return temp;
}

double ActivationLayer::reluFunction(double input)
{
	double temp = input > 0 ? input : 0;
	return temp;
}
