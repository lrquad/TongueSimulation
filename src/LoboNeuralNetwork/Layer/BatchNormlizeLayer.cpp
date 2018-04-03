#include "BatchNormlizeLayer.h"
#include <vector>
#include <iostream>


BatchNormlizeLayer::BatchNormlizeLayer(VectorXd Mean_, VectorXd Var_, VectorXd Gamma_, VectorXd Beta_, double Epsilon_)
{
	std::cout << "batch normlize layer" << std::endl;
	this->gamma_ = Gamma_;
	this->beta_ = Beta_;
	this->mean_ = Mean_;
	this->var_ = Var_;
	this->epsilon_ = Epsilon_;
}

BatchNormlizeLayer::~BatchNormlizeLayer()
{

}

void BatchNormlizeLayer::layerRun(VectorXd &input, VectorXd &output)
{
	output.resize(input.size());
	output.setZero();

	for (int i = 0; i < input.size(); i++)
	{
		double temp = (input.data()[i] - mean_[i]);
		double temp2 = std::sqrt(var_[i] + epsilon_);

		double z1_hat = temp / temp2;
		output.data()[i] = gamma_[i]*z1_hat + beta_.data()[i];
	}
}
