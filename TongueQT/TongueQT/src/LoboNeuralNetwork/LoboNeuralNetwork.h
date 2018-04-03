#pragma once

#include <Eigen/Dense>
#include <vector>
using namespace Eigen;

class LoboNeuralLayer;

class LoboNeuralNetwork
{
public:
	LoboNeuralNetwork();
	~LoboNeuralNetwork();

	virtual void initMyOwnDNN();

	virtual void addLayers(LoboNeuralLayer* layer);

	virtual void loadDNN(const char* filename);
	virtual void loadDNNV2(const char* filename , const char* activationfunction = NULL);

	virtual void convertToGLSL(const char* filename);

	virtual VectorXd predictV2(VectorXd &inputVectorXd);

	virtual VectorXd predict(VectorXd &inputVectorXd);

protected:

	int n_input;
	int n_output;
	int n_hiddenLayers;

	std::vector<LoboNeuralLayer*> DNN_layers;

	std::vector<MatrixXd> weights;
	std::vector<VectorXd> biases;
};

