#include "LoboNeuralNetwork.h"
#include <fstream>
#include <algorithm>
#include <iostream>
#include "LoboNeuralNetwork/Layer/LoboNeuralLayer.h"
#include "LoboNeuralNetwork/Layer/LoboLayersCore.h"
#include "Functions/deleteSTDvector.h"

LoboNeuralNetwork::LoboNeuralNetwork()
{

}


LoboNeuralNetwork::~LoboNeuralNetwork()
{
	deleteStdvectorPointer(DNN_layers);
}

void LoboNeuralNetwork::initMyOwnDNN()
{

}

void LoboNeuralNetwork::addLayers(LoboNeuralLayer* layer)
{
	DNN_layers.push_back(layer);
}

void LoboNeuralNetwork::loadDNN(const char* filename)
{
	weights.clear();
	biases.clear();

	std::ifstream instream(filename);

	int number_of_layers;
	instream >> number_of_layers;

	n_hiddenLayers = number_of_layers;

	for (int i = 0; i < number_of_layers; i++)
	{
		int x, y;
		instream >> x >> y; 
		MatrixXd weightMatrix(x, y);
		for (int j = 0; j < x; j++)
		{
			for (int k = 0; k < y; k++)
			{
				instream >> weightMatrix.data()[k*x + j];
			}
		}
		weights.push_back(weightMatrix);
	}

	for (int i = 0; i < number_of_layers; i++)
	{
		int x;
		instream >> x;
		VectorXd bias(x);

		for (int j = 0; j < x; j++)
		{
			instream >> bias.data()[j];
		}
		biases.push_back(bias);
	}

	instream.close();

	std::cout << "loaded DNN finished ..." << std::endl;
}

void LoboNeuralNetwork::loadDNNV2(const char* filename, const char* activationfunction)
{
	std::ifstream instream(filename);
	int number_of_layers;
	instream >> number_of_layers;
	for (int i = 0; i < number_of_layers; i++)
	{
		int number_of_paramter;
		instream >> number_of_paramter;

		//full connected layer
		if (number_of_paramter == 2)
		{
			int x, y;
			instream >> x >> y;
			MatrixXd weightMatrix(x, y);
			for (int j = 0; j < x; j++)
			{
				for (int k = 0; k < y; k++)
				{
					instream >> weightMatrix.data()[k*x + j];
				}
			}

			instream >> x;
			VectorXd bias(x);

			for (int j = 0; j < x; j++)
			{
				instream >> bias.data()[j];
			}
			addLayers(new FullConnectLayer(weightMatrix,bias));
		}

		//batch normlization layer
		if (number_of_paramter == 4)
		{
			int x;
			//beta gamma mean var
			VectorXd vect[4];

			for (int j = 0; j < 4; j++)
			{
				instream >> x;
				vect[j].resize(x);
				for (int k = 0; k < x; k++)
				{
					instream >> vect[j][k];
				}
			}
			addLayers(new BatchNormlizeLayer(vect[2], vect[3], vect[0], vect[1], 1e-10));
		}

		if (number_of_paramter == 0)
		{
			if (activationfunction == NULL)
				addLayers(new ActivationLayer("tanh"));
			else
				addLayers(new ActivationLayer(activationfunction));
		}
	}
}

void LoboNeuralNetwork::convertToGLSL(const char* filename)
{
	std::ofstream outputstream(filename);
	for (int i = 0; i < DNN_layers.size(); i++)
	{
		DNN_layers[i]->convertToGLSL(outputstream,i);
	}
	outputstream.close();
}

Eigen::VectorXd LoboNeuralNetwork::predictV2(VectorXd &inputVectorXd)
{
	VectorXd hiddenVector = inputVectorXd;
	VectorXd outputVector;

	for (int i = 0; i < DNN_layers.size(); i++)
	{
		DNN_layers[i]->layerRun(hiddenVector, outputVector);
		hiddenVector = outputVector;
	}
	return outputVector;
}

Eigen::VectorXd LoboNeuralNetwork::predict(VectorXd &inputVectorXd)
{
	VectorXd hiddenVector = inputVectorXd;
	for (int i = 0; i < n_hiddenLayers; i++)
	{

		VectorXd temp = weights[i].transpose()*hiddenVector + biases[i];
		hiddenVector = temp;

		if (i!=n_hiddenLayers-1)
		for (int j = 0; j < hiddenVector.size(); j++)
		{
			hiddenVector.data()[j] = std::max(0.0, hiddenVector.data()[j]);
		}
	}
	return hiddenVector;
}

