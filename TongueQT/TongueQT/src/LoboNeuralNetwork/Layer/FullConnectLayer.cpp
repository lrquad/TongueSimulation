#include "FullConnectLayer.h"
#include <iostream>


FullConnectLayer::FullConnectLayer(MatrixXd weights_, VectorXd bias_)
{
	std::cout << "fullConnectLayer = > " << weights_.rows()<<" "<< weights_.cols() << std::endl;
	this->weights = weights_.transpose();
	this->bias = bias_;
}

FullConnectLayer::~FullConnectLayer()
{
}

void FullConnectLayer::layerRun(VectorXd &input, VectorXd &output)
{
	output = weights*input + bias;
}

void FullConnectLayer::convertToGLSL(std::ofstream &outputstream, int layerid)
{
	for (int i = 0; i < weights.rows(); i++)
	{
		std::ostringstream data;
		data << "a_" << layerid<<"_"<< i;
		std::string varname = data.str();

		outputstream <<"const float "<< varname <<"[" << weights.cols()<<"] = float[](";

		for (int j = 0; j < weights.cols(); j++)
		{
			outputstream << weights.data()[j*weights.rows() + i];
			if (j != weights.cols()-1)
			outputstream << ",";
		}
		outputstream << ");" << std::endl;
	}

	std::ostringstream bdata;
	bdata << "b_" << layerid << "_" << 0;
	std::string bvarname = bdata.str();

	outputstream << "const float " << bvarname << "[" << bias.rows() << "] = float[](";

	for (int j = 0; j < bias.rows(); j++)
	{
		outputstream << bias.data()[j];
		if (j != bias.rows() - 1)
			outputstream << ",";
	}
	outputstream << ");" << std::endl;

	//compute part

	//input
	std::string inputVectorvarname = "";
	if (layerid == 0)
	{
		std::ostringstream inputVector;
		inputVector << "in_" << layerid << "_" << 0;
		inputVectorvarname = inputVector.str();

		outputstream << "float " << inputVectorvarname << "[" << weights.cols() << "] = float[](";
		for (int i = 0; i < weights.cols(); i++)
		{
			outputstream << "0";
			if (i != weights.cols() - 1)
			{
				outputstream << ",";
			}
		}
		outputstream << ");" << std::endl;
	}
	else
	{
		std::ostringstream inputVector;
		inputVector << "out_" << layerid-2 << "_" << 0;
		inputVectorvarname = inputVector.str();

		outputstream << "float " << inputVectorvarname << "[" << weights.cols() << "] = float[](";
		for (int i = 0; i < weights.cols(); i++)
		{
			outputstream << "0";
			if (i != weights.cols() - 1)
			{
				outputstream << ",";
			}
		}
		outputstream << ");" << std::endl;

	}


	//output
	std::ostringstream outputVector;
	outputVector << "out_" << layerid << "_" << 0;
	std::string outputVectorvarname = outputVector.str();

	for (int i = 0; i < weights.rows(); i++)
	{
		std::ostringstream data;
		data << "a_" << layerid << "_" << i;
		std::string varname = data.str();

		outputstream << outputVectorvarname << "[" << i << "] = ";
		for (int j = 0; j < weights.cols(); j++)
		{
			outputstream << varname << "[" << j << "]" << "*" << inputVectorvarname << "[" << j << "]" <<"+";
		}

		outputstream << bvarname << "[" << i << "]";
		outputstream << ";" << std::endl;
	}


	//activation
	for (int i = 0; i < weights.rows(); i++)
	{
		outputstream << outputVectorvarname << "[" << i << "] = tanh ("
			<< outputVectorvarname << "[" << i << "]);" << std::endl;
	}


}


