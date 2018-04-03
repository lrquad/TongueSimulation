#pragma once
#include <Eigen/Dense>
using namespace Eigen;
#include <fstream>
class LoboNeuralLayer
{
public:
	LoboNeuralLayer();
	~LoboNeuralLayer();

	virtual void layerRun(VectorXd &input, VectorXd &output) = 0;

	virtual void convertToGLSL(std::ofstream &outputstream, int layerid){};


protected:



};

