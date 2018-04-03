#pragma once
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <vector>
using namespace Eigen;

class ExternalForceGenerator
{
public:
	ExternalForceGenerator(SparseMatrix<double>* massMatrix);
	~ExternalForceGenerator();

	virtual void generateForce(int numFrames, int r,std::vector<VectorXd> &forces);
	virtual void readconfig(const char* filename){};

protected:

	SparseMatrix<double>* massMatrix;

};