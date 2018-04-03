#pragma once
#include <Eigen/Dense>
#include <Eigen/Sparse>
using namespace Eigen;

class LoboVolumetricMesh;
class TetElement;
class ReducedModesCoefficients
{
public:
	ReducedModesCoefficients(MatrixXd* modes, LoboVolumetricMesh* volumetric_mesh, MatrixXd* linearCoef, MatrixXd* quadraticCoef, MatrixXd* cubicCoef, double *lambdaLame_, double *muLame_, bool verbose = false);
	~ReducedModesCoefficients();

	int getLinearSize(){ return linearSize; };
	int getQuadraticSize(){ return quadraticSize; };
	int getCubicSize(){ return cubicSize; }



protected:

	void computeSubspaceCoefficients(TetElement& te, int r, MatrixXd *modes = NULL);

	int r;

	int linearSize;
	int quadraticSize;
	int cubicSize;

	MatrixXd* modes_;

	MatrixXd* linearCoef_;
	MatrixXd* quadraticCoef_;
	MatrixXd* cubicCoef_;

	LoboVolumetricMesh* volumetric_mesh_;

	double *lambdaLame;
	double *muLame;
};

