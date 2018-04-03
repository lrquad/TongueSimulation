#pragma once
#include "ExternalForceGenerator.h"
#include <Eigen/Geometry>

class LoboVolumetricMesh;

class BendingForceGenerator:public ExternalForceGenerator
{
public:
	BendingForceGenerator(SparseMatrix<double>* massMatrix, LoboVolumetricMesh* volumetricmesh);
	~BendingForceGenerator();

	virtual void generateForce(int numFrames, int r, std::vector<VectorXd> &forces);

protected:
	LoboVolumetricMesh* volumetricmesh;
};

