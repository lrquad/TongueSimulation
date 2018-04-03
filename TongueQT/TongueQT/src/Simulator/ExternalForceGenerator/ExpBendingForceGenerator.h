#pragma once
#include "BendingForceGenerator.h"
class ExpBendingForceGenerator:public BendingForceGenerator
{
public:
	ExpBendingForceGenerator(SparseMatrix<double>* massMatrix, LoboVolumetricMesh* volumetricmesh);
	~ExpBendingForceGenerator();

	virtual void generateForce(int numFrames, int r, std::vector<VectorXd> &forces);

};

