#pragma once
#include "FunnelForceGenerator.h"
class StartForceFieldGenerator:public FunnelForceGenerator
{
public:
	StartForceFieldGenerator(SparseMatrix<double>* massMatrix, LoboVolumetricMesh* volumetricmesh, VectorXd fullgravity);
	~StartForceFieldGenerator();

	virtual void readconfig(const char* filename);

	virtual void generateForce(int r, std::vector<VectorXd> &forces, MatrixXd* subspacePhi = NULL);

protected:
	Vector3d forceField;
};

