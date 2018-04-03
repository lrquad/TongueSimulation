#pragma once
#include "ExternalForceGenerator.h"
class LoboVolumetricMesh;

class CactusBendingForce:public ExternalForceGenerator
{
public:

	CactusBendingForce(SparseMatrix<double>* massMatrix, LoboVolumetricMesh* volumetricmesh);
	~CactusBendingForce();


	virtual void generateForce(int r, std::vector<VectorXd> &forces, MatrixXd* subspacePhi = NULL);
	virtual void readconfig(const char* filename);

protected:
	LoboVolumetricMesh* volumetricmesh;
	int numFrame;
	int leftframestart;
	int leftframeend;

	int rightframestart;
	int rightframeend;
	Vector3d nodeforce;
};

