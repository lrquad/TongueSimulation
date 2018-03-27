#pragma once
#include "LoboIsotropicHyperelasticWithCompressionResistance.h"
#include "LoboVolumetricMesh/LoboVolumetriceMeshCore.h"

class LoboStVKIsotropicMaterial:public LoboIsotropicHyperelasticWithCompressionResistance
{
public:
	LoboStVKIsotropicMaterial(TetVolumetricMesh* tetmesh, int enableCompressionResistance = 0, double compressionResistance = 0.0);
	~LoboStVKIsotropicMaterial();

	virtual void updateMaterial();


	virtual double ComputeEnergy(int elementIndex, double * invariants);
	virtual void ComputeEnergyGradient(int elementIndex, double * invariants, double * gradient); // invariants and gradient are 3-vectors
	virtual void ComputeEnergyHessian(int elementIndex, double * invariants, double * hessian); // invariants is a 3-vector, hessian is a 3x3 symmetric matrix, unrolled into a 6-vector, in the following order: (11, 12, 13, 22, 23, 33).

	virtual void getLambdaLame(int eleid, double &lambda);
	virtual void getMuLame(int eleid, double &mu);

protected:

	double* lambdaLame;
	double* muLame;

	double compressionResistance;
	double * EdivNuFactor;
	virtual double GetCompressionResistanceFactor(int elementIndex);

	TetVolumetricMesh* tetmesh;

};

