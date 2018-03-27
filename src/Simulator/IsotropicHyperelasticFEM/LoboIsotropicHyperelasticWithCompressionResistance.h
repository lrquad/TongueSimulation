#pragma once

#include "LoboIsotropicMaterial.h"

class LoboIsotropicHyperelasticWithCompressionResistance:
	public LoboIsotropicMaterial
{
public:
	LoboIsotropicHyperelasticWithCompressionResistance(bool enableCompressionResistance = 0);
	~LoboIsotropicHyperelasticWithCompressionResistance();

	virtual void updateMaterial() = 0;

	virtual double ComputeEnergy(int elementIndex, double * invariants) = 0;
	virtual void ComputeEnergyGradient(int elementIndex, double * invariants, double * gradient) = 0; // invariants and gradient are 3-vectors
	virtual void ComputeEnergyHessian(int elementIndex, double * invariants, double * hessian) = 0; // invariants is a 3-vector, hessian is a 3x3 symmetric matrix, unrolled into a 6-vector, in the following order: (11, 12, 13, 22, 23, 33).

protected:


	bool enableCompressionResistance;

	virtual double GetCompressionResistanceFactor(int elementIndex);

	void AddCompressionResistanceEnergy(int elementIndex, double * invariants, double * energy);
	void AddCompressionResistanceGradient(int elementIndex, double * invariants, double * gradient);
	void AddCompressionResistanceHessian(int elementIndex, double * invariants, double * hessian);

};


