#include "LoboStVKIsotropicMaterial.h"

LoboStVKIsotropicMaterial::LoboStVKIsotropicMaterial(TetVolumetricMesh* tetmesh, int enableCompressionResistance /*= 0*/, double compressionResistance /*= 0.0*/) :LoboIsotropicHyperelasticWithCompressionResistance(enableCompressionResistance)
{
	this->tetmesh = tetmesh;
	int numElements = tetmesh->getNumElements();
	lambdaLame = (double*)malloc(sizeof(double) * numElements);
	muLame = (double*)malloc(sizeof(double) * numElements);

	if (enableCompressionResistance)
		EdivNuFactor = (double*)malloc(sizeof(double) * numElements);
	else
		EdivNuFactor = NULL;

	for (int el = 0; el < numElements; el++)
	{
		LoboVolumetricMesh::Material * material = tetmesh->getElementMaterial(el);
		LoboVolumetricMesh::ENuMaterial * eNuMaterial = downcastENuMaterial(material);
		if (eNuMaterial == NULL)
		{
			printf("Error: NeoHookeanIsotropicMaterial: mesh does not consist of E, nu materials.\n");
			throw 1;
		}

		lambdaLame[el] = eNuMaterial->getLambda();
		muLame[el] = eNuMaterial->getMu();

		if (enableCompressionResistance)
		{
			EdivNuFactor[el] = compressionResistance * eNuMaterial->getE() / (1.0 - 2.0 * eNuMaterial->getNu());
			//printf("Setting EdivNuFactor[%d]=%G\n", el, EdivNuFactor[el]);
		}
	}

}

LoboStVKIsotropicMaterial::~LoboStVKIsotropicMaterial()
{
	free(EdivNuFactor);
	free(lambdaLame);
	free(muLame);
}

void LoboStVKIsotropicMaterial::updateMaterial()
{
	int numElements = tetmesh->getNumElements();

	for (int el = 0; el < numElements; el++)
	{
		LoboVolumetricMesh::Material * material = tetmesh->getElementMaterial(el);
		LoboVolumetricMesh::ENuMaterial * eNuMaterial = downcastENuMaterial(material);
		if (eNuMaterial == NULL)
		{
			printf("Error: NeoHookeanIsotropicMaterial: mesh does not consist of E, nu materials.\n");
			throw 1;
		}

		lambdaLame[el] = eNuMaterial->getLambda();
		muLame[el] = eNuMaterial->getMu();

		if (enableCompressionResistance)
		{
			EdivNuFactor[el] = compressionResistance * eNuMaterial->getE() / (1.0 - 2.0 * eNuMaterial->getNu());
			//printf("Setting EdivNuFactor[%d]=%G\n", el, EdivNuFactor[el]);
		}
	}
}

double LoboStVKIsotropicMaterial::ComputeEnergy(int elementIndex, double * invariants)
{
	double IC = invariants[0];
	double IIC = invariants[1];
	//double IIIC = invariants[2]; // not needed for StVK

	double energy = 0.125 * lambdaLame[elementIndex] * (IC - 3.0) * (IC - 3.0) + 0.25 * muLame[elementIndex] * (IIC - 2.0 * IC + 3.0);

	AddCompressionResistanceEnergy(elementIndex, invariants, &energy);

	return energy;
}

void LoboStVKIsotropicMaterial::ComputeEnergyGradient(int elementIndex, double * invariants, double * gradient)
{
	double IC = invariants[0];
	gradient[0] = 0.25 * lambdaLame[elementIndex] * (IC - 3.0) - 0.5 * muLame[elementIndex];
	gradient[1] = 0.25 * muLame[elementIndex];
	gradient[2] = 0.0;


	AddCompressionResistanceGradient(elementIndex, invariants, gradient);
}

void LoboStVKIsotropicMaterial::ComputeEnergyHessian(int elementIndex, double * invariants, double * hessian)
{
	// 11
	hessian[0] = 0.25 * lambdaLame[elementIndex];
	// 12
	hessian[1] = 0.0;
	// 13
	hessian[2] = 0.0;
	// 22
	hessian[3] = 0.0;
	// 23
	hessian[4] = 0.0;
	// 33
	hessian[5] = 0.0;

	AddCompressionResistanceHessian(elementIndex, invariants, hessian);
}

void LoboStVKIsotropicMaterial::getLambdaLame(int eleid, double &lambda)
{
	lambda = lambdaLame[eleid];
}

void LoboStVKIsotropicMaterial::getMuLame(int eleid, double &mu)
{
	mu = muLame[eleid];
}

double LoboStVKIsotropicMaterial::GetCompressionResistanceFactor(int elementIndex)
{
	return EdivNuFactor[elementIndex];
}
