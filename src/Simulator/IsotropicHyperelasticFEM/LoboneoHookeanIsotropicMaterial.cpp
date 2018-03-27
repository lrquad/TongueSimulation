#include "LoboneoHookeanIsotropicMaterial.h"

LoboneoHookeanIsotropicMaterial::LoboneoHookeanIsotropicMaterial(TetVolumetricMesh* tetmesh, int enableCompressionResistance /*= 0*/, double compressionResistance /*= 0.0*/) :LoboIsotropicHyperelasticWithCompressionResistance(enableCompressionResistance)
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

LoboneoHookeanIsotropicMaterial::~LoboneoHookeanIsotropicMaterial()
{
	free(EdivNuFactor);
	free(lambdaLame);
	free(muLame);
}

void LoboneoHookeanIsotropicMaterial::updateMaterial()
{
	std::cout << "updateMaterial  " << std::endl;
	int numElements = tetmesh->getNumElements();

	LoboVolumetricMesh::Material* materia = tetmesh->getMaterialById(0);
	LoboVolumetricMesh::ENuMaterial* enmateria = (LoboVolumetricMesh::ENuMaterial*)materia;
	std::cout << enmateria->getNu() << std::endl;

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

double LoboneoHookeanIsotropicMaterial::ComputeEnergy(int elementIndex, double * invariants)
{
	double IC = invariants[0];
	double IIIC = invariants[2];
	double J = sqrt(IIIC);
	double logJ = log(J);

	// Note: computation of J and logJ will fail for an inverted element.
	// The IsotropicHyperelasticFEM class will prevent inversions (assuming proper
	// threshold was set), so normally this is not an issue.

	double energy = 0.5 * muLame[elementIndex] * (IC - 3.0) - muLame[elementIndex] * logJ + 0.5 * lambdaLame[elementIndex] * logJ * logJ;

	AddCompressionResistanceEnergy(elementIndex, invariants, &energy);

	return energy;
}

void LoboneoHookeanIsotropicMaterial::ComputeEnergyGradient(int elementIndex, double * invariants, double * gradient)
{
	//printf("Entered NeoHookeanIsotropicMaterial::ComputeEnergyGradient\n");

	double IIIC = invariants[2];
	gradient[0] = 0.5 * muLame[elementIndex];
	gradient[1] = 0.0;
	gradient[2] = (-0.5 * muLame[elementIndex] + 0.25 * lambdaLame[elementIndex] * log(IIIC)) / IIIC;

	AddCompressionResistanceGradient(elementIndex, invariants, gradient);
}

void LoboneoHookeanIsotropicMaterial::ComputeEnergyHessian(int elementIndex, double * invariants, double * hessian)
{
	double IIIC = invariants[2];
	// 11
	hessian[0] = 0.0;
	// 12
	hessian[1] = 0.0;
	// 13
	hessian[2] = 0.0;
	// 22
	hessian[3] = 0.0;
	// 23
	hessian[4] = 0.0;
	// 33
	hessian[5] = (0.25 * lambdaLame[elementIndex] + 0.5 * muLame[elementIndex] - 0.25 * lambdaLame[elementIndex] * log(IIIC)) / (IIIC * IIIC);

	AddCompressionResistanceHessian(elementIndex, invariants, hessian);
}

void LoboneoHookeanIsotropicMaterial::getLambdaLame(int eleid, double &lambda)
{
	lambda = lambdaLame[eleid];
}

void LoboneoHookeanIsotropicMaterial::getMuLame(int eleid, double &mu)
{
	mu = muLame[eleid];
}

double LoboneoHookeanIsotropicMaterial::GetCompressionResistanceFactor(int elementIndex)
{
	return EdivNuFactor[elementIndex];
}
