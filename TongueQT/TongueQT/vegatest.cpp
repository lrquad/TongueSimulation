#include "vegatest.h"
#include "Vega/libraries/stvk/StVKElementABCD.h"
#include "Vega/libraries/stvk/StVKTetABCD.h"
#include "Vega/libraries/stvk/StVKElementABCDLoader.h"
#include "Vega/libraries/stvk/StVKInternalForces.h"
#include "Vega/libraries/stvk/StVKStiffnessMatrix.h"
#include "Vega/libraries/isotropicHyperelasticFEM/isotropicMaterial.h"
#include "Vega/libraries/isotropicHyperelasticFEM/StVKIsotropicMaterial.h"
#include "Vega/libraries/isotropicHyperelasticFEM/neoHookeanIsotropicMaterial.h"
#include "Vega/libraries/isotropicHyperelasticFEM/isotropicHyperelasticFEM.h"
#include "Vega/libraries/elasticForceModel/isotropicHyperelasticFEMForceModel.h"
#include "volumetricMesh.h"
#include "volumetricMeshLoader.h"
#include <fstream>
#include <QElapsedTimer>

#include <iostream>

Vegatest::Vegatest(const char* volumtricmeshfile, double young, double possion, double density)
{
	int addGravity = 0;
	double g = 9.8;

	VolumetricMesh::fileFormatType fileFormat = VolumetricMesh::ASCII;
	int verbose = 1;
	volumetricMesh = VolumetricMeshLoader::load(volumtricmeshfile, fileFormat, verbose);
	volumetricMesh->setSingleMaterial(young, possion, density);
	

	unsigned int loadingFlag = 0; // 0 = use the low-memory version, 1 = use the high-memory version
	precomputedIntegrals = StVKElementABCDLoader::load(volumetricMesh, loadingFlag);

	stVKInternalForces = new StVKInternalForces(volumetricMesh, precomputedIntegrals, addGravity, g);

	stVKStiffnessMatrix = new StVKStiffnessMatrix(stVKInternalForces);


	tetMesh = dynamic_cast<TetMesh*>(volumetricMesh);
	isotropicMaterial = NULL;
	//isotropicMaterial = new StVKIsotropicMaterial(tetMesh,1,500);
	isotropicMaterial = new NeoHookeanIsotropicMaterial(tetMesh, 1, 500);

	isotropicHyperelasticFEM = new IsotropicHyperelasticFEM(tetMesh, isotropicMaterial, 0.5, addGravity, g);
	isotropicHyperelasticFEMForceModel = new IsotropicHyperelasticFEMForceModel(isotropicHyperelasticFEM);



}

Vegatest::~Vegatest()
{
	delete isotropicHyperelasticFEMForceModel;
	delete isotropicHyperelasticFEM;
	delete isotropicMaterial;
	delete tetMesh;
	delete stVKStiffnessMatrix;
	delete stVKInternalForces;
	delete precomputedIntegrals;
	delete volumetricMesh;
}

void Vegatest::test(double * displacement)
{
	int numVertex = volumetricMesh->getNumVertices();
	int r = numVertex * 3;
	
	VectorXd internalforce(r);
	internalforce.setZero();

	if (displacement != NULL)
	{
		QElapsedTimer timer;
		timer.start();
		isotropicHyperelasticFEMForceModel->GetInternalForce(displacement, internalforce.data());
		SparseMatrix * tangentStiffnessMatrix;
		isotropicHyperelasticFEMForceModel->GetTangentStiffnessMatrixTopology(&tangentStiffnessMatrix);
		isotropicHyperelasticFEMForceModel->GetTangentStiffnessMatrix(displacement, tangentStiffnessMatrix);
		std::cout << "vega " << timer.nsecsElapsed()*1e-6 << std::endl;
		tangentStiffnessMatrix->SaveToMatlabFormat("test.txt");
		
		delete tangentStiffnessMatrix;
	}
}