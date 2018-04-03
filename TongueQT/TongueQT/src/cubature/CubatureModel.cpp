#include "CubatureModel.h"
#include "cubatureCore.h"
#include "Simulator/ReducedForceModel/ReducedSubspaceForceModel.h"
#include "Simulator/ReducedSubspaceModel/ReducedSubspaceModel.h"
#include "LoboVolumetricMesh/LoboVolumetriceMeshCore.h"
#include <fstream>
#include "DomainDecomposition/DomainDecomposition.h"

CubatureModel::CubatureModel(LoboVolumetricMesh* volumetricmesh, ReducedSubspaceModel* subspacemodel, ReducedSubspaceForceModel* forcemodel, DomainDecomposition* domaindecompsition)
{
	this->volumetricMesh = volumetricmesh;
	this->subspacemodel = subspacemodel;
	this->forcemodel = forcemodel;
	this->domaindecompsition = domaindecompsition;

	this->setSetNumPoses(250);

	cubaturepartition = new CubatureOnePartition(domaindecompsition, volumetricMesh);
	cubaturepartition->init();

	cubaturedataset = new ReducedCubatureDataSet(200, forcemodel, cubaturepartition, subspacemodel->getR(), 1.0);
	cubaturedataset->setIsRigidModes(false);

	reducedCubatrueTraining = new ReducedCubatureTrainingOptimization(forcemodel, volumetricMesh, cubaturedataset, cubaturepartition);
}

CubatureModel::~CubatureModel()
{
}

void CubatureModel::initCubatureModel()
{

}

void CubatureModel::doCubatureTraining()
{
	subspacemodel->setUsegravity(false);
	cubaturedataset->setNumPoses(getSetNumPoses());

	std::cout << "numPose: " << this->getSetNumPoses() << std::endl;

	cubaturedataset->generateDataSet();
	std::cout << "generated data" << std::endl;

	cubaturedataset->computeInternalForce();
	
	reducedCubatrueTraining->setThreshold(0.08);
	reducedCubatrueTraining->trainingData();

	subspacemodel->setReducedCubatureTraining(reducedCubatrueTraining);
	subspacemodel->setUseCubature(true);
	subspacemodel->setUsegravity(true);

}

void CubatureModel::readCubature(const char* filename)
{
	reducedCubatrueTraining->readCubatures(filename);
	subspacemodel->setReducedCubatureTraining(reducedCubatrueTraining);
}

void CubatureModel::saveCubature(const char* filename)
{
	reducedCubatrueTraining->saveCubatures(filename);
}

void CubatureModel::saveCubatureBinary(std::ofstream& outstream)
{
	reducedCubatrueTraining->saveCubaturesBinary(outstream);
}

void CubatureModel::readCubatureBinary(std::ifstream& instream)
{
	reducedCubatrueTraining->readCubaturesBinary(instream);
	subspacemodel->setReducedCubatureTraining(reducedCubatrueTraining);
}
