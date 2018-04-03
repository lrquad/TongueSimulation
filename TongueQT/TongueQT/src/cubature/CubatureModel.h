#pragma once
#include <fstream>

class ReducedSubspaceModel;
class ReducedSubspaceForceModel;
class LoboVolumetricMesh;
class ReducedCubatureTrainingOptimization;
class ReducedCubatureDataSet;
class CubaturePartition;
class DomainDecomposition;

class CubatureModel
{
public:
	CubatureModel(LoboVolumetricMesh* volumetricmesh, ReducedSubspaceModel* subspacemodel, ReducedSubspaceForceModel* forcemodel, DomainDecomposition* domaindecompsition);

	~CubatureModel();

	virtual void initCubatureModel();
	virtual void doCubatureTraining();

	virtual void readCubature(const char* filename);
	virtual void saveCubature(const char* filename);

	virtual void saveCubatureBinary(std::ofstream& outstream);
	virtual void readCubatureBinary(std::ifstream& instream);


	bool getUseNumPoses() const { return useNumPoses; }
	void setUseNumPoses(bool val) { useNumPoses = val; }
	int getSetNumPoses() const { return setNumPoses; }
	void setSetNumPoses(int val) { setNumPoses = val; }
protected:
	


	LoboVolumetricMesh* volumetricMesh;
	ReducedSubspaceForceModel* forcemodel;
	ReducedSubspaceModel* subspacemodel;

	ReducedCubatureTrainingOptimization* reducedCubatrueTraining;
	ReducedCubatureDataSet* cubaturedataset;
	CubaturePartition* cubaturepartition;
	DomainDecomposition* domaindecompsition;

	bool useNumPoses;
	int setNumPoses;


};

