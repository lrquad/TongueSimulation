#pragma once
#include "DomainDecompositionPartitionOnly.h"

class DomainDecompositionUserDefine:public DomainDecompositionPartitionOnly
{

public:
	DomainDecompositionUserDefine(LoboVolumetricMesh* volumetricmesh, LoboVolumetricMeshGraph* volumetricmeshgraph, std::vector<int> seeds_index, WeightFunctionLaplacian* weightfunction = NULL, bool createSubVolumetricMesh = false);
	~DomainDecompositionUserDefine();

	virtual void markNewDomain(std::vector<int> elelist);
	virtual void markEleToExistDomain(std::vector<int> elelist,int targetdomain);

	virtual int getDomainidByElement(int eleid);

	virtual void doDecomposition();

	virtual void saveDomainMarks(const char* filename);
	virtual void readDomainMarks(const char* filename);


	std::vector<int> getelementDomainMakr();



protected:


	virtual void checkDomainConnection();


	std::vector<int> elementDomainMark;

};

