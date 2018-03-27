#pragma once
#include "DomainDecompositionPartition.h"

class DomainDecompositionPartitionOnly:public DomainDecompositionPartition
{
public:
	DomainDecompositionPartitionOnly(LoboVolumetricMesh* volumetricmesh, std::vector<int> seeds_index, WeightFunctionLaplacian* weightfunction, bool createSubVolumetricMesh = false);

	DomainDecompositionPartitionOnly(LoboVolumetricMesh* volumetricmesh, LoboVolumetricMeshGraph* volumetricmeshgraph, std::vector<int> seeds_index, WeightFunctionLaplacian* weightfunction, bool createSubVolumetricMesh = false);

	~DomainDecompositionPartitionOnly();

	virtual void doDecomposition();
	virtual void createDomainConstraints();

protected:

	virtual void fillDomain();

};

