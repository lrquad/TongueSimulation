#pragma once
#include "DomainDecompositionFramebased.h"

class DomainDecompositionPartition: public DomainDecompositionFramebased
{
public:
	DomainDecompositionPartition(LoboVolumetricMesh* volumetricmesh, std::vector<int> seeds_index, WeightFunctionLaplacian* weightfunction, bool createSubVolumetricMesh = false);

	DomainDecompositionPartition(LoboVolumetricMesh* volumetricmesh, LoboVolumetricMeshGraph* volumetricmeshgraph, std::vector<int> seeds_index, WeightFunctionLaplacian* weightfunction, bool createSubVolumetricMesh = false);


	~DomainDecompositionPartition();

	virtual void doDecomposition();
	virtual void createDomainConstraints();

protected:

	virtual void fillDomain();



};

