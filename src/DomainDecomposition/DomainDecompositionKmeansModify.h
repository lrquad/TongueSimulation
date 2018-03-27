#pragma once
#include "DomainDecompositionKmeans.h"

class DomainDecompositionKmeansModify:public DomainDecompositionKmeans
{
public:
	DomainDecompositionKmeansModify(LoboVolumetricMesh* volumetricmesh, LoboVolumetricMeshGraph* volumetricmesh_graph, int numConstrainedDOFs, int* constraintedDOFS, int k,std::vector<int> seeds);
	~DomainDecompositionKmeansModify();

	virtual void doDecomposition();

protected:
	std::vector<int> given_seeds;

};

