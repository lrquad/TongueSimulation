#pragma once
#include "DomainDecomposition.h"


class DomainDecompositionDefault:public DomainDecomposition
{
public:
	DomainDecompositionDefault(LoboVolumetricMesh* volumetricmesh, std::vector<int> seeds_index, bool createSubVolumetricMesh = false);
	~DomainDecompositionDefault();

	virtual void doDecomposition();

};

