#pragma once
#include "CubaturePartition.h"

class CubatureDomainPartition:public CubaturePartition
{
public:
	CubatureDomainPartition(DomainDecomposition* domaindecompsition, LoboVolumetricMesh* volumetricMesh);
	~CubatureDomainPartition();

	virtual void init();
};

