#pragma once
#include "CubaturePartition.h"

class CubatureOnePartition:public CubaturePartition
{
public:
	CubatureOnePartition(DomainDecomposition* domaindecompsition, LoboVolumetricMesh* volumetricMesh);
	~CubatureOnePartition();

	virtual void init();

protected:
	virtual void generatePartitionDOFs();

};

