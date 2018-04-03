#include "CubatureOnePartition.h"
#include "DomainDecomposition/DomainDecomposition.h"
#include "LoboVolumetricMesh/LoboVolumetricMesh.h"

CubatureOnePartition::CubatureOnePartition(DomainDecomposition* domaindecompsition, LoboVolumetricMesh* volumetricMesh) :CubaturePartition(domaindecompsition, volumetricMesh)
{

}


CubatureOnePartition::~CubatureOnePartition()
{

}

void CubatureOnePartition::init()
{
	int numVertex = volumetricMesh->getNumVertices();
	int numElement = volumetricMesh->getNumElements();

	int numDomains = 1;
	numPartition = 1;
	std::vector<int> flags(numDomains);
	std::fill(flags.begin(), flags.end(), 1);

	partitionFlag.push_back(flags);
	partitionIndices.resize(numPartition);
	for (int i = 0; i < numVertex; i++)
	{
		partitionIndices[0].push_back(i);
	}

	partitionEleIndices.resize(numPartition);
	for (int i = 0; i < numElement; i++)
	{
		partitionEleIndices[0].push_back(i);
	}

	generatePartitionDOFs();
}

void CubatureOnePartition::generatePartitionDOFs()
{
	int numDomains = 1;

	int numElements = volumetricMesh->getNumElements();
	int numVertex = volumetricMesh->getNumVertices();

	partitionDOFs.resize(numPartition);
	for (int i = 0; i < numPartition; i++)
	{
		for (int j = 0; j < numDomains; j++)
		{
			if (partitionFlag[i][j] == 1)
			{
				partitionDOFs[i].push_back(j);
			}
		}
	}
}
