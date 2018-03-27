#include "CubatureDomainPartition.h"
#include "DomainDecomposition/DomainDecomposition.h"
#include "LoboVolumetricMesh/LoboVolumetricMesh.h"


CubatureDomainPartition::CubatureDomainPartition(DomainDecomposition* domaindecompsition, LoboVolumetricMesh* volumetricMesh) :CubaturePartition(domaindecompsition,volumetricMesh)
{
	
}

CubatureDomainPartition::~CubatureDomainPartition()
{
}

void CubatureDomainPartition::init()
{
	int numDomains = domaindecompsition->getNumDomain();
	int numVertex = volumetricMesh->getNumVertices();
	int numElement = volumetricMesh->getNumElements();

	numPartition = domaindecompsition->getNumDomain();
	


}
