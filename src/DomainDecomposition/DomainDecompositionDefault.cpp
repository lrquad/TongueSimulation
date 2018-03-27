#include "DomainDecompositionDefault.h"
#include "LoboVolumetricMesh/LoboVolumetriceMeshCore.h"


DomainDecompositionDefault::DomainDecompositionDefault(LoboVolumetricMesh* volumetricmesh, std::vector<int> seeds_index, bool createSubVolumetricMesh /*= false*/) :DomainDecomposition(volumetricmesh,seeds_index,createSubVolumetricMesh)
{

}

DomainDecompositionDefault::~DomainDecompositionDefault()
{
}

void DomainDecompositionDefault::doDecomposition()
{

}
