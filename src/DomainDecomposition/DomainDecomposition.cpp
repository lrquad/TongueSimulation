#include "DomainDecomposition.h"
#include "LoboVolumetricMesh\LoboVolumetricMesh.h"

DomainDecomposition::DomainDecomposition(LoboVolumetricMesh* volumetricmesh, std::vector<int> seeds_index, bool createSubVolumetricMesh)
{
	volumetricmesh_graph = NULL;
	this->volumetricmesh = volumetricmesh;
	this->seeds_index = seeds_index;
	this->createSubVolumetricMesh = createSubVolumetricMesh;
	numDomains = seeds_index.size();
	numVertex = volumetricmesh->getNumVertices();
	numElements = volumetricmesh->getNumElements();

}


DomainDecomposition::DomainDecomposition(LoboVolumetricMesh* volumetricmesh, LoboVolumetricMeshGraph* volumetricmeshgraph, std::vector<int> seeds_index, bool createSubVolumetricMesh /*= false*/) :DomainDecomposition(volumetricmesh,seeds_index,createSubVolumetricMesh)
{
	this->volumetricmesh_graph = volumetricmeshgraph;
}

DomainDecomposition::DomainDecomposition(LoboVolumetricMesh* volumetricmesh, LoboVolumetricMeshGraph* volumetricmeshgraph, bool createSubVolumetricMesh /*= false*/)
{
	this->volumetricmesh_graph = volumetricmeshgraph;
	this->volumetricmesh = volumetricmesh;
	this->createSubVolumetricMesh = createSubVolumetricMesh;
	numDomains = seeds_index.size();
	numVertex = volumetricmesh->getNumVertices();
	numElements = volumetricmesh->getNumElements();
}

DomainDecomposition::~DomainDecomposition()
{
}

void DomainDecomposition::getDomain(int domainid, std::vector<int> &elements)
{
	int numElements = domainsElements[domainid].size();
	elements.reserve(numElements);
	for (int i = 0; i < numElements; i++)
	{
		elements[i] = domainsElements[domainid][i];
	}
}

LoboVolumetricMesh* DomainDecomposition::getSubVolumetricMesh(int domainid)
{
	return NULL;
}

std::vector<int> DomainDecomposition::getDomainNodes(int frameid)
{
	return domainsNodes[frameid];
}

std::vector<int> DomainDecomposition::getDomainRegionEle(int frameid)
{
	return domainsElements[frameid];
}

std::vector<int> DomainDecomposition::getNodeDomains(int nodeid)
{
	return nodesDomains[nodeid];
}

std::vector<int> DomainDecomposition::getElementDomains(int eleid)
{
	return elementDomains[eleid];
}
