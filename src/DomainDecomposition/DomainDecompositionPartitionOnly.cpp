#include "DomainDecompositionPartitionOnly.h"
#include "LoboVolumetricMesh\LoboNodeBase.h"
#include "Functions/findElementInVector.h"
#include <iostream>
#include "LoboVolumetricMesh/LoboVolumetriceMeshCore.h"


DomainDecompositionPartitionOnly::DomainDecompositionPartitionOnly(LoboVolumetricMesh* volumetricmesh, std::vector<int> seeds_index, WeightFunctionLaplacian* weightfunction, bool createSubVolumetricMesh /*= false*/) :DomainDecompositionPartition(volumetricmesh,seeds_index,weightfunction,createSubVolumetricMesh)
{

}

DomainDecompositionPartitionOnly::DomainDecompositionPartitionOnly(LoboVolumetricMesh* volumetricmesh, LoboVolumetricMeshGraph* volumetricmeshgraph, std::vector<int> seeds_index, WeightFunctionLaplacian* weightfunction, bool createSubVolumetricMesh /*= false*/) : DomainDecompositionPartition(volumetricmesh, volumetricmeshgraph, seeds_index, weightfunction, createSubVolumetricMesh)
{

}

DomainDecompositionPartitionOnly::~DomainDecompositionPartitionOnly()
{
}

void DomainDecompositionPartitionOnly::doDecomposition()
{
	computeDistanceToFrameSeed();
	searchNeighborDomainByNodes();
	fillDomain();
	ignoreUnConnectedNodes();
	checkDomainConnection();
	createDomainConstraints();
	generateRemoveConstrainInfo();
}

void DomainDecompositionPartitionOnly::createDomainConstraints()
{
	std::cout << "need to finish" << std::endl;
}

void DomainDecompositionPartitionOnly::fillDomain()
{
	domainsElements.clear();
	domainsElements.resize(numDomains);
	nodesDomains.clear();
	nodesDomains.resize(numVertex);
	domainsNodes.clear();
	domainsNodes.resize(numDomains);
	elementDomains.clear();
	elementDomains.resize(numElements);

	int nodePerElement = volumetricmesh->getNumElementVertices();

	for (int i = 0; i < numDomains; i++)
	{
		int domainid = i;
		std::vector<bool> domainNodemark(numVertex);
		std::fill(domainNodemark.begin(), domainNodemark.end(), false);

		//add this seed's partition first
		int partition_size = distance_region_info[domainid].size();
		for (int j = 0; j < partition_size; j++)
		{
			int partition_nodeid = distance_region_info[domainid][j];

			domainsNodes[domainid].push_back(partition_nodeid);
			nodesDomains[partition_nodeid].push_back(domainid);

			domainNodemark[partition_nodeid] = true;
		}
		

		for (int j = 0; j < numElements; j++)
		{
			bool assume_is_in = true;
			for (int k = 0; k < nodePerElement; k++)
			{
				int nodeid = volumetricmesh->getElementNode(j, k);
				if (!domainNodemark[nodeid])
					assume_is_in = false;
			}

			if (assume_is_in)
			{
				domainsElements[i].push_back(j);
				elementDomains[j].push_back(i);
			}
		}
	}


	//no unvisited elements
	//but just check
	for (int i = 0; i < numElements; i++)
	{
		if (elementDomains[i].size() == 0)
		{
			double mindistance = DBL_MAX;
			int belongto = -1;
			for (int j = 0; j < numDomains; j++)
			{
				int frameid = j;
				double eledistance = elementDistancetoFrame.data()[frameid*numElements + i];
				if (eledistance < mindistance)
				{
					mindistance = eledistance;
					belongto = j;
				}
			}

			domainsElements[belongto].push_back(i);
			elementDomains[i].push_back(belongto);
		}
	}
}
