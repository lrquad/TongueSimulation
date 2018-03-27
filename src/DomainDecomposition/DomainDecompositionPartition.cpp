#include "DomainDecompositionPartition.h"
#include "LoboVolumetricMesh\LoboVolumetricMesh.h"
#include "LoboVolumetricMesh\LoboNodeBase.h"
#include "Functions/findElementInVector.h"
#include <iostream>

DomainDecompositionPartition::DomainDecompositionPartition(LoboVolumetricMesh* volumetricmesh, std::vector<int> seeds_index, WeightFunctionLaplacian* weightfunction, bool createSubVolumetricMesh)
	:DomainDecompositionFramebased(volumetricmesh, seeds_index, weightfunction, createSubVolumetricMesh)
{

}


DomainDecompositionPartition::DomainDecompositionPartition(LoboVolumetricMesh* volumetricmesh, LoboVolumetricMeshGraph* volumetricmeshgraph, std::vector<int> seeds_index, WeightFunctionLaplacian* weightfunction, bool createSubVolumetricMesh /*= false*/) :DomainDecompositionFramebased(volumetricmesh, volumetricmeshgraph, seeds_index, weightfunction, createSubVolumetricMesh)
{

}

DomainDecompositionPartition::~DomainDecompositionPartition()
{
	
}

void DomainDecompositionPartition::doDecomposition()
{
	computeDistanceToFrameSeed();
	searchNeighborDomainByNodes();
	fillDomain();
	ignoreUnConnectedNodes();
	checkDomainConnection();
	createDomainConstraints();
	generateRemoveConstrainInfo();
}

void DomainDecompositionPartition::createDomainConstraints()
{
	region_border.resize(numDomains);
	region_inside_constraint.resize(numDomains);
	region_all_constraints.resize(numDomains);

	for (int i = 0; i < numDomains; i++)
	{
		// find domain border first
		int domainsize = domainsNodes[i].size();
		for (int j = 0; j < domainsize; j++)
		{
			int nodeid = domainsNodes[i][j];
			LoboNodeBase* node = volumetricmesh->getNodeRef(nodeid);
			int neighborsize = node->neighbor.size();
			for (int e = 0; e < neighborsize; e++)
			{
				int neighborid = node->neighbor[e];
				if (neighborid != -1)
					if (!findElement(nodesDomains[neighborid], i))
					{
						//ele is a border
						region_border[i].push_back(nodeid);
						break;
					}
			}//end for e
		}
	}

	//neighbor seed
	for (int i = 0; i < numDomains; i++)
	{
		int domainid = i;
		int neighborsize = this->seed_neighbor_graph[i].size();
		for (int j = 0; j < neighborsize; j++)
		{
			int neighborid = seed_neighbor_graph[i][j];
			int seed = seeds_index[neighborid];
			region_inside_constraint[domainid].push_back(seed);
		}
	}

	//merge two types of constraints.
	for (int i = 0; i < numDomains; i++)
	{
		std::vector<bool> constrainMark(numVertex);
		std::fill(constrainMark.begin(), constrainMark.end(), false);
		for (int j = 0; j < region_border[i].size(); j++)
		{
			int nodeid = region_border[i][j];
			constrainMark[nodeid] = true;
		}

		for (int j = 0; j < region_inside_constraint[i].size(); j++)
		{
			int nodeid = region_inside_constraint[i][j];
			constrainMark[nodeid] = true;
		}

		for (int j = 0; j < numVertex; j++)
		{
			if (constrainMark[j])
			{
				region_all_constraints[i].push_back(j);
			}
		}
	}

}

void DomainDecompositionPartition::fillDomain()
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

		int neighbor_partitionsize = this->seed_neighbor_graph[domainid].size();
		for (int j = 0; j < neighbor_partitionsize; j++)
		{
			int neighborDomainid = seed_neighbor_graph[domainid][j];

			partition_size = distance_region_info[neighborDomainid].size();

			for (int k = 0; k < partition_size; k++)
			{
				int partition_nodeid = distance_region_info[neighborDomainid][k];

				//add to domainid
				domainsNodes[domainid].push_back(partition_nodeid);
				nodesDomains[partition_nodeid].push_back(domainid);
				domainNodemark[partition_nodeid] = true;
			}
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
			std::cout << "un visited element" << std::endl;
		}
	}

}
