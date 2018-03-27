#include "DomainDecompositionLocal.h"
#include "LoboVolumetricMesh\LoboVolumetricMesh.h"
#include "LoboVolumetricMesh\LoboNodeBase.h"
#include "Functions/findElementInVector.h"
#include <iostream>
#include "DomainDecompositionPartition.h"

DomainDecompositionLocal::DomainDecompositionLocal(
	DomainDecompositionPartition* oriDomainDecomposition,
	LoboVolumetricMesh* volumetricmesh, LoboVolumetricMeshGraph* volumetricmeshgraph, std::vector<int> seeds_index,  WeightFunctionLaplacian* weightfunction, bool createSubVolumetricMesh /*= false*/)
	:DomainDecompositionFramebased(volumetricmesh,seeds_index,weightfunction,createSubVolumetricMesh)
{
	this->oriDomainDecomposition = oriDomainDecomposition;
	domainRadius.resize(numDomains);

	//just set 0.5 for test need change in future.
	std::fill(domainRadius.begin(), domainRadius.end(), 2.0);
}

DomainDecompositionLocal::~DomainDecompositionLocal()
{

}

void DomainDecompositionLocal::doDecomposition()
{
	computeDistanceToFrameSeed();
	searchDomainRegion();
	fillDomain();
	ignoreUnConnectedNodes();
	checkDomainConnection();
	createDomainConstraints();
	generateRemoveConstrainInfo();
}

void DomainDecompositionLocal::createDomainConstraints()
{
	region_border.resize(numDomains);
	region_inside_constraint.resize(numDomains);
	region_all_constraints.resize(numDomains);
	std::vector<int> ori_seeds = oriDomainDecomposition->getSeeds();

	for (int i = 0; i < numDomains; i++)
	{
		int domainsize = domainsNodes[i].size();
		for (int j = 0; j < domainsize; j++)
		{
			int nodeid = domainsNodes[i][j];
			LoboNodeBase* node = volumetricmesh->getNodeRef(nodeid);
			int neighborsize = node->neighbor.size();
			bool isborder = false;
			for (int e = 0; e < neighborsize; e++)
			{
				int neighborid = node->neighbor[e];
				if (neighborid != -1)
					if (!findElement(nodesDomains[neighborid], i))
					{
						//ele is a border
						region_border[i].push_back(nodeid);
						isborder = true;
						break;
					}
			}//end for e

			if (!isborder)
			for (int k = 0; k < ori_seeds.size(); k++)
			{
				if (nodeid == ori_seeds[k])
				{
					region_border[i].push_back(nodeid);
				}
			}
		}

	}

	//no neighbor seed

	region_all_constraints = region_border;

}


double DomainDecompositionLocal::getDomainRadius(int domainid)
{
	return domainRadius[domainid];
}

void DomainDecompositionLocal::setDomainRadius(int domainid, double val)
{
	domainRadius[domainid] = val;
}

void DomainDecompositionLocal::fillDomain()
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
				{
					assume_is_in = false;
				}
			}

			if (assume_is_in)
			{
				domainsElements[i].push_back(j);
				elementDomains[j].push_back(i);
			}
		}

	}
}

void DomainDecompositionLocal::searchDomainRegion()
{
	partition_nodemark.resize(numVertex);
	std::fill(partition_nodemark.begin(), partition_nodemark.end(), -1);

	for (int i = 0; i < distance_region_info.size(); i++)
	{
		distance_region_info[i].clear();
	}
	distance_region_info.clear();

	//new domains + ori domains
	int fullnumDomains = numDomains + oriDomainDecomposition->getNumDomain();
	// the number of original domains
	int oriNumDomains = oriDomainDecomposition->getNumDomain();

	//will only keep the info of new domains
	distance_region_info.resize(numDomains);
	for (int i = 0; i < numVertex; i++)
	{
		for (int j = 0; j < numDomains; j++)
		{
			double distance = this->getNodeToFrameDistance(i, j);
			if (distance < domainRadius[j])
			{
				distance_region_info[j].push_back(i);
			}
		}
	}
}

//this function can determin the region of adaptive domain
void DomainDecompositionLocal::searchNeighborDomainByNodes()
{
	partition_nodemark.resize(numVertex);
	std::fill(partition_nodemark.begin(), partition_nodemark.end(), -1);
	
	for (int i = 0; i < distance_region_info.size(); i++)
	{
		distance_region_info[i].clear();
	}
	distance_region_info.clear();

	//new domains + ori domains
	int fullnumDomains = numDomains + oriDomainDecomposition->getNumDomain();
	// the number of original domains
	int oriNumDomains = oriDomainDecomposition->getNumDomain();

	//will only keep the info of new domains
	distance_region_info.resize(numDomains);

	//loop all vertex to check which domain is the closest domain
	for (int i = 0; i < numVertex; i++)
	{
		double mindistance = DBL_MAX;
		int belongregion = -1;
		for (int j = 0; j < numDomains; j++)
		{
			double distance = this->getNodeToFrameDistance(i, j);
			if (distance < mindistance)
			{
				mindistance = distance;
				belongregion = j;
			}
		}

		for (int j = 0; j < oriNumDomains; j++)
		{
			double distance = oriDomainDecomposition->getNodeToFrameDistance(i, j);
			if (distance < mindistance)
			{
				mindistance = distance;

				//because this region is original region, not adaptive region. so let index shift num Domains
				belongregion = j + numDomains;
			}
		}

		if (belongregion < numDomains)
		{
			partition_nodemark[i] = belongregion;
			distance_region_info[belongregion].push_back(i);
		}
	}

	seed_neighbor_graph.clear();
	seed_neighbor_graph.resize(fullnumDomains);






}
