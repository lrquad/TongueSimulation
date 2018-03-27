#include "DomainDecompositionKmeans.h"
#include "LoboVolumetricMesh/LoboVolumetricMesh.h"
#include "LoboVolumetricMesh/Graph/LoboVolumetricMeshGraph.h"
#include <ctime>

DomainDecompositionKmeans::DomainDecompositionKmeans(LoboVolumetricMesh* volumetricmesh, LoboVolumetricMeshGraph* volumetricmesh_graph, int numConstrainedDOFs, int* constraintedDOFS, int k) :
DomainDecompositionFramebased(volumetricmesh,volumetricmesh_graph,NULL)
{
	this->numClustring = k;
	seeds_index.resize(numClustring);
	numDomains = numClustring;

	distancetoFrame.resize(numVertex, numDomains+numConstrainedDOFs/3);
	elementDistancetoFrame.resize(numElements, numDomains + numConstrainedDOFs / 3);

	this->numConstrainedDOFs = numConstrainedDOFs;
	this->constraintedDOFS = constraintedDOFS;

	std::cout <<"input K ===> "<< numClustring << std::endl;

	this->initGuess();

}

DomainDecompositionKmeans::~DomainDecompositionKmeans()
{

}

void DomainDecompositionKmeans::doDecomposition()
{
	//compute 
	int iteration = 1;
	int maxiteration = 100;
	while (iteration&&maxiteration>0)
	{
		computeDistanceToFrameSeed();
		searchNeighborDomainByNodes();

		iteration = updateClusterCenters();
		maxiteration--;
	}


	std::cout <<"kmeans iteration left: "<< maxiteration << std::endl;

	//insert constrained info
	for (int i = 0; i < numConstrainedDOFs / 3; i++)
	{
		int nodeid = constraintedDOFS[i * 3] / 3;
		seeds_index.push_back(nodeid);
		
		std::vector<double> nodedistance;
		volumetricmesh_graph->compute_dijkstra_shortest_paths(seeds_index[seeds_index.size() - 1], nodedistance);
		for (int j = 0; j < numVertex; j++)
		{
			distancetoFrame.data()[(seeds_index.size() - 1) * numVertex + j] = nodedistance[j];
		}
	}

	numDomains = seeds_index.size();
	searchNeighborDomainByNodes();
	updateDomainRadus();
}

double DomainDecompositionKmeans::getDomainRadius(int domainid)
{
	return domainRadius[domainid];
}

void DomainDecompositionKmeans::fillDomain()
{
	domainsElements.clear();
	domainsElements.resize(numDomains);
	nodesDomains.clear();
	nodesDomains.resize(numVertex);
	domainsNodes.clear();
	domainsNodes.resize(numDomains);
	elementDomains.clear();
	elementDomains.resize(numElements);

}

void DomainDecompositionKmeans::updateDomainRadus()
{
	domainRadius.resize(numDomains);
	for (int i = 0; i < numDomains; i++)
	{
		int neighborsize = seed_neighbor_graph[i].size();
		double maxdistance = -DBL_MAX;
		int maxneighbor = -1;

		for (int j = 0; j < neighborsize; j++)
		{
			int neighborid = seed_neighbor_graph[i][j];
			int domainseed = seeds_index[neighborid];

			double distance = getNodeToFrameDistance(domainseed, i);

			if (distance > maxdistance)
			{
				maxdistance = distance;
				maxneighbor = neighborid;
			}
		}

		domainRadius[i] = maxdistance;
		std::cout <<"domain radius => "<< domainRadius[i] << std::endl;
	}
}

int DomainDecompositionKmeans::updateClusterCenters()
{
	bool neediteration = 0;
	for (int i = 0; i < numDomains; i++)
	{
		Vector3d newCenter = Vector3d::Zero();
		for (int j = 0; j < this->distance_region_info[i].size(); j++)
		{
			int nodeid = this->distance_region_info[i][j];
			Vector3d xi = volumetricmesh->getNodeRestPosition(nodeid);
			newCenter += xi;
		}
		newCenter /= this->distance_region_info[i].size();

		int newseed = volumetricmesh->searchCloseNode(newCenter);
		if (seeds_index[i] != newseed)
		{
			neediteration = 1;
		}
		seeds_index[i] = newseed;

	}

	return neediteration;
}

void DomainDecompositionKmeans::initGuess()
{
	Vector3d geometrycenter = Vector3d::Zero();
	

	//init first guess
	numVertex = volumetricmesh->getNumVertices();
	
	
	std::vector<int> nodeindices(volumetricmesh->getNumVertices());
	for (int i = 0; i < numVertex; i++)
	{
		nodeindices[i] = i;
		geometrycenter += volumetricmesh->getNodeRestPosition(i);
	}
	geometrycenter /= numVertex;

	int firstseed = volumetricmesh->searchCloseNode(geometrycenter);
	seeds_index[0] = firstseed;
	//asume all have max distance

	memset(distancetoFrame.data(), sizeof(double)*distancetoFrame.rows()*distancetoFrame.cols(), DBL_MAX);

	std::vector<double> nodedistance;
	volumetricmesh_graph->compute_dijkstra_shortest_paths(seeds_index[0], nodedistance);
	for (int j = 0; j < numVertex; j++)
	{
		distancetoFrame.data()[0*numVertex + j] = nodedistance[j];
	}

	int nextdomaind = 1;
	while (nextdomaind < numClustring)
	{
		int nextseed = -1;
		double max_dis = -DBL_MAX;
		for (int i = 0; i < numVertex; i++)
		{
			double min_dis = DBL_MAX;
			for (int j = 0; j < nextdomaind; j++)
			{
				double distance = this->getNodeToFrameDistance(i,j);
				min_dis = min_dis < distance ? min_dis : distance;
			}

			if (min_dis > max_dis)
			{
				max_dis = min_dis;
				nextseed = i;
			}
		}
		seeds_index[nextdomaind] = nextseed;
		volumetricmesh_graph->compute_dijkstra_shortest_paths(seeds_index[nextdomaind], nodedistance);

		for (int j = 0; j < numVertex; j++)
		{
			distancetoFrame.data()[nextdomaind * numVertex + j] = nodedistance[j];
		}

		nextdomaind++;
	}
	


}
