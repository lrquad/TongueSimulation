#include "DomainDecompositionKmeansModify.h"
#include "LoboVolumetricMesh/Graph/LoboVolumetricMeshGraph.h"

DomainDecompositionKmeansModify::DomainDecompositionKmeansModify(LoboVolumetricMesh* volumetricmesh, LoboVolumetricMeshGraph* volumetricmesh_graph, int numConstrainedDOFs, int* constraintedDOFS, int k, std::vector<int> seeds)
	:DomainDecompositionKmeans(volumetricmesh, volumetricmesh_graph, numConstrainedDOFs, constraintedDOFS, seeds.size())
{
	given_seeds = seeds;
}

DomainDecompositionKmeansModify::~DomainDecompositionKmeansModify()
{

}

void DomainDecompositionKmeansModify::doDecomposition()
{
	seeds_index = given_seeds;
	computeDistanceToFrameSeed();
	searchNeighborDomainByNodes();

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
