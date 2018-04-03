#pragma once
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <iostream>                  // for std::cout
#include <utility>                   // for std::pair
#include <algorithm>                 // for std::for_each


using namespace boost;
typedef adjacency_list<listS, vecS, undirectedS,
	no_property, property<edge_weight_t, double> > Graph;

typedef std::pair<int, int> Edge;

class LoboVolumetricMesh;


class LoboVolumetricMeshGraph
{
public:
	LoboVolumetricMeshGraph(LoboVolumetricMesh* volumetricmesh);
	~LoboVolumetricMeshGraph();

	virtual void init();

	virtual void compute_dijkstra_shortest_paths(int startnode,std::vector<double> &distance);

	virtual void compute_dijkstra_shortest_paths(int startnode, int endnode);

protected:

	virtual void initEdgeList();

	LoboVolumetricMesh* volumetricmesh;
	Graph* boost_graph;

	std::vector<Edge> edge_list;
	std::vector<double> edge_weights;
};

