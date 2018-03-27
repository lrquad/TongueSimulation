#include "LoboVolumetricMeshGraph.h"
#include "LoboVolumetricMesh/LoboVolumetricMesh.h"
#include "LoboVolumetricMesh/LoboNodeBase.h"

typedef graph_traits<Graph>::vertex_descriptor Vertex;

LoboVolumetricMeshGraph::LoboVolumetricMeshGraph(LoboVolumetricMesh* volumetricmesh)
{
	this->volumetricmesh = volumetricmesh;
}


LoboVolumetricMeshGraph::~LoboVolumetricMeshGraph()
{
	delete boost_graph;
}

void LoboVolumetricMeshGraph::init()
{
	int numVertex = volumetricmesh->getNumVertices();
	//edges
	initEdgeList();
	boost_graph = new Graph(edge_list.begin(), edge_list.end(),edge_weights.begin(),numVertex);
}

void LoboVolumetricMeshGraph::compute_dijkstra_shortest_paths(int startnode, std::vector<double> &distance)
{
	int numVertex = volumetricmesh->getNumVertices();
	distance.resize(numVertex);
	Vertex s = *(vertices(*boost_graph).first + startnode);
	dijkstra_shortest_paths(*boost_graph, s, distance_map(&distance[0]));
}

void LoboVolumetricMeshGraph::compute_dijkstra_shortest_paths(int startnode, int endnode)
{
	Vertex s = *(vertices(*boost_graph).first + startnode);
}

void LoboVolumetricMeshGraph::initEdgeList()
{	
	edge_list.clear();

	int numVertex = volumetricmesh->getNumVertices();
	for (int i = 0; i < numVertex; i++)
	{
		int nodeid = i;
		LoboNodeBase* node = volumetricmesh->getNodeRef(nodeid);
		int neighborsize = node->neighbor.size();
		for (int j = 0; j < neighborsize; j++)
		{
			int neighborid = node->neighbor[j];
			
			//avoid insert same edge
			if (neighborid > nodeid)
			{
				edge_list.push_back(Edge(nodeid, neighborid));
				double distance = (volumetricmesh->getNodeRestPosition(nodeid) - volumetricmesh->getNodeRestPosition(neighborid)).norm();
				edge_weights.push_back(distance);
			}
		}
	}
}
