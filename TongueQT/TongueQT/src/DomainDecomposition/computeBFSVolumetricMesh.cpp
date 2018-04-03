#include "computeBFSVolumetricMesh.h"
#include "LoboVolumetricMesh\LoboNodeBase.h"

void computeDijkstraDistance(LoboVolumetricMesh* volumetricmesh, int seedindex, std::vector<double> &nodedistance)
{
	int numVertex = volumetricmesh->getNumVertices();
	std::cout << "start DijkstraDistance ..." << std::endl;
	nodedistance.resize(numVertex);
	std::fill(nodedistance.begin(), nodedistance.end(), DBL_MAX);

	int startelement = seedindex;
	std::vector<bool> visited_mark;
	visited_mark.resize(numVertex);
	std::fill(visited_mark.begin(), visited_mark.end(), false);

	
	visited_mark[startelement] = true;
	nodedistance[startelement] = 0;

	std::vector<int> queue_list(numVertex);
	for (int i = 0; i < numVertex; i++)
	{
		queue_list[i] = i;
	}

	while (queue_list.size() != 0)
	{
		//pick min value;
		double mindis = DBL_MAX;
		int minvertexid;
		int minqueueindex;
		for (int i = 0; i < queue_list.size(); i++)
		{
			if (nodedistance[queue_list[i]] < mindis)
			{
				mindis = nodedistance[queue_list[i]];
				minvertexid = queue_list[i];
				minqueueindex = i;
			}
		}
		queue_list.erase(queue_list.begin()+minqueueindex);
		int current_ele = minvertexid;

		LoboNodeBase* node = volumetricmesh->getNodeRef(current_ele);

		int neighborsize = node->neighbor.size();

		for (int j = 0; j < neighborsize; j++)
		{
			int neighborid = node->neighbor[j];

			double distancetocurrent;
			Vector3d neighborposition = volumetricmesh->getNodeRestPosition(neighborid);
			distancetocurrent = (node->ori_position - neighborposition).norm();

			if (nodedistance[current_ele] + distancetocurrent<nodedistance[neighborid])
			nodedistance[neighborid] = nodedistance[current_ele] + distancetocurrent;	
		}
	}

	std::cout << "finished DijkstraDistance." << std::endl;

}

void computeBFSDistance(LoboVolumetricMesh* volumetricmesh, int seedindex, std::vector<double> &nodedistance)
{
	int numVertex = volumetricmesh->getNumVertices();
	nodedistance.resize(numVertex);
	std::fill(nodedistance.begin(), nodedistance.end(), 0);

	int startelement = seedindex;
	std::vector<bool> visited_mark;
	visited_mark.resize(numVertex);
	std::fill(visited_mark.begin(), visited_mark.end(), false);
	
	std::queue<int> queue_list;
	queue_list.push(startelement);
	visited_mark[startelement] = true;

	while (queue_list.size() != 0)
	{
		int current_ele = queue_list.front();
		queue_list.pop();

		LoboNodeBase* node = volumetricmesh->getNodeRef(current_ele);

		int neighborsize = node->neighbor.size();

		for (int j = 0; j < neighborsize; j++)
		{
			int neighborid = node->neighbor[j];
			if (!visited_mark[neighborid])
			{
				double distancetocurrent;
				Vector3d neighborposition = volumetricmesh->getNodeRestPosition(neighborid);
				distancetocurrent = (node->ori_position - neighborposition).norm();
				nodedistance[neighborid] = nodedistance[current_ele] + distancetocurrent;
				queue_list.push(neighborid);
				visited_mark[neighborid] = true;
			}
		}
	}
	std::cout << "finished BFD." << std::endl;
}

int computeBFSDistance(LoboVolumetricMesh* volumetricmesh, int seedindex, std::vector<double> &nodedistance, std::vector<int> subnodesindex)
{
	
	int fullnumVertex = volumetricmesh->getNumVertices();
	int numVertex = subnodesindex.size();
	//create a map
	std::vector<int> map_full_sup(fullnumVertex);
	std::fill(map_full_sup.begin(), map_full_sup.end(), -1);
	for (int i = 0; i < subnodesindex.size(); i++)
	{
		int full = subnodesindex[i];
		map_full_sup[full] = i;
	}
	
	nodedistance.resize(numVertex);
	std::fill(nodedistance.begin(), nodedistance.end(), DBL_MAX);

	std::vector<bool> visited_mark;
	visited_mark.resize(numVertex);
	std::fill(visited_mark.begin(), visited_mark.end(), false);


	int startelement = map_full_sup[seedindex];
	if (startelement == -1)
	{
		std::cout << "error seed is not in region" << std::endl;
		return 2;
	}
	
	std::queue<int> queue_list;
	queue_list.push(startelement);
	visited_mark[startelement] = true;
	nodedistance[startelement] = 0;

	int countvisited = 1;
	while (queue_list.size() != 0)
	{
		int current_ele = queue_list.front();
		queue_list.pop();
		int current_node_full = subnodesindex[current_ele];

		LoboNodeBase* node = volumetricmesh->getNodeRef(current_node_full);

		int neighborsize = node->neighbor.size();

		for (int j = 0; j < neighborsize; j++)
		{
			int neighborid = node->neighbor[j];
			
			if (map_full_sup[neighborid]!=-1)
			if (!visited_mark[map_full_sup[neighborid]])
			{
				double distancetocurrent;
				Vector3d neighborposition = volumetricmesh->getNodeRestPosition(neighborid);
				distancetocurrent = (node->ori_position - neighborposition).norm();
				nodedistance[map_full_sup[neighborid]] = nodedistance[current_ele] + distancetocurrent;
				queue_list.push(map_full_sup[neighborid]);
				visited_mark[map_full_sup[neighborid]] = true;
				countvisited++;
			}
		}
	}

	if (countvisited == subnodesindex.size())
	{
		return 1;
	}
	else
	{
		return 0;
	}
}