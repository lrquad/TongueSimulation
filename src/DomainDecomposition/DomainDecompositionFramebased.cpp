#include "DomainDecompositionFramebased.h"
#include "computeBFSVolumetricMesh.h"
#include "LoboVolumetricMesh\LoboNodeBase.h"
#include "Functions/findElementInVector.h"
#include "SparseMatrix/SparseMatrixRemoveRows.h"
#include "LoboVolumetricMesh\LoboNodeBase.h"
#include "LoboVolumetricMesh\Graph\LoboVolumetricMeshGraph.h"
#include <fstream>
#include <QElapsedTimer>
#include "weightFunction/WeightFunctionLaplacian.h"

DomainDecompositionFramebased::DomainDecompositionFramebased(LoboVolumetricMesh* volumetricmesh, std::vector<int> seeds_index, WeightFunctionLaplacian* weightfunction, bool createSubVolumetricMesh) :DomainDecomposition(volumetricmesh, seeds_index, createSubVolumetricMesh)
{
	distancetoFrame.resize(numVertex, numDomains);
	elementDistancetoFrame.resize(numElements, numDomains);

	this->weightfunction = weightfunction;

	constrainInfoCalled = false;
}

DomainDecompositionFramebased::DomainDecompositionFramebased(LoboVolumetricMesh* volumetricmesh, LoboVolumetricMeshGraph* volumetricmeshgraph, std::vector<int> seeds_index, WeightFunctionLaplacian* weightfunction, bool createSubVolumetricMesh /*= false*/) :DomainDecomposition(volumetricmesh,volumetricmeshgraph,seeds_index,createSubVolumetricMesh)
{
	distancetoFrame.resize(numVertex, numDomains);
	elementDistancetoFrame.resize(numElements, numDomains);

	this->weightfunction = weightfunction;

	constrainInfoCalled = false;
}

DomainDecompositionFramebased::DomainDecompositionFramebased(LoboVolumetricMesh* volumetricmesh, LoboVolumetricMeshGraph* volumetricmeshgraph, WeightFunctionLaplacian* weightfunction, bool createSubVolumetricMesh /* = false */):DomainDecomposition(volumetricmesh, volumetricmeshgraph, createSubVolumetricMesh)
{
	

	this->weightfunction = weightfunction;
	constrainInfoCalled = false;
}

DomainDecompositionFramebased::~DomainDecompositionFramebased()
{
	if (constrainInfoCalled)
	{
		for (int i = 0; i < numDomains; i++)
		{
			free(DomainRemoveDofs[i]);
		}
	}
}

void DomainDecompositionFramebased::doDecomposition()
{
	computeDistanceToFrameSeed();
	searchNeighborDomain();
	findMostDistanceNeighborseed();
	fillDomain();
	//checkDomainConnection();
	createDomainConstraints();
	generateRemoveConstrainInfo();
	//searchBorderElement();
}

void DomainDecompositionFramebased::generateRemoveConstrainInfo()
{
	this->numDomainRemoveDofs.resize(numDomains);
	this->DomainRemoveDofs.resize(numDomains);
	for (int i = 0; i < numDomains; i++)
	{
		int numDofs = domainsNodes[i].size();
		numDomainRemoveDofs[i] = (numVertex - numDofs) * 3;
		DomainRemoveDofs[i] = (int*)malloc(sizeof(int)*numDomainRemoveDofs[i]);

		std::vector<bool> nodemark(numVertex);
		std::fill(nodemark.begin(), nodemark.end(), false);
		for (int j = 0; j < numDofs; j++)
		{
			int nodeid = domainsNodes[i][j];
			nodemark[nodeid] = true;
		}
		int count = 0;
		for (int j = 0; j < numVertex; j++)
		{
			if (!nodemark[j])
			{
				DomainRemoveDofs[i][count * 3 + 0] = j * 3 + 0;
				DomainRemoveDofs[i][count * 3 + 1] = j * 3 + 1;
				DomainRemoveDofs[i][count * 3 + 2] = j * 3 + 2;
				count++;
			}
		}
	}

	mapOldNewDomain.resize(numDomains);
	int r = numVertex * 3;
	for (int i = 0; i < numDomains; i++)
	{
		int numRemovedDofs = this->getNumRemovedDOFs(i);
		int * removedDofs = this->getRemovedDOFs(i);
		createMapByConstrains(mapOldNewDomain[i], r, numRemovedDofs, removedDofs);
	}
}

std::vector<int> DomainDecompositionFramebased::getDomainMap(int domainid)
{
	return mapOldNewDomain[domainid];
}

void DomainDecompositionFramebased::checkDomainConnection()
{
	std::vector<bool> connectionresult(numDomains);
	for (int i = 0; i < numDomains; i++)
	{
		connectionresult[i] = checkDomainConnection(i);
	}

	bool need_reassgin = false;

	for (int i = 0; i < numDomains; i++)
	{
		if (!connectionresult[i])
		{
			need_reassgin = true;
		}
	}

	if (!need_reassgin)
	{
		return;
	}

	for (int i = 0; i < numVertex; i++)
	{
		nodesDomains[i].clear();
	}

	for (int i = 0; i < numElements; i++)
	{
		elementDomains[i].clear();
	}

	for (int i = 0; i < numDomains; i++)
	{
		for (int j = 0; j < domainsNodes[i].size(); j++)
		{
			int nodeid = domainsNodes[i][j];
			nodesDomains[nodeid].push_back(i);
		}

		for (int j = 0; j < domainsElements[i].size();j++)
		{
			int eleid = domainsElements[i][j];
			elementDomains[eleid].push_back(i);
		}
	}

}

bool DomainDecompositionFramebased::checkDomainConnection(int domainid)
{
	std::vector<int> subnodeid = domainsNodes[domainid];
	std::vector<double> nodedistance;
	int result = computeBFSDistance(volumetricmesh, seeds_index[domainid], nodedistance, subnodeid);
	
	if (result == 0)
	{
		std::vector<bool> newSubnodemark(numVertex);
		std::fill(newSubnodemark.begin(), newSubnodemark.end(), false);
		std::vector<int> newSubnodeid;
		for (int i = 0; i < nodedistance.size(); i++)
		{
			if (nodedistance[i] < DBL_MAX)
			{
				newSubnodeid.push_back(subnodeid[i]);
				newSubnodemark[subnodeid[i]] = true;
			}
		}

		std::vector<int> newSubelement;
		std::vector<int> oldSubelement = domainsElements[domainid];
		int nodePerElement = volumetricmesh->getNumElementVertices();


		for (int i = 0; i < oldSubelement.size(); i++)
		{
			int ele = oldSubelement[i];
			bool assume_is_in = true;
			for (int k = 0; k < nodePerElement; k++)
			{
				int nodeid = volumetricmesh->getElementNode(ele, k);
				if (!newSubnodemark[nodeid])
				{
					assume_is_in = false;
				}
			}

			if (assume_is_in)
			{
				newSubelement.push_back(ele);
			}
		}

		domainsNodes[domainid] = newSubnodeid;
		domainsElements[domainid] = newSubelement;
		return false;
	}
	return true;
}


void DomainDecompositionFramebased::ignoreUnConnectedNodes()
{
	int nodePerElement = volumetricmesh->getNumElementVertices();

	for (int i = 0; i < numVertex; i++)
	{
		nodesDomains[i].clear();
	}


	for (int i = 0; i < numDomains; i++)
	{
		domainsNodes[i].clear();
		std::vector<bool> domainNodesMark(numVertex);
		std::fill(domainNodesMark.begin(), domainNodesMark.end(), false);
		int domainelementsize = domainsElements[i].size();
		for (int j = 0; j < domainelementsize; j++)
		{
			int eleid = domainsElements[i][j];
			
			for (int k = 0; k < nodePerElement; k++)
			{
				int nodeid = volumetricmesh->getElementNode(eleid, k);
				domainNodesMark[nodeid] = true;
			}
		}

		for (int j = 0; j < numVertex; j++)
		{
			if (domainNodesMark[j])
			{
				domainsNodes[i].push_back(j);
				nodesDomains[j].push_back(i);
			}
		}
	}
}

std::vector<int> DomainDecompositionFramebased::getDomainRegionBorder(int frameid)
{
	
	return region_border[frameid];
}

std::vector<int> DomainDecompositionFramebased::getDomainInsideConstraints(int frameid)
{
	return region_inside_constraint[frameid];
}

std::vector<int> DomainDecompositionFramebased::getDomainConstrain(int frameid)
{
	return region_all_constraints[frameid];
}

std::vector<int> DomainDecompositionFramebased::getDomainBorderEle(int frameid)
{
	return region_border_elements[frameid];
}

std::vector<int> DomainDecompositionFramebased::getDomainPartition(int frameid)
{
	return distance_region_info[frameid];
}

std::vector<int> DomainDecompositionFramebased::getDomainNeighbor(int frameid)
{
	return seed_neighbor_graph[frameid];
}

int DomainDecompositionFramebased::getNumRemovedDOFs(int domainid)
{
	return numDomainRemoveDofs[domainid];
}

int* DomainDecompositionFramebased::getRemovedDOFs(int domainid)
{
	return DomainRemoveDofs[domainid];
}


void DomainDecompositionFramebased::searchBorderElement()
{
	int nodePerElement = volumetricmesh->getNumElementVertices();
	region_border_elements.resize(numDomains);
	for (int i = 0; i < numDomains; i++)
	{
		std::vector<bool> node_mark(numVertex);
		std::fill(node_mark.begin(), node_mark.end(), false);

		int bordernodesize = region_border[i].size();
		for (int j = 0; j < bordernodesize; j++)
		{
			node_mark[region_border[i][j]] = true;
		}

		for (int j = 0; j < numElements; j++)
		{
			bool assume_is_in = true;
			for (int k = 0; k < nodePerElement; k++)
			{
				int nodeid = volumetricmesh->getElementNode(j, k);
				if (!node_mark[nodeid])
				{
					assume_is_in = false;
				}
			}
			if (assume_is_in)
			{
				region_border_elements[i].push_back(j);
			}
		}
	}
}

void DomainDecompositionFramebased::fillDomain()
{
	// fill each domain by most distance neighbor seed.
	// if element's distance is less than most distance neighbor seed
	// It will be set to this domain.
	// If after fill, an element is still not belongs to any domain,
	// will search a nearest one.

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
		double influence_distace;
		int mostdistace_neighborseed = seeds_index[this->mostSeed[i]];
		influence_distace = this->getNodeToFrameDistance(mostdistace_neighborseed, i);

		std::cout <<"domain "<<i<< " influence_distacne :" << influence_distace << std::endl;

		for (int j = 0; j < numElements; j++)
		{
			bool assume_is_in = true;
			for (int k = 0; k < nodePerElement; k++)
			{
				int nodeid = volumetricmesh->getElementNode(j, k);
				double distance = this->getNodeToFrameDistance(nodeid, i);

				if (distance > influence_distace)
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

		for (int j = 0; j < numVertex; j++)
		{
			double distance = this->getNodeToFrameDistance(j, i);
			if (distance <= influence_distace)
			{
				nodesDomains[j].push_back(i);
				domainsNodes[i].push_back(j);
			}
		}
	}

	//search for non visited element.
	for (int i = 0; i < numVertex; i++)
	{
		if (nodesDomains[i].size() == 0)
		{
			double minDistance = DBL_MAX;
			int minDomain = -1;
			for (int j = 0; j < numDomains; j++)
			{
				double distance = this->getNodeToFrameDistance(i, j);
				if (distance < minDistance)
				{
					minDistance = distance;
					minDomain = j;
				}
			}
			nodesDomains[i].push_back(minDomain);
			domainsNodes[minDomain].push_back(i);
		}
	}

	for (int i = 0; i < numElements; i++)
	{
		if (elementDomains[i].size() == 0)
		{
			double minDistance = DBL_MAX;
			int minDomain = -1;
			for (int j = 0; j < numDomains; j++)
			{
				double distance = this->getEleToFrameDistance(i, j);
				if (distance < minDistance)
				{
					minDistance = distance;
					minDomain = j;
				}
			}
			elementDomains[i].push_back(minDomain);
			domainsElements[minDomain].push_back(i);
		}
	}
}

void DomainDecompositionFramebased::mergeConstraint(int domainid, std::vector<bool>& global_mark)
{
	std::vector<bool> constrained_mark(numVertex);
	std::fill(constrained_mark.begin(), constrained_mark.end(), false);
	for (int i = 0; i < region_all_constraints[domainid].size(); i++)
	{
		constrained_mark[region_all_constraints[domainid][i]] = true;
	}
	int domaindnodesize = this->domainsNodes[domainid].size();
	
	for (int i = 0; i < domaindnodesize; i++)
	{
		int nodeid = domainsNodes[domainid][i];
		if (global_mark[nodeid])
		{
			constrained_mark[nodeid] = true;
		}
	}

	region_all_constraints[domainid].clear();
	for (int i = 0; i < numVertex; i++)
	{
		if (constrained_mark[i])
		{
			region_all_constraints[domainid].push_back(i);
		}
	}
}

void DomainDecompositionFramebased::createDomainConstraints()
{
	// two type of constraints here
	// one is domains border
	// other is inside neighbor seeds's sphere constraints

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
				if (neighborid!=-1)
				if (!findElement(nodesDomains[neighborid], i))
				{
					//ele is a border
					region_border[i].push_back(nodeid);
					break;
				}
			}//end for e
		}
	}
	
	for (int i = 0; i < numDomains; i++)
	{
		int domainid = i;
		int neighborsize = this->seed_neighbor_graph[i].size();
		std::vector<bool> marks(numVertex);
		std::fill(marks.begin(), marks.end(), false);
		for (int j = 0; j < neighborsize; j++)
		{
			int neighborid = this->seed_neighbor_graph[i][j];
			if (neighborid != mostSeed[i])
			{
				//this need to be constraint
				searchInsideBorder(domainid, this->seeds_index[neighborid], marks);
			}
		}

		for (int j = 0; j < numVertex; j++)
		{
			if (marks[j])
			{
				region_inside_constraint[domainid].push_back(j);
			}
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

void DomainDecompositionFramebased::searchInsideBorder(int frameid, int neighborseed, std::vector<bool> &mark)
{
	std::vector<bool> insidemark(numVertex);
	std::fill(insidemark.begin(), insidemark.end(), false);
	std::vector<int> framecontainnodes;

	double influencedistance = this->getNodeToFrameDistance(neighborseed, frameid);

	//search in subnodes
	int numSubnodes = domainsNodes[frameid].size();

	for (int i = 0; i < numSubnodes; i++)
	{
		double distance = this->getNodeToFrameDistance(domainsNodes[frameid][i], frameid);
		if (distance <= influencedistance)
		{
			insidemark[domainsNodes[frameid][i]] = true;
			framecontainnodes.push_back(domainsNodes[frameid][i]);
		}
	}

	//search border
	for (int i = 0; i < framecontainnodes.size(); i++)
	{
		int nodeid = framecontainnodes[i];
		LoboNodeBase * node = volumetricmesh->getNodeRef(nodeid);
		int neighborsize = node->neighbor.size();
		for (int j = 0; j < neighborsize; j++)
		{
			int neighborid = node->neighbor[j];
			if (neighborid != -1)
			{
				if (insidemark[neighborid] == false)
				{
					mark[nodeid] = true;
					break;
				}
			}
		}
	}
}

void DomainDecompositionFramebased::findMostDistanceNeighborseed()
{
	mostSeed.resize(numDomains);
	for (int i = 0; i < numDomains; i++)
	{
		int neighborsize = seed_neighbor_graph[i].size();
		int mostdistanceid = -1;
		double mostdistance = -DBL_MAX;
		for (int j = 0; j < neighborsize; j++)
		{
			int neighborid = seed_neighbor_graph[i][j];
			int neighborseed = this->seeds_index[neighborid];
			double distance = this->getNodeToFrameDistance(neighborseed, i);
			if (distance > mostdistance)
			{
				mostdistance = distance;
				mostdistanceid = neighborid;
			}
		}
		mostSeed[i] = mostdistanceid;
	}

	for (int i = 0; i < numDomains; i++)
	{
		std::cout<<"Domain "<<i<<"most distance neighbor " << mostSeed[i] << std::endl;
	}
}

void DomainDecompositionFramebased::computeDistanceToFrameSeed()
{
	distancetoFrame.setZero();
	elementDistancetoFrame.setZero();
	for (int i = 0; i < numDomains; i++)
	{
		int frame_seed = seeds_index[i];
		std::vector<double> nodedistance;
		if (volumetricmesh_graph != NULL)
		{
			volumetricmesh_graph->compute_dijkstra_shortest_paths(frame_seed, nodedistance);
		}
		else
		{
			computeDijkstraDistance(volumetricmesh, frame_seed, nodedistance);
		}

		for (int j = 0; j < numVertex; j++)
		{
			distancetoFrame.data()[i*numVertex + j] = nodedistance[j];
		}
	}

	int nodePerElement = volumetricmesh->getNumElementVertices();


	for (int frameid = 0; frameid < numDomains;frameid++)
	for (int i = 0; i < numElements; i++)
	{
		double distance = 0;
		for (int j = 0; j < nodePerElement; j++)
		{
			int nodeid = volumetricmesh->getElementNode(i, j);

			distance += this->getNodeToFrameDistance(nodeid, frameid);
		}
		distance /= nodePerElement;
		elementDistancetoFrame.data()[frameid*numElements + i] = distance;
	}
}

void DomainDecompositionFramebased::searchNeighborDomainByNodes()
{
	partition_nodemark.resize(numVertex);
	std::fill(partition_nodemark.begin(), partition_nodemark.end(), -1);
	
	for (int i = 0; i < distance_region_info.size(); i++)
	{
		distance_region_info[i].clear();
	}
	distance_region_info.clear();

	distance_region_info.resize(numDomains);

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

		partition_nodemark[i] = belongregion;
		distance_region_info[belongregion].push_back(i);
	}

	seed_neighbor_graph.clear();
	seed_neighbor_graph.resize(numDomains);

	for (int i = 0; i < numDomains; i++)
	{
		int region_size = distance_region_info[i].size();
		for (int j = 0; j < region_size; j++)
		{
			int node = distance_region_info[i][j];
			LoboNodeBase* noderef = volumetricmesh->getNodeRef(node);
			int neighborsize = noderef->neighbor.size();
			for (int k = 0; k < neighborsize; k++)
			{
				int neighborid = noderef->neighbor[k];

				if (partition_nodemark[neighborid] != i)
				{
					int neighborregion = partition_nodemark[neighborid];

					if (!findElement(seed_neighbor_graph[i], neighborregion))
						seed_neighbor_graph[i].push_back(neighborregion);
				}
			}
		}
	}

}

void DomainDecompositionFramebased::searchNeighborDomain()
{
	region_elemark.resize(numElements);
	std::fill(region_elemark.begin(), region_elemark.end(), -1);
	distance_region_info.resize(numDomains);

	for (int i = 0; i < numElements; i++)
	{
		double mindistance = DBL_MAX;
		int belongregion = -1;
		for (int j = 0; j < numDomains; j++)
		{
			double distance = this->getEleToFrameDistance(i, j);
			if (distance < mindistance)
			{
				mindistance = distance;
				belongregion = j;
			}
		}
		region_elemark[i] = belongregion;
		distance_region_info[belongregion].push_back(i);
	}

	seed_neighbor_graph.resize(numDomains);

	for (int i = 0; i < numDomains; i++)
	{
		int region_size = distance_region_info[i].size();
		for (int j = 0; j < region_size; j++)
		{
			int ele = distance_region_info[i][j];
			int neighborsize = volumetricmesh->getNumElementNeighbors(ele);

			for (int e = 0; e < neighborsize; e++)
			{
				int neighborid = volumetricmesh->getElmentNeighborid(ele, e);
				if (neighborid!=-1)
				if (region_elemark[neighborid] != i)
				{
					int neighborreiong = region_elemark[neighborid];

					if (!findElement(seed_neighbor_graph[i], neighborreiong))
					seed_neighbor_graph[i].push_back(neighborreiong);
				}
			}//end for e
		}//end for j
	}//end for i


}

double DomainDecompositionFramebased::getNodeToFrameDistance(int nodeid, int frameid)
{
	return distancetoFrame.data()[frameid*numVertex + nodeid];
}

double DomainDecompositionFramebased::getEleToFrameDistance(int eleid, int frameid)
{
	return elementDistancetoFrame.data()[frameid*numElements + eleid];
}


