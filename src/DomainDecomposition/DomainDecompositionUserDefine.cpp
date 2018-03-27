#include "DomainDecompositionUserDefine.h"
#include "LoboVolumetricMesh/LoboVolumetriceMeshCore.h"
#include <fstream>


DomainDecompositionUserDefine::DomainDecompositionUserDefine(LoboVolumetricMesh* volumetricmesh, LoboVolumetricMeshGraph* volumetricmeshgraph, std::vector<int> seeds_index, WeightFunctionLaplacian* weightfunction /*= NULL*/, bool createSubVolumetricMesh /*= false*/) :DomainDecompositionPartitionOnly(volumetricmesh,volumetricmeshgraph,seeds_index,weightfunction,false)
{
	elementDomainMark.resize(volumetricmesh->getNumElements());
	std::fill(elementDomainMark.begin(), elementDomainMark.end(), -1);
	numDomains = 0;
}

DomainDecompositionUserDefine::~DomainDecompositionUserDefine()
{
}

void DomainDecompositionUserDefine::markNewDomain(std::vector<int> elelist)
{
	for (int i = 0; i < elelist.size(); i++)
	{
		elementDomainMark[elelist[i]] = numDomains;
	}
	numDomains++;
}

void DomainDecompositionUserDefine::markEleToExistDomain(std::vector<int> elelist, int targetdomain)
{
	if (targetdomain >= numDomains)
	{
		std::cout << "the domain " << targetdomain << "not exist" << std::endl;
		return;
	}

	for (int i = 0; i < elelist.size(); i++)
	{
		elementDomainMark[elelist[i]] = targetdomain;
	}
}

int DomainDecompositionUserDefine::getDomainidByElement(int eleid)
{
	return elementDomainMark[eleid];
}

void DomainDecompositionUserDefine::doDecomposition()
{
	distancetoFrame.resize(numVertex, numDomains);
	elementDistancetoFrame.resize(numElements, numDomains);

	distancetoFrame.setZero();
	elementDistancetoFrame.setZero();

	for (int frameid = 0; frameid < numDomains; frameid++)
	for (int i = 0; i < numElements;i++)
	{
		double distance = 1;
		if (elementDomainMark[i] == frameid)
		{ 
			distance = 0;
		}
		elementDistancetoFrame.data()[frameid*numElements + i] = distance;
	}

	for (int frameid = 0; frameid < numDomains; frameid++)
	for (int i = 0; i < numVertex; i++)
	{
		LoboNodeBase* node = volumetricmesh->getNodeRef(i);
		double distance = 0;
		for (int j = 0; j < node->element_list.size(); j++)
		{
			int elementid = node->element_list[j];
			distance += elementDistancetoFrame.data()[frameid*numElements + elementid];
		}
		distance /= node->element_list.size();
		distancetoFrame.data()[frameid*numVertex + i] = distance;
	}

	searchNeighborDomainByNodes();

	fillDomain();
	ignoreUnConnectedNodes();
	checkDomainConnection();
	//createDomainConstraints();
	//generateRemoveConstrainInfo();

}

void DomainDecompositionUserDefine::saveDomainMarks(const char* filename)
{
	std::ofstream outputstream(filename);
	outputstream << numDomains << std::endl;
	outputstream << elementDomainMark.size() << std::endl;
	for (int i = 0; i < elementDomainMark.size(); i++)
	{
		outputstream << elementDomainMark[i] << std::endl;
	}
	outputstream.close();
}

void DomainDecompositionUserDefine::readDomainMarks(const char* filename)
{
	std::ifstream inputstream(filename);
	bool b = inputstream.is_open();
	if (!b)
	{
		std::cout << "can't open " << filename << std::endl;
		inputstream.close();
		return;
	}

	inputstream >> numDomains;
	int elementdomainMarkssize;
	inputstream >> elementdomainMarkssize;
	for (int i = 0; i < elementdomainMarkssize; i++)
	{
		inputstream >> elementDomainMark[i];
	}
	inputstream.close();


	std::vector<std::vector<int>> domainEleList;
	domainEleList.resize(numDomains);
	for (int i = 0; i < elementDomainMark.size(); i++)
	{
		if (elementDomainMark[i] != -1)
		{
			int domainid = elementDomainMark[i];
			domainEleList[domainid].push_back(i);
		}
	}

	//clean the empty domain
	bool everythingOk = true;
	int domaincount = 0;
	for (int i = 0; i < numDomains; i++)
	{
		if (domainEleList[i].size() == 0)
		{
			everythingOk = false;
			continue;
		}
		for (int j = 0; j < domainEleList[i].size(); j++)
		{
			int eleid = domainEleList[i][j];
			elementDomainMark[eleid] = domaincount;
		}
		domaincount++;
	}
	numDomains = domaincount;

}

std::vector<int> DomainDecompositionUserDefine::getelementDomainMakr()
{
	return elementDomainMark;
}

void DomainDecompositionUserDefine::checkDomainConnection()
{

}
