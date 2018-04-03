#include "CubaturePartition.h"
#include "DomainDecomposition/DomainDecomposition.h"
#include "LoboVolumetricMesh/LoboVolumetricMesh.h"
#include <fstream>
#include <iostream>

CubaturePartition::CubaturePartition(DomainDecomposition* domaindecompsition, LoboVolumetricMesh* volumetricMesh)
{
	this->volumetricMesh = volumetricMesh;
	this->domaindecompsition = domaindecompsition;
}


CubaturePartition::~CubaturePartition()
{

}

void CubaturePartition::init()
{
	initPartitionNodeIndices(); 
	initPartitionEleIndices();
	//intiPartitionEleIndicesByNode();

	generatePartitionDOFs();

}

void CubaturePartition::initPartitionNodeIndices()
{
	int numDomains = domaindecompsition->getNumDomain();
	int numVertex = volumetricMesh->getNumVertices();
	vertexPartition.resize(numVertex);
	std::fill(vertexPartition.begin(), vertexPartition.end(), -1);

	for (int i = 0; i < numVertex; i++)
	{
		std::vector<int> flags(numDomains);
		std::fill(flags.begin(), flags.end(), 0);

		std::vector<int> nodeDomains = domaindecompsition->getNodeDomains(i);
		int nodedomainsize = nodeDomains.size();

		for (int j = 0; j < nodedomainsize; j++)
		{
			int domainid = nodeDomains[j];
			flags[domainid] = 1;
		}//end for j 

		//check which partition this vertex belongs to.
		//If not found matched partition, add this new partition.
		int matchedPartitionFlag = -1;

		for (int j = 0; j < partitionFlag.size(); j++)
		{
			bool assumematch = true;

			for (int k = 0; k < partitionFlag[j].size(); k++)
			{
				if (partitionFlag[j][k] != flags[k])
				{
					assumematch = false;
				}
			}

			if (assumematch == true)
			{
				matchedPartitionFlag = j;
				break;
			}
		}// end for j

		if (matchedPartitionFlag == -1)
		{
			partitionFlag.push_back(flags);
			vertexPartition[i] = partitionFlag.size() - 1;
		}
		else
		{
			vertexPartition[i] = matchedPartitionFlag;
		}
	}

	numPartition = partitionFlag.size();
	partitionIndices.resize(numPartition);

	for (int i = 0; i < numVertex; i++)
	{
		partitionIndices[vertexPartition[i]].push_back(i);
	}
}

void CubaturePartition::initPartitionEleIndices()
{
	int numDomains = domaindecompsition->getNumDomain();
	int numElement = volumetricMesh->getNumElements();

	elementPartition.resize(numElement);
	std::fill(elementPartition.begin(), elementPartition.end(), -1);

	for (int i = 0; i < numElement; i++)
	{
		std::vector<int> flags(numDomains);
		std::fill(flags.begin(), flags.end(), 0);
		std::vector<int> elendomains = domaindecompsition->getElementDomains(i);
		int eledomainssize = elendomains.size();
		for (int j = 0; j < eledomainssize; j++)
		{
			flags[elendomains[j]] = 1;
		}

		// search in partition
		int matchedPartitionFlag = -1;
		for (int j = 0; j < partitionFlag.size(); j++)
		{
			bool assumematch = true;

			for (int k = 0; k < partitionFlag[j].size(); k++)
			{
				if (partitionFlag[j][k] != flags[k])
				{
					assumematch = false;
				}
			}

			if (assumematch == true)
			{
				matchedPartitionFlag = j;
				break;
			}
		}// end for j

		if (matchedPartitionFlag == -1)
		{
			std::cout << "You need to check domain decomposition. There are some elements may outside the domain." << std::endl;
		}

		if (matchedPartitionFlag == -1)
		{
			partitionFlag.push_back(flags);
			elementPartition[i] = partitionFlag.size() - 1;
		}
		else
		{
			elementPartition[i] = matchedPartitionFlag;
		}
	}

	numPartition = partitionFlag.size();
	partitionEleIndices.resize(numPartition);

	for (int i = 0; i < numElement; i++)
	{
		partitionEleIndices[elementPartition[i]].push_back(i);
	}

}

void CubaturePartition::intiPartitionEleIndicesByNode()
{
	int numElement = volumetricMesh->getNumElements();
	int numDomains = domaindecompsition->getNumDomain();
	elementPartition.resize(numElement);
	std::fill(elementPartition.begin(), elementPartition.end(), -1);
	int numEleVertices = volumetricMesh->getNumElementVertices();
	
	numPartition = partitionFlag.size();
	partitionEleIndices.resize(numPartition);

	for (int i = 0; i < numElement; i++)
	{
		std::vector<bool> elementPartition(numPartition);
		std::fill(elementPartition.begin(), elementPartition.end(), false);

		for (int j = 0; j < numEleVertices; j++)
		{
			int nodeid = volumetricMesh->getElementNode(i, j);
			int nodepartition = vertexPartition[nodeid];
			elementPartition[nodepartition] = true;
		}

		for (int j = 0; j < elementPartition.size(); j++)
		{
			if (elementPartition[j])
			{
				partitionEleIndices[j].push_back(i);
			}
		}
	}
}

void CubaturePartition::generatePartitionDOFs()
{
	int numDomains = domaindecompsition->getNumDomain();

	int numElements = volumetricMesh->getNumElements();
	int numVertex = volumetricMesh->getNumVertices();

	partitionDOFs.resize(numPartition);
	for (int i = 0; i < numPartition; i++)
	{
		for (int j = 0; j < numDomains; j++)
		{
			if (partitionFlag[i][j] == 1)
			{
				partitionDOFs[i].push_back(j);
			}
		}
	}
}

void CubaturePartition::saveCubaturePartition(const char* filename)
{
	int numDomains = domaindecompsition->getNumDomain();
	int numVertex = volumetricMesh->getNumVertices();
	int numElements = volumetricMesh->getNumElements();

	std::ofstream outputstream(filename);
	outputstream << numPartition << std::endl;
	outputstream << numDomains << std::endl;
	outputstream << numVertex << std::endl;
	outputstream << numElements << std::endl;
	//output partition flag info
	for (int i = 0; i < numPartition; i++)
	{
		for (int j = 0; j < numDomains; j++)
		{
			outputstream << partitionFlag[i][j] << " ";
		}
		outputstream << std::endl;
	}

	//output flag mark
	for (int i = 0; i < numVertex; i++)
	{
		outputstream << vertexPartition[i] << std::endl;
	}

	//output element indices
	for (int i = 0; i < numPartition; i++)
	{
		outputstream << partitionEleIndices[i].size() << " ";
		for (int j = 0; j < partitionEleIndices[i].size(); j++)
		{
			outputstream << partitionEleIndices[i][j] << " ";
		}
		outputstream << std::endl;
	}

	outputstream.close();
}

void CubaturePartition::readCubaturePartition(const char* filename)
{
	int numDomains;
	int numVertex;
	int numElements;
	std::ifstream inputstream(filename);
	inputstream >> numPartition;
	inputstream >> numDomains;
	inputstream >> numVertex;
	inputstream >> numElements;

	partitionFlag.resize(numPartition);
	for (int i = 0; i < numPartition; i++)
	{
		partitionFlag[i].resize(numDomains);
		for (int j = 0; j < numDomains; j++)
		{
			inputstream >> partitionFlag[i][j];
		}
	}

	vertexPartition.resize(numVertex);
	for (int i = 0; i < numVertex; i++)
	{
		inputstream >> vertexPartition[i];
	}

	partitionEleIndices.resize(numPartition);
	for (int i = 0; i < numPartition; i++)
	{
		int elesize;
		inputstream >> elesize;
		partitionEleIndices[i].resize(elesize);
		for (int j = 0; j < elesize; j++)
		{
			inputstream >> partitionEleIndices[i][j];
		}
	}

	inputstream.close();

	generatePartitionDOFs();

	partitionIndices.clear();
	partitionIndices.resize(numPartition);
	for (int i = 0; i < numVertex; i++)
	{
		partitionIndices[vertexPartition[i]].push_back(i);
	}
}

int CubaturePartition::getNumPartition()
{
	return numPartition;
}

std::vector<int> CubaturePartition::getPartitionIndices(int partitionId)
{
	return partitionIndices[partitionId];
}

std::vector<int> CubaturePartition::getpartitionEleIndices(int partitionId)
{
	return partitionEleIndices[partitionId];
}

std::vector<int> CubaturePartition::getPartitionDomainIndices(int partitionId)
{
	return partitionDOFs[partitionId];
}

int CubaturePartition::getNumElements()
{
	return volumetricMesh->getNumElements();
}

