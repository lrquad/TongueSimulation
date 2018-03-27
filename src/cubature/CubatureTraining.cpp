#include "CubatureTraining.h"
#include <iostream>
#include <fstream>

CubatureTraining::CubatureTraining(CubatureDataSet* cubaturedateset, CubaturePartition * cubaturepartition)
{
	greedyThreshold = 0.03; //default
	this->cubatureDataSet = cubaturedateset;
	this->cubaturePartition = cubaturepartition;
}

CubatureTraining::CubatureTraining(const char* filename)
{
	readCubatures(filename);
}

CubatureTraining::CubatureTraining()
{
	greedyThreshold = 0.03; //default
	this->cubatureDataSet = NULL;
	this->cubaturePartition = NULL;
}

CubatureTraining::~CubatureTraining()
{

}

std::vector<int> CubatureTraining::getCubatureSet()
{
	return cubatureIndices;
}

std::vector<double> CubatureTraining::getCubatureWeightSet()
{
	return cubatureWeight;
}

std::vector<int>* CubatureTraining::getCubatureSetRef()
{
	return &cubatureIndices;
}

std::vector<double>* CubatureTraining::getCubatureWeightSetRef()
{
	return &cubatureWeight;
}

void CubatureTraining::saveCubatures(const char* filename)
{
	std::cout << "save cubature points ->" << filename << std::endl;
	std::ofstream outputstream(filename);
	outputstream.precision(32);
	outputstream << numCubatures << std::endl;
	for (int i = 0; i < numCubatures; i++)
	{
		outputstream << this->cubatureIndices[i] << " ";
	}
	outputstream << std::endl;
	for (int i = 0; i < numCubatures; i++)
	{
		outputstream << this->cubatureWeight[i] << " ";
	}
	outputstream << std::endl;
	outputstream.close();
}

void CubatureTraining::saveCubaturesBinary(std::ofstream &outstream)
{
	int size = cubatureIndices.size();
	outstream.write((char*)&size, sizeof(int));
	outstream.write((char*)cubatureIndices.data(), sizeof(int)*cubatureIndices.size());
	outstream.write((char*)cubatureWeight.data(), sizeof(double)*cubatureWeight.size());
}

void CubatureTraining::readCubaturesBinary(std::ifstream &instream)
{
	int size;
	instream.read((char*)&size, sizeof(int));
	cubatureIndices.resize(size);
	instream.read((char*)cubatureIndices.data(), sizeof(int)*cubatureIndices.size());
	
	cubatureWeight.resize(size);
	instream.read((char*)cubatureWeight.data(), sizeof(double)*cubatureWeight.size());

}

int CubatureTraining::readCubatures(const char* filename)
{
	std::cout << "read cubature points ->" << filename << std::endl;
	std::ifstream inputstream(filename);

	if (!inputstream.is_open())
	{
		return 0;
	}

	inputstream >> numCubatures;
	cubatureIndices.resize(numCubatures);
	for (int i = 0; i < numCubatures; i++)
	{
		inputstream >> cubatureIndices[i];
	}
	cubatureWeight.resize(numCubatures);
	for (int i = 0; i < numCubatures; i++)
	{
		inputstream >> cubatureWeight[i];
	}
	inputstream.close();

	return 1;
}

void CubatureTraining::mergeCubaturePoints()
{
	cubatureIndices.clear();
	cubatureWeight.clear();

	for (int i = 0; i < partitionCubatureIndices.size(); i++)
	{
		for (int j = 0; j < partitionCubatureIndices[i].size(); j++)
		{
			if (partitionCubatureWeights[i][j] == 0)
			{
				continue;
			}
			cubatureIndices.push_back(partitionCubatureIndices[i][j]);
			cubatureWeight.push_back(partitionCubatureWeights[i][j]);
		}
	}
	

	std::cout << "We have :" << cubatureIndices.size() << "cubature points." << std::endl;
	numCubatures = cubatureIndices.size();
}
