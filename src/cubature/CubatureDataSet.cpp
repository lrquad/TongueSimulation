#include "CubatureDataSet.h"
#include "cubature/CubaturePartition.h"
#include <fstream>
#include <iostream>

CubatureDataSet::CubatureDataSet(int numPoses, CubaturePartition* cubaturePartition)
{
	this->cubaturePartition = cubaturePartition;
	this->numPoses = numPoses;
	numPartition = 0;
	r = 0;
	pose_ratio = 1;

	if (cubaturePartition!=NULL)
	numPartition = cubaturePartition->getNumPartition();
}

CubatureDataSet::~CubatureDataSet()
{

}

void CubatureDataSet::saveDataSet(const char* filename)
{
	std::cout << "save cubature data.." << std::endl;
	std::ofstream outputstream(filename);
	outputstream << numPartition << std::endl;
	outputstream << numPoses << std::endl;
	
	for (int i = 0; i < numPoses; i++)
	{
		for (int j = 0; j < r; j++)
		{
			outputstream << modelPoseSet[i][j] << std::endl;;
		}
	}

	for (int i = 0; i < numPartition; i++)
	{
		outputstream << partitionR[i] << std::endl;
		for (int j = 0; j < numPoses; j++)
		{
			int forceid = i*numPoses + j;
			for (int k = 0; k < r; k++)
			{
				outputstream << internalforceSet[forceid][k] << std::endl;
			}
		}
	}

	outputstream.close();
}

void CubatureDataSet::readDataSet(const char* filename)
{
	std::cout << "read " << filename << std::endl;
	std::ifstream inputstream(filename);
	inputstream >> numPartition;
	inputstream >> numPoses;

	for (int i = 0; i < numPoses; i++)
	{
		for (int j = 0; j < r; j++)
		{
			inputstream >> modelPoseSet[i][j];
		}
	}

	for (int i = 0; i < numPartition; i++)
	{
		inputstream >> partitionR[i];
		for (int j = 0; j < numPoses; j++)
		{
			int forceid = i*numPoses + j;
			for (int k = 0; k < r; k++)
			{
				inputstream >> internalforceSet[forceid][k];
			}
		}
	}
	inputstream.close();
}

void CubatureDataSet::changePoseMagnitude(double ratio)
{	
	pose_ratio = ratio;
}

void CubatureDataSet::getModelPose(int poseid, double* displacement)
{
	memcpy(displacement, modelPoseSet[poseid], sizeof(double)*r);
	for (int i = 0; i < r; i++)
	{
		displacement[i] *= pose_ratio;
	}
}

void CubatureDataSet::setModelPose(int poseid, double* displacement)
{
	memcpy(modelPoseSet[poseid], displacement, sizeof(double)*r);
	for (int i = 0; i < r; i++)
	{
		modelPoseSet[poseid][i] /= pose_ratio;
	}
}

void CubatureDataSet::getInternalForce(int poseid, int partitionId, double* internalforce)
{
	int forceid = partitionId*numPoses+poseid;
	memcpy(internalforce, this->internalforceSet[forceid], sizeof(double)*r);
}

