#include "ReducedCubatureDataSet.h"
#include "CubaturePartition.h"
#include "Simulator/ReducedForceModel/ReducedForceModel.h"
#include <time.h>
#include <iostream>
#include <fstream>

ReducedCubatureDataSet::ReducedCubatureDataSet(int numPose, ReducedForceModel* reducedforcemodel, CubaturePartition* cubaturePartition, int DOFs_Per_Domain, double qAmplitude) :CubatureDataSet(numPose, cubaturePartition)
{
	this->reducedforcemodel = reducedforcemodel;
	r = reducedforcemodel->getR();
	this->qAmplitude = qAmplitude;
	this->DOFs_Per_Domain = DOFs_Per_Domain;
}

ReducedCubatureDataSet::~ReducedCubatureDataSet()
{
	for (int i = 0; i < modelPoseSet.size(); i++)
	{
		free(modelPoseSet[i]);
	}

	for (int i = 0; i < internalforceSet.size(); i++)
	{
		free(internalforceSet[i]);
	}
}

void ReducedCubatureDataSet::generateDataSet()
{
	numPartition = cubaturePartition->getNumPartition();
	
	partitionR.resize(numPartition);
	partitionIndices.resize(numPartition);
	modelPoseSet.resize(numPoses);
	internalforceSet.resize(numPartition*numPoses);

	for (int i = 0; i < numPartition; i++)
	{
		partitionIndices[i] = cubaturePartition->getPartitionIndices(i);
		partitionR[i] = cubaturePartition->getPartitionDomainIndices(i).size()*DOFs_Per_Domain;
	}

	generateRandomPose();
	initInternalForce();
	//computeInternalForce();
}

void ReducedCubatureDataSet::computeInternalForce()
{
	for (int i = 0; i < numPartition; i++)
	{
		computePartitionInternalForce(i);
	}
	reducedforcemodel->resetModel();
}

void ReducedCubatureDataSet::savePoseToFile(const char* filename)
{
	std::ofstream outputStream(filename);
	outputStream << numPoses << std::endl;
	for (int i = 0; i < numPoses; i++)
	{
		for (int j = 0; j < r; j++)
		{
			outputStream << modelPoseSet[i][j] << std::endl;
		}
	}
	outputStream.close();
}

void ReducedCubatureDataSet::readPoseFromFile(const char* filename)
{
	//clean old poses mem

	for (int i = 0; i < numPoses; i++)
	{
		free(modelPoseSet[i]);
	}

	std::ifstream inputstream(filename);
	int numTrainingPose;
	inputstream >> numTrainingPose;
	if (numTrainingPose < numPoses)
	{
		numPoses = numTrainingPose;
		modelPoseSet.resize(numPoses);
		internalforceSet.resize(numPartition*numPoses);
		for (int i = 0; i < numTrainingPose; i++)
		{
			modelPoseSet[i] = (double*)malloc(sizeof(double)*r);
			for (int j = 0; j < r; j++)
			{
				inputstream >> modelPoseSet[i][j];
			}
		}
	}

	if (numTrainingPose >= numPoses)
	{
		int skip = numTrainingPose / numPoses;
		std::cout << "skip" << skip << std::endl;
		VectorXd pose(r);
		int offset = 0;
		for (int i = 0; i < numTrainingPose; i++)
		{
			for (int j = 0; j < r; j++)
			{
				inputstream >> pose.data()[j];
			}

			if (i%skip == 0&&i!=0&&offset<numPoses)
			{
				modelPoseSet[offset] = (double*)malloc(sizeof(double)*r);
				for (int j = 0; j < r; j++)
				{
					modelPoseSet[offset][j] = pose.data()[j];
				}
				offset++;
			}
		}
	}
	inputstream.close();
	
	initInternalForce();
	std::cout << "num pose:" << numPoses << std::endl;
}

void ReducedCubatureDataSet::readPoseFromFileAndNumPoses(const char* filename)
{
	for (int i = 0; i < numPoses; i++)
	{
		free(modelPoseSet[i]);
	}

	std::ifstream inputstream(filename);
	int numTrainingPose;
	inputstream >> numTrainingPose;

	numPoses = numTrainingPose;
	modelPoseSet.resize(numPoses);
	internalforceSet.resize(numPartition*numPoses);
	for (int i = 0; i < numTrainingPose; i++)
	{
		modelPoseSet[i] = (double*)malloc(sizeof(double)*r);
		for (int j = 0; j < r; j++)
		{
			inputstream >> modelPoseSet[i][j];
		}
	}

	inputstream.close();
	initInternalForce();
	std::cout << "num pose:" << numPoses << std::endl;
}

void ReducedCubatureDataSet::initInternalForce()
{
	for (int i = 0; i < numPartition*numPoses; i++)
	{
		free(internalforceSet[i]);
	}

	for (int i = 0; i < numPartition; i++)
	{
		for (int j = 0; j < numPoses; j++)
		{
			int forceid = i*numPoses + j;
			internalforceSet[forceid] = (double*)malloc(sizeof(double)*r);
		}
	}
}

void ReducedCubatureDataSet::computePartitionInternalForce(int partitionid)
{
	double* q = (double*)malloc(sizeof(double)*r);
	
	for (int i = 0; i < numPoses; i++)
	{
		std::cout << "\r" <<"pose: "<< i;
		int forceid = partitionid*numPoses + i;
		std::vector<int> eleSet = cubaturePartition->getPartitionIndices(partitionid);
		this->getModelPose(i, q);
		reducedforcemodel->getInternalForce(q,internalforceSet[forceid],&eleSet);
	}
	std::cout << "\r" << std::endl;
	free(q);
}

void ReducedCubatureDataSet::generateRandomPose()
{
	srand(time(NULL));
	for (int i = 0; i < numPoses; i++)
	{
		modelPoseSet[i] = (double*)malloc(sizeof(double)*r);
		for (int j = 0; j < r; j++)
		{
			modelPoseSet[i][j] = qAmplitude*(-1.0 + 2.0 * rand() / RAND_MAX);
		}
	}
}
