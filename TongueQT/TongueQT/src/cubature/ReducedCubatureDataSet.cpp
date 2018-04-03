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
	setIsRigidModes(false);
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
		if (getIsRigidModes())
		{
			std::cout << "is rigid" << std::endl;
			partitionR[i] += 6;
		}
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

void ReducedCubatureDataSet::readPoseFromFileBinary(const char* filename)
{
	std::cout << filename << std::endl;
	std::ifstream in(filename, std::ios::in | std::ios::binary);
	if (!in.good())
	{
		std::cout << "file not open" << std::endl;
		return;
	}

	for (int i = 0; i < modelPoseSet.size(); i++)
	{
		free(modelPoseSet[i]);
	}

	int numTrainingPose;
	in.read((char*)(&numTrainingPose), sizeof(int));

	std::vector<VectorXd> sequence_q;

	for (int i = 0; i < numTrainingPose; i++)
	{
		int vectorsize;
		in.read((char*)(&vectorsize), sizeof(int));
		VectorXd temp(vectorsize);
		in.read((char*)temp.data(), vectorsize*sizeof(VectorXd::Scalar));
		sequence_q.push_back(temp);
	}
	
	in.close();

	//filterPoses(sequence_q);

	std::cout << sequence_q.size() <<"pose read..." << std::endl;

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
				modelPoseSet[i][j] = sequence_q[i].data()[j];
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
			pose = sequence_q[i];

			if (i%skip == 0 && i != 0 && offset < numPoses)
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

	std::cout << "init internalforce" << std::endl;
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
		std::vector<int> eleSet = cubaturePartition->getpartitionEleIndices(partitionid);
		this->getModelPose(i, q);
		reducedforcemodel->getInternalForce(q, internalforceSet[forceid], &eleSet);
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
