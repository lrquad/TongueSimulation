#include "PoseDataSet.h"
#include <fstream>
#include "Functions/deleteSTDvector.h"
#include <iostream>

PoseDataSet::PoseDataSet(int R)
{
	numPoses = 0;
	this->R = R;
}


PoseDataSet::~PoseDataSet()
{
	deleteStdvectorPointer(full_pose);
}

void PoseDataSet::loadPoseFile(const char* filename)
{
	std::ifstream inputstream(filename);
	inputstream >> numPoses;
	inputstream >> R;

	deleteStdvectorPointer(full_pose);
	full_pose.resize(numPoses);

	for (int i = 0; i < numPoses; i++)
	{
		full_pose[i] = new VectorXd(R);
		for (int j = 0; j < R; j++)
		{
			inputstream >> full_pose[i]->data()[j];
		}
	}
}



void PoseDataSet::savePoseFile(const char* filename)
{
	std::ofstream outputstream(filename);
	outputstream << numPoses << std::endl;
	outputstream << R << std::endl;
	for (int i = 0; i < numPoses; i++)
	{
		for (int j = 0; j < R; j++)
		{
			outputstream << full_pose[i]->data()[j] << std::endl;
		}
	}
}

void PoseDataSet::inversePose()
{
	for (int i = 0; i < numPoses; i++)
	{
		for (int j = 0; j < R; j++)
		{
			if (j%3==0)
			 full_pose[i]->data()[j]*=-1;
		}
	}
}

void PoseDataSet::insertNewPose(double* pose, int verbose /*= 0*/)
{
	VectorXd *fullpose = new VectorXd(R);
	memcpy(fullpose->data(), pose, R*sizeof(double));
	
	this->full_pose.push_back(fullpose);
	numPoses++;

	if (verbose)
	std::cout <<"Number poses: "<< numPoses << std::endl;
}

void PoseDataSet::convertPoseToMatrix(MatrixXd &output)
{
	output.resize(R,numPoses);
	output.setZero();
	for (int i = 0; i < numPoses; i++)
	{
		output.col(i) = *full_pose[i];
	}
}

Eigen::VectorXd PoseDataSet::getFullPose(int poseid)
{
	return *full_pose[poseid];
}

void PoseDataSet::clearData()
{
	numPoses = 0;
	deleteStdvectorPointer(full_pose);
}
