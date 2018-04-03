#include "ReducedPoseDataSet.h"
#include <fstream>
#include <iostream>
#include "Functions/deleteSTDvector.h"

ReducedPoseDataSet::ReducedPoseDataSet(int R, int r, MatrixXd subspacePhi) :PoseDataSet(R)
{
	this->r = r;
	this->r = subspacePhi.cols();
	this->subspacePhi = subspacePhi;
	leastsqure_solver.compute(subspacePhi.transpose()*subspacePhi);
}


ReducedPoseDataSet::ReducedPoseDataSet(int R) :PoseDataSet(R)
{
	this->r = 0;
}

ReducedPoseDataSet::~ReducedPoseDataSet()
{
	deleteStdvectorPointer(reduced_pose);
	deleteStdvectorPointer(fited_full_pose);
}

void ReducedPoseDataSet::fitFromFullpose()
{
	if (r == 0)
	{
		std::cout << "date init failed";
		return;
	}

	if (full_pose.size() != numPoses)
	{
		std::cout << "full_pose size: " << full_pose.size() << std::endl;
		std::cout << "non full pose found" << std::endl;
	}

	deleteStdvectorPointer(reduced_pose);
	this->reduced_pose.resize(numPoses);
	deleteStdvectorPointer(fited_full_pose);
	fited_full_pose.resize(numPoses);

	for (int i = 0; i < numPoses; i++)
	{
		this->reduced_pose[i] = new VectorXd(r);
		*reduced_pose[i] = leastsqure_solver.solve(subspacePhi.transpose()*(*full_pose[i]));

		fited_full_pose[i] = new VectorXd(R);
		*fited_full_pose[i] = (subspacePhi)*(*reduced_pose[i]);
	}
}

void ReducedPoseDataSet::computeFitError()
{
	double error = 0;
	double maxerror = -DBL_MAX;
	for (int i = 0; i < numPoses; i++)
	{
		VectorXd errorvector(fited_full_pose[i]->rows());
		for (int k = 0; k < fited_full_pose[i]->rows(); k++)
		{
			errorvector[k] = std::abs(fited_full_pose[i]->data()[k] - full_pose[i]->data()[k]) / std::abs(full_pose[i]->data()[k]);

			if (std::abs(full_pose[i]->data()[k]) < 1e-10)
			{
				errorvector[k] = 0;
			}
		}

		double poseerror = (*fited_full_pose[i] - *full_pose[i]).norm() / full_pose[i]->norm();
		//error += errorvector.maxCoeff();
		error += poseerror;
		if (poseerror > maxerror)
		{
			maxerror = poseerror;
		}

	}
	std::cout << "fit relative error" << maxerror << std::endl;
}

void ReducedPoseDataSet::setSubspacePhi(MatrixXd phi)
{
	this->r = phi.cols();
	this->subspacePhi = phi;
	leastsqure_solver.compute(subspacePhi.transpose()*subspacePhi);
}

void ReducedPoseDataSet::loadReducedPoseFile(const char* filename)
{
	std::ifstream inputstream(filename);
	int buffer;
	inputstream >> numPoses;
	inputstream >> buffer;
	if (buffer != r)
	{
		inputstream.close();
		std::cout << "This is not reduced pose file.." << std::endl;
		return;
	}
	r = buffer;

	deleteStdvectorPointer(reduced_pose);
	this->reduced_pose.resize(numPoses);
	for (int i = 0; i < numPoses; i++)
	{
		this->reduced_pose[i] = new VectorXd(r);
		for (int j = 0; j < r; j++)
		{
			inputstream >> reduced_pose[i]->data()[j];
		}
	}
	inputstream.close();
}

void ReducedPoseDataSet::saveReducedPoseFile(const char* filename)
{
	std::ofstream outputstream(filename);
	outputstream << numPoses << std::endl;
	outputstream << r << std::endl;
	for (int i = 0; i < numPoses; i++)
	{
		for (int j = 0; j < r; j++)
		{
			outputstream << reduced_pose[i]->data()[j] << std::endl;
		}
	}
	outputstream.close();
}

Eigen::VectorXd ReducedPoseDataSet::getReducedPose(int poseid)
{
	return *reduced_pose[poseid];
}

Eigen::VectorXd ReducedPoseDataSet::getFitedFullPose(int poseid)
{
	return *fited_full_pose[poseid];
}

void ReducedPoseDataSet::clearData()
{
	numPoses = 0;
	deleteStdvectorPointer(reduced_pose);
	deleteStdvectorPointer(fited_full_pose);
	deleteStdvectorPointer(full_pose);
}

int ReducedPoseDataSet::getr()
{
	return r;
}

