#pragma once
#include <Eigen/Dense>
#include <vector>
using namespace Eigen;

class PoseDataSet
{
public:
	PoseDataSet(int R);
	~PoseDataSet();

	virtual void loadPoseFile(const char* filename);
	virtual void savePoseFile(const char* filename);

	virtual void inversePose();

	virtual void insertNewPose(double* pose, int verbose = 1);
	virtual void convertPoseToMatrix(MatrixXd &output);

	//query
	int getNumPose(){ return numPoses; }
	VectorXd getFullPose(int poseid);
	
	int getR(){ return R; }

	virtual void clearData();

protected: 

	std::string filename;
	int numPoses;
	int R;

	std::vector<VectorXd*> full_pose;

};

