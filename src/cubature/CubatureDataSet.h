#pragma once
#include <Eigen/Dense>
#include <Eigen/Sparse>
using namespace Eigen;
class CubaturePartition;
class CubatureDataSet
{
public:
	CubatureDataSet(int numPoses, CubaturePartition* cubaturePartition);
	~CubatureDataSet();

	virtual void generateDataSet() = 0;
	virtual void computeInternalForce() = 0;

	virtual void saveDataSet(const char* filename);
	virtual void readDataSet(const char* filename);

	void changePoseMagnitude(double ratio);

	inline int getR(){ return r; }
	inline virtual int getNumPartition(){ return numPartition; }
	inline virtual int getNumPoses(){ return numPoses;}
	virtual void setNumPoses(double val){ numPoses = val;}
	
	virtual std::vector<int> getPartitionIndices(int partitionId){ return partitionIndices[partitionId]; }

	inline virtual int getPartitionNumDOFs(int partitionId){ return partitionR[partitionId];}

	virtual void getModelPose(int poseid,double* displacement);
	virtual void setModelPose(int poseid, double* displacement);
	virtual void getInternalForce(int poseid, int partitionId, double* internalforce);

protected:

	CubaturePartition* cubaturePartition;

	int r;
	int numPartition;
	int numPoses;
	double pose_ratio;

	std::vector<int> partitionR;
	std::vector<std::vector<int>> partitionIndices;
	std::vector<double*> modelPoseSet;
	std::vector<double*> internalforceSet;

};

