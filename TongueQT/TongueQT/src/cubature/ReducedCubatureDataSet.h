#pragma once
#include "CubatureDataSet.h"

class ReducedForceModel;
class ReducedCubatureDataSet:public CubatureDataSet
{
public:
	ReducedCubatureDataSet(int numPose, ReducedForceModel* reducedforcemodel, CubaturePartition* cubaturePartition, int DOFs_Per_Domain, double qAmplitude = 1.0);
	~ReducedCubatureDataSet();

	virtual void generateDataSet();
	virtual void computeInternalForce();
	virtual void savePoseToFile(const char* filename);
	virtual void readPoseFromFile(const char* filename);
	virtual void readPoseFromFileBinary(const char* filename);
	virtual void readPoseFromFileAndNumPoses(const char* filename);


	int getDOFsPerDomain(){ return DOFs_Per_Domain; };

	bool getIsRigidModes() const { return isRigidModes; }
	void setIsRigidModes(bool val) { isRigidModes = val; }
protected:

	virtual void filterPoses(std::vector<VectorXd> &sequence_q){};
	virtual void initInternalForce();
	virtual void computePartitionInternalForce(int partitionid);
	virtual void generateRandomPose();
	

	int DOFs_Per_Domain;
	ReducedForceModel* reducedforcemodel;

	double qAmplitude;
	bool isRigidModes;
};

