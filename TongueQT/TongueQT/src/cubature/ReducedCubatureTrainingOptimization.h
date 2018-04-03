#pragma once
#include "ReducedCubatureTraining.h"

class ReducedCubatureTrainingOptimization : public ReducedCubatureTraining
{
public:
	ReducedCubatureTrainingOptimization(ReducedForceModel* reducedForceModel, LoboVolumetricMesh* volumetricmesh, ReducedCubatureDataSet* cubaturedateset, CubaturePartition * cubaturepartition);
	~ReducedCubatureTrainingOptimization();

	virtual void trainingData();

	virtual void checkError(double* pose);

protected:

	//override function
	virtual void trainingPartition(int partitionId, double errorthreshold);
	
	virtual void getPoseForce(int partitionId, VectorXd*f,std::vector<int> poseIndex);
	virtual void chooseMaxPointSubSet(int partitionId, std::vector<int> poseIndex, std::vector<int>* candidatePointssubIndex, std::vector<int>* candidatePointsGlobalIndex, VectorXd residual, int &selectedindex);
	virtual void computeSubsetPoseInternalForceNormlized(int nodeid, int partitionId, VectorXd* b, std::vector<int> poseIndex);
	virtual void computeSubsetPoseInternalForce(int nodeid, int partitionId,VectorXd* b, std::vector<int> poseIndex);

	void poseIdGenerator(int numRandomPoses,int totalPoses, std::vector<int>& resultIndex);
	void computeMatrixASubSet(int partitionId, int newIndex, MatrixXd* A, std::vector<int> poseIndex);
	void computeMatrixASubset(int partitionId, std::vector<int> selectedIndices, MatrixXd* subA, std::vector<int> poseIndex);
	virtual double doFullsetTraining(int partitionId,std::vector<int> selectedIndices,VectorXd& weights,VectorXd &b);

	void pushNodeForce(int newnodeid, int partitionId);

	//selected index * numPose
	std::vector<VectorXd> selectedNodesPoseForce;
};

