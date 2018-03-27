#pragma once
#include "CubatureTraining.h"

class ReducedForceModel;
class LoboVolumetricMesh;
class ReducedCubatureDataSet;

/*!
 * \class ReducedCubatureTraining
 *
 * \brief I change this class to based on element now.

 * \date August 2016
 */
class ReducedCubatureTraining:public CubatureTraining
{
public:
	ReducedCubatureTraining(ReducedForceModel* reducedForceModel, LoboVolumetricMesh* volumetricmesh, ReducedCubatureDataSet* cubaturedateset, CubaturePartition * cubaturepartition);
	~ReducedCubatureTraining();
	ReducedCubatureTraining(const char* filename);

	virtual void trainingData();

protected:
	//see Optimizing Cubature for Efficient Integration of Subspace Deformations
	//pre compute g vector for all node 
	virtual void initVectorG();

	//release mem
	virtual void releaseVectorG();

	virtual void trainingPartition(int partitionId, double errorthreshold);
	virtual void computeForceB(int partitionId, VectorXd* b);
	virtual void computeMatrixA(int partitionId,std::vector<int> selectedIndices,MatrixXd *A);

	virtual void selectCandidatePoints(int partitionid,std::vector<int>* candidatePointssubIndex,std::vector<int>* candidatePointsGlobalIndex, int candidatesize);

	virtual void chooseMaxPoint(int partitionid, std::vector<int>* candidatePointssubIndex, std::vector<int>* candidatePointsGlobalIndex, VectorXd residual, int &selectedindex);

	/**
	 * @brief Put brief description here
	 *
	 * Put full description here
	 *
	 * @param[in,out] nodeid This is not nodeid anymore, it's element id in volumtric mesh
	 * @param[in,out] partitionId Put argument desc here
	 * @param[in,out] b Put argument desc here
	 * @return Put return information here
	 */
	 virtual void computeAllPoseInternalForce(int nodeid,int partitionId, VectorXd *b);

	virtual void extendMatrixA(int partitionId, int newIndex, MatrixXd* A);
	virtual void mapPartitionDOFs(VectorXd DOFs, VectorXd &subDOFs, std::vector<int> relatedDomain, int DOFS_Per_Domain);



	ReducedForceModel* reducedForceModel;
	LoboVolumetricMesh* volumetricMesh;
	ReducedCubatureDataSet* reducedCubatureDataSet;

	std::vector<std::vector<int>> partitionIndices;
	std::vector<VectorXd> eleInternalForce;
	std::vector<int> partitionIdOffset;
};

