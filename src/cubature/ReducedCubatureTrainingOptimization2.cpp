#include "ReducedCubatureTrainingOptimization.h"
#include "LoboVolumetricMesh/LoboVolumetricMesh.h"
#include "cubature/CubaturePartition.h"
#include "cubature/CubatureDataSet.h"
#include "Simulator/ReducedForceModel/ReducedForceModel.h"
#include "cubature/ReducedCubatureDataSet.h"
#include <iostream>
#include "nnls.h"
#include <fstream>
#include <QElapsedTimer>
#include  <numeric>


ReducedCubatureTrainingOptimization::ReducedCubatureTrainingOptimization(ReducedForceModel* reducedForceModel, LoboVolumetricMesh* volumetricmesh, ReducedCubatureDataSet* cubaturedateset, CubaturePartition * cubaturepartition) :ReducedCubatureTraining(reducedForceModel,volumetricmesh,cubaturedateset,cubaturepartition)
{
	
}

ReducedCubatureTrainingOptimization::~ReducedCubatureTrainingOptimization()
{
}

void ReducedCubatureTrainingOptimization::trainingData()
{
	QElapsedTimer timer;
	timer.start();

	int numParitions = cubaturePartition->getNumPartition();
	int numVertex = volumetricMesh->getNumVertices();

	partitionCubatureIndices.resize(numParitions);
	partitionCubatureWeights.resize(numParitions);
	partitionIndices.resize(numParitions);

	for (int i = 0; i < numParitions; i++)
	{
		partitionIndices[i] = cubaturePartition->getpartitionEleIndices(i);
		trainingPartition(i, greedyThreshold);
	}

	mergeCubaturePoints();
	std::cout << "training time: " << timer.elapsed() <<" ms"<< std::endl;
}

void ReducedCubatureTrainingOptimization::checkError(double* pose)
{
	
}

void ReducedCubatureTrainingOptimization::trainingPartition(int partitionId, double errorthreshold)
{
	std::cout << "start training ..." << std::endl;
	int numPoses = reducedCubatureDataSet->getNumPoses();
	int numRandomPose = 10;//10

	int R = reducedCubatureDataSet->getPartitionNumDOFs(partitionId);
	//random pose generator
	std::vector<int> poseIndex; 
	poseIdGenerator(numRandomPose, numPoses, poseIndex);
	
	//sub training set
	VectorXd b;
	getPoseForce(partitionId, &b, poseIndex);
	VectorXd fullb;
	this->computeForceB(partitionId, &fullb);

	//sub training set
	VectorXd r(b.rows());
	r = b;

	//full training set
	MatrixXd A(numPoses*R, 0);
	//sub training set
	MatrixXd subA(numRandomPose*R, 0);

	VectorXd weights;

	double error = 1.0;
	double pre_error = error;
	int count = 0;
	std::cout << "start loop ..." << std::endl;

	selectedNodesPoseForce.clear();

	while (error > errorthreshold)
	{
		std::vector<int> poseIndex;
		poseIdGenerator(numRandomPose, numPoses, poseIndex);
		getPoseForce(partitionId, &b, poseIndex);

		std::vector<int> candidatePointLocalindex;
		std::vector<int> candidatePointGlobalindex;
		int selected;
		int size = partitionIndices[partitionId].size();
		size = 100;

		
		this->selectCandidatePoints(partitionId, &candidatePointLocalindex, &candidatePointGlobalindex, size);
		this->chooseMaxPointSubSet(partitionId, poseIndex, &candidatePointLocalindex, &candidatePointGlobalindex, r, selected);

		int localindex = candidatePointLocalindex[selected];
		int nodeid = candidatePointGlobalindex[selected];

		partitionIndices[partitionId].erase(partitionIndices[partitionId].begin() + localindex);
		partitionCubatureIndices[partitionId].push_back(nodeid);
		
		pushNodeForce(nodeid, partitionId);
		computeMatrixASubset(partitionId, partitionCubatureIndices[partitionId], &subA, poseIndex);

		NNLS<MatrixXd> nnls(subA);
		if (!nnls.solve(b))
		{
			std::cout << "failed" << std::endl;
		}

		weights = nnls.x();

		r = b - subA*weights;

		double suberror = r.norm() / b.norm();

		if (count % (R/3) == 0)
		{
			error = doFullsetTraining(partitionId, partitionCubatureIndices[partitionId], weights, fullb);
			std::cout << std::endl;
			std::cout << "reall error:" << error << std::endl;
		}

		count++;

		std::cout << "\r" << "error: " << suberror << " cubature count:" << count;

	}

	std::cout << "\r" << std::endl;

	for (int i = 0; i < partitionCubatureIndices[partitionId].size(); i++)
	{
		partitionCubatureWeights[partitionId].push_back(weights.data()[i]);
	}
}

void ReducedCubatureTrainingOptimization::getPoseForce(int partitionId, VectorXd*f, std::vector<int> poseIndex)
{
	int numPoses = poseIndex.size();
	int R = reducedCubatureDataSet->getPartitionNumDOFs(partitionId);
	int r = reducedCubatureDataSet->getR();

	std::vector<int> partionDOFs = cubaturePartition->getPartitionDomainIndices(partitionId);
	int DOFS_Per_Domain = reducedCubatureDataSet->getDOFsPerDomain();

	f->resize(numPoses*R, 1);

	for (int i = 0; i < numPoses; i++)
	{
		int poseid = poseIndex[i];

		VectorXd forcei(r);
		reducedCubatureDataSet->getInternalForce(poseid, partitionId, forcei.data());
		VectorXd subForcei(R);
		mapPartitionDOFs(forcei, subForcei, partionDOFs, DOFS_Per_Domain);
		f->block(i*R, 0, R, 1) = subForcei / subForcei.norm();
	}

}

void ReducedCubatureTrainingOptimization::chooseMaxPointSubSet(int partitionId, std::vector<int> poseIndex, std::vector<int>* candidatePointssubIndex, std::vector<int>* candidatePointsGlobalIndex, VectorXd residual, int &selectedindex)
{
	double maxvalue = -DBL_MAX;
	int bestPoint = -1;
	for (int i = 0; i < candidatePointsGlobalIndex->size(); i++)
	{
		VectorXd b;
		//int forceid = partitionIdOffset[partitionid] + i;
		//b = eleInternalForce[forceid];
		computeSubsetPoseInternalForce(candidatePointsGlobalIndex->at(i), partitionId, &b,poseIndex);

		double dif = (b.dot(residual)) / b.norm() / residual.norm();
		if (dif > maxvalue)
		{
			maxvalue = dif;
			bestPoint = i;
		}
	}
	selectedindex = bestPoint;
}

void ReducedCubatureTrainingOptimization::computeSubsetPoseInternalForceNormlized(int nodeid, int partitionId, VectorXd* b, std::vector<int> poseIndex)
{
	int numPose = poseIndex.size();
	int R = reducedCubatureDataSet->getPartitionNumDOFs(partitionId);

	std::vector<int> partionDOFs = cubaturePartition->getPartitionDomainIndices(partitionId);
	int r = reducedCubatureDataSet->getR();
	int DOFS_Per_Domain = reducedCubatureDataSet->getDOFsPerDomain();
	b->resize(numPose*R);

	for (int i = 0; i < numPose; i++)
	{
		int poseid = poseIndex[i];
		VectorXd displacement(r);
		reducedCubatureDataSet->getModelPose(poseid, displacement.data());
		VectorXd forcei(r);
		reducedCubatureDataSet->getInternalForce(poseid, partitionId, forcei.data());
		VectorXd subForcei(R);
		mapPartitionDOFs(forcei, subForcei, partionDOFs, DOFS_Per_Domain);

		VectorXd gforce(r);
		VectorXd subgforce(R);

		std::vector<int> nodeindice(1);
		nodeindice[0] = nodeid;
		reducedForceModel->getInternalForce(displacement.data(), gforce.data(), &nodeindice);
		mapPartitionDOFs(gforce, subgforce, partionDOFs, DOFS_Per_Domain);
		subgforce = subgforce / subForcei.norm();
		b->block(i*R, 0, R, 1) = subgforce;
	}
}

void ReducedCubatureTrainingOptimization::computeSubsetPoseInternalForce(int nodeid, int partitionId, VectorXd* b, std::vector<int> poseIndex)
{
	int numPose = poseIndex.size();
	int R = reducedCubatureDataSet->getPartitionNumDOFs(partitionId);

	std::vector<int> partionDOFs = cubaturePartition->getPartitionDomainIndices(partitionId);
	int r = reducedCubatureDataSet->getR();
	int DOFS_Per_Domain = reducedCubatureDataSet->getDOFsPerDomain();
	b->resize(numPose*R);

	for (int i = 0; i < numPose; i++)
	{
		int poseid = poseIndex[i];
		VectorXd displacement(r);
		reducedCubatureDataSet->getModelPose(poseid, displacement.data());

		VectorXd gforce(r);
		VectorXd subgforce(R);
		std::vector<int> nodeindice(1);
		nodeindice[0] = nodeid;
		reducedForceModel->getInternalForce(displacement.data(), gforce.data(), &nodeindice);
		mapPartitionDOFs(gforce, subgforce, partionDOFs, DOFS_Per_Domain);

		b->block(i*R, 0, R, 1) = subgforce;
	}
}

void ReducedCubatureTrainingOptimization::poseIdGenerator(int numRandomPoses, int totalPoses, std::vector<int>& resultIndex)
{
	srand(1);
	std::vector<int> original_index(totalPoses);
	std::iota(original_index.begin(), original_index.end(), 0);
	std::random_shuffle(original_index.begin(), original_index.end());
	
	resultIndex.resize(numRandomPoses);
	for (int i = 0; i < numRandomPoses; i++)
	{
		resultIndex[i] = original_index[i];
	}
}

void ReducedCubatureTrainingOptimization::computeMatrixASubSet(int partitionId, int newIndex, MatrixXd* subA, std::vector<int> poseIndex)
{
	MatrixXd tempA(subA->rows(), subA->cols() + 1);
	VectorXd b;
	computeSubsetPoseInternalForceNormlized(newIndex, partitionId, &b, poseIndex);
	tempA << *subA, b;
	(*subA) = tempA;
}

void ReducedCubatureTrainingOptimization::computeMatrixASubset(int partitionId, std::vector<int> selectedIndices, MatrixXd* subA, std::vector<int> poseIndex)
{
	int numPoses = poseIndex.size();
	int totalNumPoses = reducedCubatureDataSet->getNumPoses();
	int n = selectedIndices.size();
	int R = reducedCubatureDataSet->getPartitionNumDOFs(partitionId);
	std::vector<int> partionDOFs = cubaturePartition->getPartitionDomainIndices(partitionId);
	int r = reducedCubatureDataSet->getR();

	subA->resize(numPoses*R, n);
	subA->setZero();
	int DOFS_Per_Domain = reducedCubatureDataSet->getDOFsPerDomain();

	/* --------------------------
	A =
	g_1(1)        g_2(1)        ... g_n(1)
	.				.				.
	.				.				.
	.				.				.
	g_1(numPoses) g_2(numPoses) ... g_n(numPoses)

	-------------------------- */

	for (int i = 0; i < numPoses; i++)
	{
		int poseid = poseIndex[i];
		VectorXd displacement(r);
		reducedCubatureDataSet->getModelPose(poseid, displacement.data());
		VectorXd forcei(r);
		reducedCubatureDataSet->getInternalForce(poseid, partitionId, forcei.data());
		VectorXd subForcei(R);
		mapPartitionDOFs(forcei, subForcei, partionDOFs, DOFS_Per_Domain);
		double subForceiNorm = subForcei.norm();
		for (int j = 0; j < selectedIndices.size(); j++)
		{
			VectorXd gforce(r);
			VectorXd subgforce(R);
			subgforce = selectedNodesPoseForce[j*totalNumPoses + poseid];

			//int nodeid = selectedIndices[j];
			//std::vector<int> nodeindice(1);
			//nodeindice[0] = nodeid;

			//reducedForceModel->getInternalForce(displacement.data(), gforce.data(), &nodeindice);

			//mapPartitionDOFs(gforce, subgforce, partionDOFs, DOFS_Per_Domain);

			subA->block(i*R, j, R, 1) = subgforce / subForceiNorm;
		}
	}
}

double ReducedCubatureTrainingOptimization::doFullsetTraining(int partitionId, std::vector<int> selectedIndices, VectorXd& weights, VectorXd &b)
{
	MatrixXd A;
	int numPoses = reducedCubatureDataSet->getNumPoses();
	std::vector<int> fullposeIndex(numPoses);
	std::iota(fullposeIndex.begin(), fullposeIndex.end(), 0);
	this->computeMatrixASubset(partitionId, selectedIndices, &A, fullposeIndex);

	NNLS<MatrixXd> nnls(A);
	if (!nnls.solve(b))
	{
		std::cout << "failed" << std::endl;
	}
	weights = nnls.x();

	VectorXd r = b - A*weights;

	double error = r.norm() / b.norm();
	return error;
}

void ReducedCubatureTrainingOptimization::pushNodeForce(int newnodeid, int partitionId)
{
	int numPoses = reducedCubatureDataSet->getNumPoses();
	int R = reducedCubatureDataSet->getPartitionNumDOFs(partitionId);
	std::vector<int> partionDOFs = cubaturePartition->getPartitionDomainIndices(partitionId);
	int r = reducedCubatureDataSet->getR();
	int DOFS_Per_Domain = reducedCubatureDataSet->getDOFsPerDomain();

	std::vector<int> nodeindice(1);
	nodeindice[0] = newnodeid;
	for (int i = 0; i < numPoses; i++)
	{

		int poseid = i;
		VectorXd displacement(r);
		reducedCubatureDataSet->getModelPose(poseid, displacement.data());

		VectorXd gforce(r);
		VectorXd subgforce(R);
		reducedForceModel->getInternalForce(displacement.data(), gforce.data(), &nodeindice);
		mapPartitionDOFs(gforce, subgforce, partionDOFs, DOFS_Per_Domain);
		selectedNodesPoseForce.push_back(subgforce);
	}
}
