#include "ReducedCubatureTraining.h"
#include "LoboVolumetricMesh/LoboVolumetricMesh.h"
#include "cubature/CubaturePartition.h"
#include "cubature/CubatureDataSet.h"
#include "Simulator/ReducedForceModel/ReducedForceModel.h"
#include "cubature/ReducedCubatureDataSet.h"
#include <iostream>
#include "nnls.h"
#include <fstream>

#include <time.h>       /* time */

ReducedCubatureTraining::ReducedCubatureTraining(ReducedForceModel* reducedForceModel, LoboVolumetricMesh* volumetricmesh, ReducedCubatureDataSet* reducedcubaturedateset, CubaturePartition * cubaturepartition) :CubatureTraining(reducedcubaturedateset, cubaturepartition)
{
	this->reducedForceModel = reducedForceModel;
	this->volumetricMesh = volumetricmesh;
	this->reducedCubatureDataSet = reducedcubaturedateset;
}


ReducedCubatureTraining::ReducedCubatureTraining(const char* filename) :CubatureTraining(filename)
{
	
}

ReducedCubatureTraining::~ReducedCubatureTraining()
{

}

void ReducedCubatureTraining::trainingData()
{

	//initVectorG();
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
	//releaseVectorG();

}

void ReducedCubatureTraining::initVectorG()
{
	std::cout << "precompute internalforce" << std::endl;
	int numParitions = cubaturePartition->getNumPartition();
	eleInternalForce.clear();
	partitionIdOffset.clear();
	int offset = 0;
	for (int i = 0; i < numParitions; i++)
	{
		std::vector<int> partitionEle = cubaturePartition->getpartitionEleIndices(i);
		partitionIdOffset.push_back(offset);
		
		for (int j = 0; j < partitionEle.size(); j++)
		{
			VectorXd b;
			int eleid = partitionEle[j];
			computeAllPoseInternalForce(eleid, i, &b);
			
			eleInternalForce.push_back(b);
			std::cout<<"\r"<< offset + j;
		}
		offset += partitionEle.size();
	}
	std::cout << "finished." << std::endl;
}

void ReducedCubatureTraining::releaseVectorG()
{
	eleInternalForce.clear();
	partitionIdOffset.clear();

}

void ReducedCubatureTraining::trainingPartition(int partitionId, double errorthreshold)
{
	int numPoses = reducedCubatureDataSet->getNumPoses();
	int R = reducedCubatureDataSet->getPartitionNumDOFs(partitionId);
	VectorXd b;
	computeForceB(partitionId, &b);
	VectorXd r = b;

	MatrixXd A(numPoses*R,0);
	VectorXd weights;

	double error = r.norm()/b.norm();
	double pre_error = error;
	int repeatcount = 0;
	while (error > errorthreshold)
	{
		//add new node
		std::vector<int> candidatePointLocalindex;
		std::vector<int> candidatePointGlobalindex;
		int selected;
		int size = partitionIndices[partitionId].size();
		//size = std::log2(size)*2;
		size = 100;
		this->selectCandidatePoints(partitionId, &candidatePointLocalindex, &candidatePointGlobalindex, size);
	
		this->chooseMaxPoint(partitionId, &candidatePointLocalindex, &candidatePointGlobalindex, r, selected);

		int localindex = candidatePointLocalindex[selected];
		int nodeid = candidatePointGlobalindex[selected];
	
		partitionIndices[partitionId].erase(partitionIndices[partitionId].begin() + localindex);
		partitionCubatureIndices[partitionId].push_back(nodeid);
	
		this->extendMatrixA(partitionId, nodeid, &A);
		//computes

		NNLS<MatrixXd> nnls(A);
		if (!nnls.solve(b))
		{
			std::cout << "failed" << std::endl;
		}
		
		weights = nnls.x();

		r = b - A*weights;

		error = r.norm() / b.norm();

		repeatcount++;
		std::cout << "\r" << "error: " << error << "repeat_count:" << repeatcount;

	}
	std::cout << "\r" << std::endl;

	for (int i = 0; i < partitionCubatureIndices[partitionId].size(); i++)
	{
		partitionCubatureWeights[partitionId].push_back(weights.data()[i]);
	}

}

void ReducedCubatureTraining::computeForceB(int partitionId, VectorXd* b)
{
	int numPoses = reducedCubatureDataSet->getNumPoses();
	int R = reducedCubatureDataSet->getPartitionNumDOFs(partitionId);
	int r = reducedCubatureDataSet->getR();
	std::vector<int> partionDOFs = cubaturePartition->getPartitionDomainIndices(partitionId);
	int DOFS_Per_Domain = reducedCubatureDataSet->getDOFsPerDomain();
	
	b->resize(numPoses*R, 1);
	
	for (int i = 0; i < numPoses; i++)
	{
		VectorXd displacement(r);
		reducedCubatureDataSet->getModelPose(i, displacement.data());
		VectorXd forcei(r);
		reducedCubatureDataSet->getInternalForce(i, partitionId, forcei.data());
		VectorXd subForcei(R);
		mapPartitionDOFs(forcei, subForcei, partionDOFs, DOFS_Per_Domain);

		b->block(i*R, 0, R, 1) = subForcei/subForcei.norm();
	}
}

void ReducedCubatureTraining::computeMatrixA(int partitionId, std::vector<int> selectedIndices, MatrixXd *A)
{
	int numPoses = reducedCubatureDataSet->getNumPoses();
	int n = selectedIndices.size();
	int R = reducedCubatureDataSet->getPartitionNumDOFs(partitionId);
	std::vector<int> partionDOFs = cubaturePartition->getPartitionDomainIndices(partitionId);
	int r = reducedCubatureDataSet->getR();
	A->resize(numPoses*R, n);
	A->setZero();
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
		VectorXd displacement(r);
		reducedCubatureDataSet->getModelPose(i, displacement.data());
		VectorXd forcei(r);
		reducedCubatureDataSet->getInternalForce(i, partitionId, forcei.data());
		VectorXd subForcei(R);
		mapPartitionDOFs(forcei, subForcei, partionDOFs, DOFS_Per_Domain);

		for (int j = 0; j < selectedIndices.size(); j++)
		{
			VectorXd gforce(r);
			VectorXd subgforce(R);
			
			int nodeid = selectedIndices[j];
			std::vector<int> nodeindice(1);
			nodeindice[0] = nodeid;
			reducedForceModel->getInternalForce(displacement.data(), gforce.data(),&nodeindice);
			mapPartitionDOFs(gforce, subgforce, partionDOFs, DOFS_Per_Domain);
			A->block(i*R, j, R, 1) = subgforce / subForcei.norm();
		}
	}

}

void ReducedCubatureTraining::selectCandidatePoints(int partitionid, std::vector<int>* candidatePointssubIndex, std::vector<int>* candidatePointsGlobalIndex, int candidatesize)
{
	std::vector<int> temp(partitionIndices[partitionid].size());
	for (int i = 0; i < temp.size(); i++)
	{
		temp[i] = i;
	}
	//srand(time(NULL));
	srand(111);
	std::random_shuffle(temp.begin(), temp.end());
	if (candidatesize > temp.size())
	{
		candidatesize = temp.size();
	}

	for (int i = 0; i < candidatesize; i++)
	{
		candidatePointssubIndex->push_back(temp[i]);
		candidatePointsGlobalIndex->push_back(partitionIndices[partitionid][temp[i]]);
	}
}

void ReducedCubatureTraining::chooseMaxPoint(int partitionid, std::vector<int>* candidatePointssubIndex, std::vector<int>* candidatePointsGlobalIndex, VectorXd residual, int &selectedindex)
{
	double maxvalue = -DBL_MAX;
	int bestPoint = -1;
	for (int i = 0; i < candidatePointsGlobalIndex->size(); i++)
	{
		VectorXd b;
		//int forceid = partitionIdOffset[partitionid] + i;
		//b = eleInternalForce[forceid];
		computeAllPoseInternalForce(candidatePointsGlobalIndex->at(i), partitionid, &b);
		double dif = (b.dot(residual)) / b.norm() / residual.norm();
		if (dif > maxvalue)
		{
			maxvalue = dif;
			bestPoint = i;
		}
	}
	selectedindex = bestPoint;
}

void ReducedCubatureTraining::computeAllPoseInternalForce(int nodeid, int partitionId, VectorXd *b)
{
	int numPoses = reducedCubatureDataSet->getNumPoses();
	int R = reducedCubatureDataSet->getPartitionNumDOFs(partitionId);
	
	std::vector<int> partionDOFs = cubaturePartition->getPartitionDomainIndices(partitionId);
	
	int r = reducedCubatureDataSet->getR();
	int DOFS_Per_Domain = reducedCubatureDataSet->getDOFsPerDomain();

	b->resize(numPoses*R);

	for (int i = 0; i < numPoses; i++)
	{
		VectorXd displacement(r);
		reducedCubatureDataSet->getModelPose(i, displacement.data());

		VectorXd gforce(r);
		VectorXd subgforce(R);

		std::vector<int> nodeindice(1);
		nodeindice[0] = nodeid;

		reducedForceModel->getInternalForce(displacement.data(), gforce.data(), &nodeindice);
		mapPartitionDOFs(gforce, subgforce, partionDOFs, DOFS_Per_Domain);

		b->block(i*R, 0, R, 1) = subgforce;
	}



}

void ReducedCubatureTraining::extendMatrixA(int partitionId, int newIndex, MatrixXd* A)
{
	int numPoses = reducedCubatureDataSet->getNumPoses();
	int R = reducedCubatureDataSet->getPartitionNumDOFs(partitionId);
	std::vector<int> partionDOFs = cubaturePartition->getPartitionDomainIndices(partitionId);
	int r = reducedCubatureDataSet->getR();
	int DOFS_Per_Domain = reducedCubatureDataSet->getDOFsPerDomain();


	MatrixXd tempA(A->rows(), A->cols()+1);
	if (A->cols() != 0)
		tempA.block(0, 0, A->rows(), A->cols()) = *A;
	else
		tempA.setZero();

	for (int i = 0; i < numPoses; i++)
	{
		VectorXd displacement(r);
		reducedCubatureDataSet->getModelPose(i, displacement.data());
		VectorXd forcei(r);
		reducedCubatureDataSet->getInternalForce(i, partitionId, forcei.data());
		VectorXd subForcei(R);
		mapPartitionDOFs(forcei, subForcei, partionDOFs, DOFS_Per_Domain);

		VectorXd gforce(r);
		VectorXd subgforce(R);

		std::vector<int> nodeindice(1);
		nodeindice[0] = newIndex;
		reducedForceModel->getInternalForce(displacement.data(), gforce.data(), &nodeindice);

		mapPartitionDOFs(gforce, subgforce, partionDOFs, DOFS_Per_Domain);
		subgforce = subgforce / subForcei.norm();

		if (A->cols() == 0)
		{
			tempA.block(i*R, 0, R, 1) = subgforce.block(0,0,R,1);
		}
		else
		{
			tempA.block(i*R, tempA.cols()-1 , R, 1) = subgforce.block(0, 0, R, 1);
		}
	}

	(*A) = tempA;


}

void ReducedCubatureTraining::mapPartitionDOFs(VectorXd DOFs, VectorXd &subDOFs, std::vector<int> relatedDomain, int DOFS_Per_Domain)
{
	int R = relatedDomain.size()*DOFS_Per_Domain;
	assert(subDOFs.rows() == R);
	int offset = 0;
	for (int i = 0; i < relatedDomain.size(); i++)
	{
		int domainid = relatedDomain[i];
		for (int j = 0; j < DOFS_Per_Domain; j++)
		{
			subDOFs.data()[offset] = DOFs.data()[domainid*DOFS_Per_Domain + j];
			offset++;
		}
	}
}
