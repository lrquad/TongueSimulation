#pragma once
#include <vector>

class DomainDecomposition;
class LoboVolumetricMesh;

/*!
 * \class CubaturePartition
 *
 * \brief 
 *
 * \date August 2016
 */

class CubaturePartition
{
public:

	//************************************
	// Parameter: DomainDecomposition * domaindecompsition
	// Using domain decomposition result to creat cubature partition(patch) which share
	// same DOFs.
	//************************************
	CubaturePartition(DomainDecomposition* domaindecompsition, LoboVolumetricMesh* volumetricMesh);
	~CubaturePartition();

	virtual void init();

	virtual void initPartitionNodeIndices();
	virtual void initPartitionEleIndices();
	virtual void intiPartitionEleIndicesByNode();


	virtual void saveCubaturePartition(const char* filename);
	virtual void readCubaturePartition(const char* filename);

	/* ====================================
	partition info query
	=====================================  */

	virtual int getNumPartition();
	virtual std::vector<int> getPartitionIndices(int partitionId);
	virtual std::vector<int> getpartitionEleIndices(int partitionId);
	virtual std::vector<int> getPartitionDomainIndices(int partitionId);

	int getNumElements();

protected:

	virtual void generatePartitionDOFs();


	
	int numPartition;

	DomainDecomposition* domaindecompsition;
	LoboVolumetricMesh* volumetricMesh;
	
	std::vector<std::vector<int>> partitionIndices;
	std::vector<std::vector<int>> partitionEleIndices;
	std::vector<std::vector<int>> partitionDOFs;
	std::vector<int> partitionR;

	std::vector<std::vector<int>> partitionFlag;
	std::vector<int> vertexPartition;
	std::vector<int> elementPartition;

};

