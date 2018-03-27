#pragma once
#include "DomainDecomposition.h"
class WeightFunctionLaplacian;

class DomainDecompositionFramebased :public DomainDecomposition
{
public:
	DomainDecompositionFramebased(LoboVolumetricMesh* volumetricmesh, std::vector<int> seeds_index, WeightFunctionLaplacian* weightfunction, bool createSubVolumetricMesh = false);
	DomainDecompositionFramebased(LoboVolumetricMesh* volumetricmesh, LoboVolumetricMeshGraph* volumetricmeshgraph, std::vector<int> seeds_index, WeightFunctionLaplacian* weightfunction, bool createSubVolumetricMesh = false);

	DomainDecompositionFramebased(LoboVolumetricMesh* volumetricmesh, LoboVolumetricMeshGraph* volumetricmeshgraph, WeightFunctionLaplacian* weightfunction, bool createSubVolumetricMesh = false);

	~DomainDecompositionFramebased();

	virtual void doDecomposition();
	virtual void createDomainConstraints();

	/* =============================
	query
	=============================*/
	double getNodeToFrameDistance(int nodeid, int frameid);
	double getEleToFrameDistance(int eleid, int frameid);
	
	std::vector<int> getDomainRegionBorder(int frameid);
	std::vector<int> getDomainInsideConstraints(int frameid);
	std::vector<int> getDomainConstrain(int frameid);
	std::vector<int> getDomainBorderEle(int frameid);
	std::vector<int> getDomainPartition(int frameid);
	std::vector<int> getDomainNeighbor(int frameid);

	int getNumRemovedDOFs(int domainid);
	int* getRemovedDOFs(int domainid);
	std::vector<int> getDomainMap(int domainid);

	void mergeConstraint(int domainid, std::vector<bool>& global_mark);

	std::vector<int> getPartition_nodemark() const { return partition_nodemark; }
	void setPartition_nodemark(std::vector<int> val) { partition_nodemark = val; }
protected:

	void computeDistanceToFrameSeed();
	void searchNeighborDomain();
	void searchNeighborDomainByNodes();
	void findMostDistanceNeighborseed();
	

	virtual void fillDomain();
	virtual void checkDomainConnection();
	virtual void ignoreUnConnectedNodes();
	virtual bool checkDomainConnection(int domainid);
	virtual void searchInsideBorder(int frameid, int neighborseed,std::vector<bool> &mark);
	virtual void searchBorderElement();

	virtual void generateRemoveConstrainInfo();


	bool constrainInfoCalled;

	MatrixXd distancetoFrame;
	MatrixXd elementDistancetoFrame;
	

	WeightFunctionLaplacian* weightfunction;
	
	std::vector<std::vector<int>> seed_neighbor_graph;
	std::vector<int> region_elemark;
	std::vector<int> partition_nodemark;
	std::vector<int> mostSeed;

	
	//record elements belongs to closet domain seed.
	std::vector<std::vector<int>> distance_region_info; 

	std::vector<std::vector<int>> region_border;
	std::vector<std::vector<int>> region_inside_constraint;
	std::vector<std::vector<int>> region_border_elements;
	
	//merge border as constraints.
	std::vector<std::vector<int>> region_all_constraints;

	std::vector<int> numDomainRemoveDofs;
	std::vector<int*> DomainRemoveDofs;

	std::vector<std::vector<int>> mapOldNewDomain;

};

