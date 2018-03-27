#pragma once
#include "DomainDecompositionFramebased.h"

class DomainDecompositionPartition;

class DomainDecompositionLocal :public DomainDecompositionFramebased
{
public:
	DomainDecompositionLocal(
		DomainDecompositionPartition* oriDomainDecomposition,
		LoboVolumetricMesh* volumetricmesh,LoboVolumetricMeshGraph* volumetricmeshgraph, std::vector<int> seeds_index, WeightFunctionLaplacian* weightfunction, bool createSubVolumetricMesh = false);

	~DomainDecompositionLocal();

	virtual void doDecomposition();
	virtual void createDomainConstraints();

	double getDomainRadius(int domainid);
	void setDomainRadius(int domainid, double val);

protected:

	void fillDomain();
	void searchDomainRegion();
	void searchNeighborDomainByNodes();

	DomainDecompositionPartition* oriDomainDecomposition;
	
	std::vector<double> domainRadius;

};