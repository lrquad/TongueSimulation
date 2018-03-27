#pragma once
#include "DomainDecompositionFramebased.h"

class DomainDecompositionKmeans :public DomainDecompositionFramebased
{
public:

	/**
	 * @brief cluster volumetric mesh by K-means method
	 *
	 * @param[in,out] volumetricmesh target volumetric mesh
	 * @param[in,out] volumetricmesh_graph pre computed volumetric mesh graph see also @LoboVolumetricMeshGraph
	 * @param[in,out] k num of clustering
	 * @return clustering results;
	 */
	DomainDecompositionKmeans(LoboVolumetricMesh* volumetricmesh, LoboVolumetricMeshGraph* volumetricmesh_graph, int numConstrainedDOFs, int* constraintedDOFS, int k);
	~DomainDecompositionKmeans();

	virtual void doDecomposition();
	double getDomainRadius(int domainid);
	int getK(){ return numClustring;}

protected:

	virtual void fillDomain();
	void updateDomainRadus();
	int updateClusterCenters();
	void initGuess();

	int numConstrainedDOFs;
	int* constraintedDOFS;

	int numClustring; //num of clustering

	std::vector<double> domainRadius;

};

