#pragma once
#include <Eigen/Dense>
#include <Eigen/Sparse>
using namespace Eigen;

class LoboVolumetricMesh;
class LoboVolumetricMeshGraph;

class DomainDecomposition
{
public:
	DomainDecomposition(LoboVolumetricMesh* volumetricmesh, std::vector<int> seeds_index, bool createSubVolumetricMesh = false);
	
	DomainDecomposition(LoboVolumetricMesh* volumetricmesh, LoboVolumetricMeshGraph* volumetricmeshgraph, std::vector<int> seeds_index, bool createSubVolumetricMesh = false);

	DomainDecomposition(LoboVolumetricMesh* volumetricmesh, LoboVolumetricMeshGraph* volumetricmeshgraph,  bool createSubVolumetricMesh = false);


	~DomainDecomposition();

	virtual void doDecomposition() = 0;

	virtual void getDomain(int domainid, std::vector<int> &elements);
	virtual LoboVolumetricMesh* getSubVolumetricMesh(int domainid);
	inline virtual LoboVolumetricMesh* getDomainVolumetricMesh(int domainid){ return domainVolumetricMesh[domainid]; };

	std::vector<int> getSeeds(){ return seeds_index; }
	void setSeeds(std::vector<int> seeds){ seeds_index = seeds; numDomains = seeds.size(); }

	std::vector<int> getDomainNodes(int frameid);
	std::vector<int> getDomainRegionEle(int frameid);
	std::vector<int> getNodeDomains(int nodeid);
	std::vector<int> getElementDomains(int eleid);

	virtual int getNumDomain(){ return numDomains; }

	int getNumVertex() const { return numVertex; }
	void setNumVertex(int val) { numVertex = val; }
protected:

	int numVertex;
	int numDomains;
	int numElements;

	std::vector<std::vector<int>> domainsElements;
	std::vector<std::vector<int>> elementDomains;
	std::vector<std::vector<int>> nodesDomains;
	std::vector<std::vector<int>> domainsNodes;

	std::vector<std::vector<int>> unvisitedElement;

	std::vector<int> seeds_index;
	LoboVolumetricMesh* volumetricmesh;
	LoboVolumetricMeshGraph* volumetricmesh_graph;
	std::vector<LoboVolumetricMesh*> domainVolumetricMesh;



	bool createSubVolumetricMesh;
};

