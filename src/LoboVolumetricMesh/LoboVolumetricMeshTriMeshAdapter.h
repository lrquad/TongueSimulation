#pragma once
#include "LoboVolumetricMeshObjAdapter.h"
class LoboTriMesh;

class LoboVolumetricMeshTriMeshAdapter:public LoboVolumetricMeshObjAdapter
{
public:
	LoboVolumetricMeshTriMeshAdapter(LoboVolumetricMesh* volumetric_mesh, LoboTriMesh* triMesh);

	~LoboVolumetricMeshTriMeshAdapter();

	virtual void computeNodeBarycentricWeight();
	virtual void updateObjMesh();

	virtual int getVolumeNodeByTriNodeId(int trinodeid);
	
	virtual void setTriMeshVertexColorData1D(double* volumenodedata);
	virtual void convertVolumtricDataToTri1D(double* volumenodedata,double* trinodedata);
	virtual void convertVolumtricDataToTri1DNoInterpolate(double* volumenodedata, double* trinodedata);
	virtual void setTriMeshVertexColorData3D(double* volumenodedata);

	virtual void computeTriDisplacement(double* tetdisplacement, double* tridisplacement);
	virtual void computeTriDisplacementNoInterpolate(double* tetdisplacement, double* tridisplacement);

	virtual void saveInfoBinary(std::ofstream &outstream);
	virtual void readInfoBinary(std::ifstream &instream);

	virtual void exportVertexColorMesh1D(const char* filename, double* volumenodedata);

	virtual void setColorMap(Vector3d color1_,Vector3d color2_);

	virtual void markTriNodes(std::vector<int> elementMark, int coloroffset = 0);

	virtual void sendVertexDataToTriMesh(VectorXd &nodeDataInpude,int dimension);


protected:

	virtual void initTriMeshMap();

	Vector3d color1;
	Vector3d color2;

	std::vector<int> trinode_node_map;
	LoboTriMesh* triMesh;

};

