#pragma once
#include <vector>
#include <Eigen/Dense>
using namespace Eigen;

class LoboNodeBase;
class PointsCloud
{
public:
	PointsCloud();
	~PointsCloud();

	void readPointsBinary(const char* filepath, Vector3d translate);
	void readPointsBinaryFILE(const char* filepath);

	int getNumPoints(){ return numVertex; };
	int getNumSlice(){ return m_nSlice; };
	int getNodesPerSlice(){ return m_nNodesPerSlice; };

	void saveObj(const char* filename);

	Vector3d getPointOriPosition(int pointid);
	Vector3d getCurPosition(int pointid);
	int searchColseNode(Vector3d input);


	void pushPoint(Vector3d position);

	/**
	 * @brief Generate a list of points randomly in xy [-1~1] plane. set z = 0
	 * @param[in,out] targetNumVertexs Put argument desc here
	 * @return Put return information here
	 */
	 void randomGeneratePoints2D(int targetNumVertexs);

protected:

	int numVertex;
	unsigned short m_nType;
	unsigned short m_nSlice;
	unsigned short m_nNodesPerSlice;

	std::vector<LoboNodeBase*> node_list;

};

