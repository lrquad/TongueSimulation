#pragma once
#include <Eigen/Dense>
#include <vector>

using namespace Eigen;
class CubeElement;

class CubeNodeMap
{
public:
	CubeNodeMap();
	~CubeNodeMap();

protected:


};

class CubeNodeMapType
{
public:	
	CubeNodeMapType(std::vector<int> neighbors);
	~CubeNodeMapType();

	virtual void computeSubTypeInfo() = 0;

public:
	
	int getSubtype() const { return subtype; }
	void setSubtype(int val) { subtype = val; }

	int getMainType() const { return mainType; }
	void setMainType(int val) { mainType = val; }

	std::vector<int> reorderNeighbor;

	Matrix3d R_;
	Matrix3d R_align_;

protected:
	virtual void computeNeighborOrderMap() = 0;

	std::vector<int> neighbors;
	
	int subtype;

	//mainType = 0 3 neighbor
	//mainType = 1 4 neighbor
	//mainType = 2 5 neighbor
	//mainType = 3 6 neighbor
	int mainType;
};

class CubeNodeThreeNeighborType : public CubeNodeMapType
{
public :
	CubeNodeThreeNeighborType(std::vector<int> neighbors);
	~CubeNodeThreeNeighborType();

	virtual void computeSubTypeInfo();

protected:
	
	virtual void computeNeighborOrderMap();
};

class CubeNodeFourNeighborType :public CubeNodeMapType
{
public :
	CubeNodeFourNeighborType(std::vector<int> neighbors);
	~CubeNodeFourNeighborType();

	virtual void computeSubTypeInfo();
protected:
	virtual void computeNeighborOrderMap();

	int directionType;
};

class CubeNodeFiveNeighborType : public CubeNodeMapType
{
public :
	CubeNodeFiveNeighborType(std::vector<int> neighbors);
	~CubeNodeFiveNeighborType();

	virtual void computeSubTypeInfo();
protected:
	virtual void computeNeighborOrderMap();

	int directionType;
};

class CubeNodeALLNeighborType :public CubeNodeMapType
{
public:
	CubeNodeALLNeighborType(std::vector<int> neighbors);
	~CubeNodeALLNeighborType();
	virtual void computeSubTypeInfo();
protected:
	virtual void computeNeighborOrderMap();
};
