#include "CollisionDetector.h"

CollisionDetector::CollisionDetector(LoboTriMesh* trimesh)
{
	this->trimesh = trimesh;
	setIsstatic(true);
	targetvelcoity.setZero();

}

CollisionDetector::~CollisionDetector()
{
}

void CollisionDetector::addNewCollitionInfo(int triid, int nodeid, double displacemenet, Vector3d norms)
{
	CollideInfo newcollideinfo = { triid, nodeid, displacemenet, norms };
	collitionInfo.push_back(newcollideinfo);
}

void CollisionDetector::clearCollitionInfo()
{
	collitionInfo.clear();
}
