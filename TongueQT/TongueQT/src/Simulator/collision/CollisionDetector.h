#pragma once
#include <btBulletDynamicsCommon.h>
#include <Eigen/Dense>
#include <vector>
using namespace Eigen;
/*!
 * \class CollisionDetection
 *
 * \brief The result of collision detection, will send to a collision handling object.
 *
 * \date September 2016
 */

class LoboTriMesh;

struct CollideInfo
{
	int collideTri;
	int nodeid;
	double displacement;
	Vector3d norms;
};

class CollisionDetector
{
public:

	CollisionDetector(LoboTriMesh* trimesh);
	~CollisionDetector();

	virtual void initCollisionShape() = 0;
	virtual void updateCollisionShape() = 0;

	void addNewCollitionInfo(int triid, int nodeid, double displacemenet, Vector3d norms);
	void clearCollitionInfo();

	/**
	 * @brief collision detection and store result in detection info.
	 * @param[in,out] objtB Must have same object type as current object
	 */
	 virtual void collideWith(CollisionDetector* objtB) = 0;


	btCollisionObject* getBt_collision_object() const { return bt_collision_object; }
	void setBt_collision_object(btCollisionObject* val) { bt_collision_object = val; }

	btDbvt * getDynamicAABBTree() const { return dynamicAABBTree; }
	void setDynamicAABBTree(btDbvt * val) { dynamicAABBTree = val; }
	bool getIsstatic() const { return isstatic; }
	void setIsstatic(bool val) { isstatic = val; }

	int getNumCollideInfoSize(){ return collitionInfo.size(); };
	CollideInfo getCollideInfo(int id){ return collitionInfo[id]; };
	
	Vector3d targetvelcoity;
	VectorXd targetvolumevelocity;
	int index_;
	int tir_index_;

protected:

	btCollisionObject* bt_collision_object;
	LoboTriMesh* trimesh;
	btDbvt *dynamicAABBTree;

	std::vector<CollideInfo> collitionInfo;
	


	bool isstatic;

};

