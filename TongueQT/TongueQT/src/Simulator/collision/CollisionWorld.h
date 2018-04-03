#pragma once
#include <btBulletDynamicsCommon.h>
#include <vector>
#include <Eigen/Dense>
using namespace Eigen;

class CollisionDetector;
class CollisionHandle;

class CollisionWorld
{
public:
	CollisionWorld();
	~CollisionWorld();

	virtual void performCollisionDetection();


	void registerCollisionDetection(CollisionDetector* collisionobject);

protected:

	virtual void initCollisionWorld();
	
	btCollisionWorld* bt_collision_world;
	btCollisionConfiguration* bt_collision_configuration;
	btCollisionDispatcher* bt_dispatcher;
	btBroadphaseInterface* bt_broadphase;
	
	std::vector<CollisionDetector*> collisionObjectList;
};

