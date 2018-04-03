#include "CollisionWorld.h"
#include "Simulator/collision/CollisionDetector.h"
#include "Simulator/collision/TriMeshCollisionDetector.h"
#include <iostream>

CollisionWorld::CollisionWorld()
{
	initCollisionWorld();
}


CollisionWorld::~CollisionWorld()
{
	delete bt_collision_world;
	delete bt_broadphase;
	delete bt_dispatcher;
	delete bt_collision_configuration;
}

void CollisionWorld::performCollisionDetection()
{
	bt_collision_world->performDiscreteCollisionDetection();

	//init collision info;
	int numManifolds = bt_collision_world->getDispatcher()->getNumManifolds();

	std::vector<Vector2i> collision_pair;
	for (int i = 0; i < numManifolds; i++) {
		btPersistentManifold* contactManifold = bt_collision_world->getDispatcher()->getManifoldByIndexInternal(i);
		int numContacts = contactManifold->getNumContacts();
		
		btCollisionObject* obA = (btCollisionObject*)(contactManifold->getBody0());
		btCollisionObject* obB = (btCollisionObject*)(contactManifold->getBody1());
		Vector2i pair;
		for (int j = 0; j < collisionObjectList.size(); j++)
		{
			if (collisionObjectList[j]->getBt_collision_object() == obA)
			{
				pair.data()[0] = j;
			}

			if (collisionObjectList[j]->getBt_collision_object() == obB)
			{
				pair.data()[1] = j;
			}
		}
		collision_pair.push_back(pair);
	}

	for (int i = 0; i < collisionObjectList.size(); i++)
	{
		collisionObjectList[i]->clearCollitionInfo();
	}
	
	if (collisionObjectList.size() >= 2)
	{
		collision_pair.clear();
		//collision_pair.push_back(Vector2i(0, 1));
		//if (collisionObjectList.size() >= 3)
		//{
		//	//collision_pair.push_back(Vector2i(0, 2));
		//	collision_pair.push_back(Vector2i(1, 2));
		//}

		for (int i = 0; i < collisionObjectList.size(); i++)
		{
			for (int j = i+1; j < collisionObjectList.size(); j++)
			{
				collision_pair.push_back(Vector2i(i, j));
			}
		}
		
	}

	for (int i = 0; i < collision_pair.size(); i++)
	{
		int meshA = collision_pair[i].data()[0];
		int meshB = collision_pair[i].data()[1];
		collisionObjectList[meshA]->collideWith(collisionObjectList[meshB]);
	}
}

void CollisionWorld::registerCollisionDetection(CollisionDetector* collisionobject)
{
	collisionObjectList.push_back(collisionobject);
	bt_collision_world->addCollisionObject(collisionobject->getBt_collision_object());
}

void CollisionWorld::initCollisionWorld()
{
	std::cout << "init collision world." << std::endl;
	double scene_size = 1000;
	unsigned int max_objects = 10000;
	bt_collision_configuration = new btDefaultCollisionConfiguration();

	bt_dispatcher = new btCollisionDispatcher(bt_collision_configuration);

	btGImpactCollisionAlgorithm::registerAlgorithm(bt_dispatcher);

	btScalar sscene_size = (btScalar)scene_size;
	btVector3 worldAabbMin(-sscene_size, -sscene_size, -sscene_size);
	btVector3 worldAabbMax(sscene_size, sscene_size, sscene_size);

	bt_broadphase = new bt32BitAxisSweep3(worldAabbMin, worldAabbMax, max_objects, 0, true);  // true for disabling ray cast accelerator

	bt_collision_world = new btCollisionWorld(bt_dispatcher, bt_broadphase, bt_collision_configuration);
}
