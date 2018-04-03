#include "TriMeshCollisionDetector.h"
#include <btBulletDynamicsCommon.h>
#include "LoboTriMesh/LoboTriMeshH.h"
#include <QElapsedTimer>
#include <iostream>
#include "Functions/computeTriangle.h"
#include <fstream>

TriMeshCollisionDetector::TriMeshCollisionDetector(LoboTriMesh* trimesh) :CollisionDetector(trimesh)
{
	coldet_model = NULL;

	initCollisionShape();
}

TriMeshCollisionDetector::~TriMeshCollisionDetector()
{
	delete bt_collision_object;
	delete bt_collision_shape;
	delete bt_tri_vertexarray;
	delete dynamicAABBTree;
	delete coldet_model;
}

void TriMeshCollisionDetector::initCollisionShape()
{
	trimeshposition = trimesh->getCurrentPositionFloat();
	trimeshindices = trimesh->getIndices();

	initBulletCollisionShape();

	QElapsedTimer timer;
	timer.start();
	initCollisionModel3D();
	std::cout << "AABB tree use time " << timer.elapsed() << "ms." << std::endl;
}

void TriMeshCollisionDetector::updateCollisionShape()
{
	VectorXf temp = trimesh->getCurrentPositionFloat();
	memcpy(trimeshposition.data(), temp.data(), sizeof(float)*trimeshposition.size());

	updateBulletCollisionShape();
	updateCollisionModel3D();
}

void TriMeshCollisionDetector::collideWith(CollisionDetector* objtB)
{
	TriMeshCollisionDetector* downcastB = (TriMeshCollisionDetector*)objtB;

	if (this->coldet_model->collision(downcastB->getColdet_model()))
	{
		std::vector<int> collidedTris1;
		std::vector<int> collidedTris2;
		coldet_model->getCollidingTriangleList(collidedTris1, collidedTris2);

		for (int i = 0; i < collidedTris1.size(); i++)
		{
			int triid1 = collidedTris1[i];
			int triid2 = collidedTris2[i];
			computeCollideInfo(triid1, triid2, downcastB);
		}

	}
	else
	{
		//std::cout << "not collide" << std::endl;
	}


}

Eigen::Vector3d TriMeshCollisionDetector::getTriNode(int nodeid)
{
	Vector3d node(trimeshposition.data()[nodeid*3+0],
		trimeshposition.data()[nodeid * 3 + 1],
		trimeshposition.data()[nodeid * 3 + 2]
		);
	return node;
}

int TriMeshCollisionDetector::getTriangleFaceNode(int faceid, int insideindex)
{
	return trimeshindices.data()[faceid * 3 + insideindex];
}

void TriMeshCollisionDetector::computeCollideInfo(int tri1, int tri2, TriMeshCollisionDetector* objB)
{
	int tri1_node[3];
	int tri2_node[3];

	 tri1_node[0] = this->getTriangleFaceNode(tri1, 0);
	 tri1_node[1] = this->getTriangleFaceNode(tri1, 1);
	 tri1_node[2] = this->getTriangleFaceNode(tri1, 2);

	 tri2_node[0] = objB->getTriangleFaceNode(tri2, 0);
	 tri2_node[1] = objB->getTriangleFaceNode(tri2, 1);
	 tri2_node[2] = objB->getTriangleFaceNode(tri2, 2);

	Vector3d tri1_nodePosition[3];
	Vector3d tri2_nodePosition[3];
	tri1_nodePosition[0] = this->getTriNode(tri1_node[0]);
	tri1_nodePosition[1] = this->getTriNode(tri1_node[1]);
	tri1_nodePosition[2] = this->getTriNode(tri1_node[2]);

	tri2_nodePosition[0] = objB->getTriNode(tri2_node[0]);
	tri2_nodePosition[1] = objB->getTriNode(tri2_node[1]);
	tri2_nodePosition[2] = objB->getTriNode(tri2_node[2]);

	Vector3d normal1, normal2;
	computeTriangleNorm(tri1_nodePosition[0], tri1_nodePosition[1], tri1_nodePosition[2], normal1);
	computeTriangleNorm(tri2_nodePosition[0], tri2_nodePosition[1], tri2_nodePosition[2], normal2);

	for (int i = 0; i < 3; i++)
	{
		double distance_Nodei_Trai2 = computeDistancePointToTriangle(tri2_nodePosition[0], tri2_nodePosition[1], tri2_nodePosition[2], normal2, tri1_nodePosition[i]);
		
		if (distance_Nodei_Trai2 < 0)
		{
			this->addNewCollitionInfo(tri1, tri1_node[i], distance_Nodei_Trai2, normal2);
		}

		double distance_Nodei_Trai1 = computeDistancePointToTriangle(tri1_nodePosition[0], tri1_nodePosition[1], tri1_nodePosition[2], normal2, tri2_nodePosition[i]);

		if (distance_Nodei_Trai1 < 0)
		{
			objB->addNewCollitionInfo(tri2, tri2_node[i], distance_Nodei_Trai1, normal1);
		}

	}

}

void TriMeshCollisionDetector::initBulletCollisionShape()
{
	bt_tri_vertexarray = new btTriangleIndexVertexArray(trimeshindices.size() / 3, trimeshindices.data(), 3 * sizeof(int), trimeshposition.size() / 3, trimeshposition.data(), 3 * sizeof(float));

	bt_collision_shape = new btGImpactMeshShape(bt_tri_vertexarray);

	bt_collision_shape->updateBound();
	bt_collision_object = new btCollisionObject();
	bt_collision_object->setCollisionShape(bt_collision_shape);
}



void TriMeshCollisionDetector::updateBulletCollisionShape()
{
	bt_collision_shape->postUpdate();
	bt_collision_shape->updateBound();
}

void TriMeshCollisionDetector::initCollisionModel3D()
{
	coldet_model = newCollisionModel3D(false);

	for (int i = 0; i < trimeshindices.size() / 3; i++)
	{
		float trianglepoints[3][3];
		int v1, v2, v3;
		v1 = trimeshindices.data()[i * 3 + 0];
		v2 = trimeshindices.data()[i * 3 + 1];
		v3 = trimeshindices.data()[i * 3 + 2];

		trianglepoints[0][0] = trimeshposition.data()[v1 * 3 + 0];
		trianglepoints[0][1] = trimeshposition.data()[v1 * 3 + 1];
		trianglepoints[0][2] = trimeshposition.data()[v1 * 3 + 2];
		trianglepoints[1][0] = trimeshposition.data()[v2 * 3];
		trianglepoints[1][1] = trimeshposition.data()[v2 * 3 + 1];
		trianglepoints[1][2] = trimeshposition.data()[v2 * 3 + 2];
		trianglepoints[2][0] = trimeshposition.data()[v3 * 3];
		trianglepoints[2][1] = trimeshposition.data()[v3 * 3 + 1];
		trianglepoints[2][2] = trimeshposition.data()[v3 * 3 + 2];
		coldet_model->addTriangle(&trianglepoints[0][0], &trianglepoints[1][0], &trianglepoints[2][0]);
	}

	coldet_model->finalize();
}

void TriMeshCollisionDetector::updateCollisionModel3D()
{
	if (coldet_model != NULL)
	{
		delete coldet_model;
	}

	coldet_model = newCollisionModel3D(false);

	for (int i = 0; i < trimeshindices.size() / 3; i++)
	{
		float trianglepoints[3][3];
		int v1, v2, v3;
		v1 = trimeshindices.data()[i * 3 + 0];
		v2 = trimeshindices.data()[i * 3 + 1];
		v3 = trimeshindices.data()[i * 3 + 2];

		trianglepoints[0][0] = trimeshposition.data()[v1 * 3 + 0];
		trianglepoints[0][1] = trimeshposition.data()[v1 * 3 + 1];
		trianglepoints[0][2] = trimeshposition.data()[v1 * 3 + 2];
		trianglepoints[1][0] = trimeshposition.data()[v2 * 3];
		trianglepoints[1][1] = trimeshposition.data()[v2 * 3 + 1];
		trianglepoints[1][2] = trimeshposition.data()[v2 * 3 + 2];
		trianglepoints[2][0] = trimeshposition.data()[v3 * 3];
		trianglepoints[2][1] = trimeshposition.data()[v3 * 3 + 1];
		trianglepoints[2][2] = trimeshposition.data()[v3 * 3 + 2];
		coldet_model->addTriangle(&trianglepoints[0][0], &trianglepoints[1][0], &trianglepoints[2][0]);
	}
	coldet_model->finalize();

}

void TriMeshCollisionDetector::initAABBTree()
{
	dynamicAABBTree = new btDbvt();
	btDbvtVolume volume;
	VectorXd trimeshPosition = trimesh->getCurrentPositionDouble();
	VectorXi triIndices = trimesh->getIndices();

	for (int i = 0; i < triIndices.size() / 3; i ++)
	{
		btVector3 trianglepoints[3];
		int v1, v2, v3;
		v1 = triIndices.data()[i * 3 + 0];
		v2 = triIndices.data()[i * 3 + 1];
		v3 = triIndices.data()[i * 3 + 2];

		trianglepoints[0][0] = trimeshPosition.data()[v1 * 3 + 0];
		trianglepoints[0][1] = trimeshPosition.data()[v1 * 3 + 1];
		trianglepoints[0][2] = trimeshPosition.data()[v1 * 3 + 2];
		trianglepoints[1][0] = trimeshPosition.data()[v2 * 3];
		trianglepoints[1][1] = trimeshPosition.data()[v2 * 3 + 1];
		trianglepoints[1][2] = trimeshPosition.data()[v2 * 3 + 2];
		trianglepoints[2][0] = trimeshPosition.data()[v3 * 3];
		trianglepoints[2][1] = trimeshPosition.data()[v3 * 3 + 1];
		trianglepoints[2][2] = trimeshPosition.data()[v3 * 3 + 2];
		volume.FromPoints(trianglepoints, 3);
		dynamicAABBTree->insert(volume, NULL);
	}
}

void TriMeshCollisionDetector::updateAABBTree()
{
	dynamicAABBTree->clear();
	btDbvtVolume volume;
	VectorXd trimeshPosition = trimesh->getCurrentPositionDouble();
	VectorXi triIndices = trimesh->getIndices();

	for (int i = 0; i < triIndices.size() / 3; i++)
	{
		btVector3 trianglepoints[3];
		int v1, v2, v3;
		v1 = triIndices.data()[i * 3 + 0];
		v2 = triIndices.data()[i * 3 + 1];
		v3 = triIndices.data()[i * 3 + 2];

		trianglepoints[0][0] = trimeshPosition.data()[v1 * 3 + 0];
		trianglepoints[0][1] = trimeshPosition.data()[v1 * 3 + 1];
		trianglepoints[0][2] = trimeshPosition.data()[v1 * 3 + 2];
		trianglepoints[1][0] = trimeshPosition.data()[v2 * 3];
		trianglepoints[1][1] = trimeshPosition.data()[v2 * 3 + 1];
		trianglepoints[1][2] = trimeshPosition.data()[v2 * 3 + 2];
		trianglepoints[2][0] = trimeshPosition.data()[v3 * 3];
		trianglepoints[2][1] = trimeshPosition.data()[v3 * 3 + 1];
		trianglepoints[2][2] = trimeshPosition.data()[v3 * 3 + 2];
		volume.FromPoints(trianglepoints, 3);
		dynamicAABBTree->insert(volume, NULL);
	}
}
