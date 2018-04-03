#pragma once
#include "CollisionDetector.h"
#include <coldet.h>


class LoboTriMesh;
class btTriangleIndexVertexArray;
class btGImpactMeshShape;
class btCollisionObject;



class TriMeshCollisionDetector:public CollisionDetector
{
public:

	TriMeshCollisionDetector(LoboTriMesh* trimesh);
	~TriMeshCollisionDetector();

	virtual void initCollisionShape();

	virtual void updateCollisionShape();

	virtual void collideWith(CollisionDetector* objtB);


	Vector3d getTriNode(int nodeid);

	int getTriangleFaceNode(int faceid,int insideindex);

	CollisionModel3D* getColdet_model() const { return coldet_model; }

protected:

	void computeCollideInfo(int tri1, int tri2, TriMeshCollisionDetector* objB);

	void initBulletCollisionShape();
	void updateBulletCollisionShape();
	virtual void initCollisionModel3D();
	virtual void updateCollisionModel3D();

	virtual void initAABBTree();
	virtual void updateAABBTree();

	btTriangleIndexVertexArray* bt_tri_vertexarray;
	btGImpactMeshShape* bt_collision_shape;
	CollisionModel3D* coldet_model;
	
	VectorXf trimeshposition;
	VectorXi trimeshindices;


};

