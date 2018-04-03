#include "CollisionHandle.h"
#include "Simulator/collision/CollisionDetector.h"
#include "LoboVolumetricMesh/LoboVolumetricMesh.h"
#include "LoboVolumetricMesh/LoboVolumetricMeshTriMeshAdapter.h"
#include <iostream>

CollisionHandle::CollisionHandle(CollisionDetector* collisionDetector, LoboVolumetricMesh* volumetricMesh, LoboVolumetricMeshTriMeshAdapter* volumetricTriMeshAdapter, LoboIntegrator* integrator)
{
	this->collisionDetector = collisionDetector;
	this->volumetricMesh = volumetricMesh;
	this->volumetricTriMeshAdapter = volumetricTriMeshAdapter;
	this->integrator = integrator;
	
	init();
}

CollisionHandle::~CollisionHandle()
{
	free(nodeVolume);
}

Eigen::VectorXd CollisionHandle::computeCollisionForce(double *velocity)
{
	int numCollide = collisionDetector->getNumCollideInfoSize();
	int numVertex = volumetricMesh->getNumVertices();
	VectorXd externalforce(numVertex*3);
	externalforce.setZero();

	std::vector<bool> nodemark(numVertex);
	std::fill(nodemark.begin(), nodemark.end(), false);

	for (int i = 0; i < numCollide; i++)
	{
		CollideInfo info = collisionDetector->getCollideInfo(i);
		//use closed node
		int nodeid = volumetricTriMeshAdapter->getVolumeNodeByTriNodeId(info.nodeid);
		
		Vector3d force = -springForceRatio*info.displacement*info.norms;

		if (velocity != NULL)
		{
			/*	Vector3d v;
				v.data()[0] = velocity[nodeid * 3 + 0];
				v.data()[1] = velocity[nodeid * 3 + 1];
				v.data()[2] = velocity[nodeid * 3 + 2];
				force -= dampingRatio*v;*/
			nodemark[nodeid] = true;
		}

		externalforce.data()[nodeid * 3 + 0] += force.data()[0];
		externalforce.data()[nodeid * 3 + 1] += force.data()[1];
		externalforce.data()[nodeid * 3 + 2] += force.data()[2];
	}

	//friction force
		for (int i = 0; i < numCollide; i++)
		{
			CollideInfo info = collisionDetector->getCollideInfo(i);
			int nodeid = volumetricTriMeshAdapter->getVolumeNodeByTriNodeId(info.nodeid);
			if (info.collideTri == 5)
			{
				Vector3d v;
				v.data()[0] = velocity[nodeid * 3 + 0];
				v.data()[1] = velocity[nodeid * 3 + 1];
				v.data()[2] = velocity[nodeid * 3 + 2];

				double density = volumetricMesh->getNodeMaterial(nodeid)->getDensity();
				double mass = density *nodeVolume[nodeid];
				double temp = dampingRatio * mass*springForceRatio;
				temp = std::sqrt(temp);

				Vector3d friction = v - (v.dot(info.norms))*info.norms;
				friction *= -temp;
				externalforce.data()[nodeid * 3 + 0] += friction.data()[0];
				externalforce.data()[nodeid * 3 + 1] += friction.data()[1];
				externalforce.data()[nodeid * 3 + 2] += friction.data()[2];
			}
		}

	VectorXd targetV = collisionDetector->targetvolumevelocity;
	for (int i = 0; i < numVertex; i++)
	{
		if (nodemark[i])
		{
			Vector3d v;
			v.data()[0] = velocity[i * 3 + 0];
			v.data()[1] = velocity[i * 3 + 1];
			v.data()[2] = velocity[i * 3 + 2];
			Vector3d targetv = collisionDetector->targetvelcoity;
			if (targetV.size() == numVertex * 3)
			{
				targetv.data()[0] = targetV.data()[i * 3 + 0];
				targetv.data()[1] = targetV.data()[i * 3 + 1];
				targetv.data()[2] = targetV.data()[i * 3 + 2];
			}

			v -= targetv;
			double density = volumetricMesh->getNodeMaterial(i)->getDensity();
			double mass = density *nodeVolume[i];
			double temp = dampingRatio * mass*springForceRatio;
			temp = std::sqrt(temp);

			Vector3d force;
			force = -temp*v;

			externalforce.data()[i * 3 + 0] += force.data()[0];
			externalforce.data()[i * 3 + 1] += force.data()[1];
			externalforce.data()[i * 3 + 2] += force.data()[2];
		}
	}

	return externalforce;
}

void CollisionHandle::computeCollisionImpulseAndDis(double *velocity, double *difdisplacement, double* externalforce)
{
	
	int numCollide = collisionDetector->getNumCollideInfoSize();
	int numVertex = volumetricMesh->getNumVertices();
	memset(externalforce, 0, sizeof(double)*numVertex * 3);

	for (int i = 0; i < numCollide; i++)
	{
		CollideInfo info = collisionDetector->getCollideInfo(i);
		//use closed node
		int nodeid = volumetricTriMeshAdapter->getVolumeNodeByTriNodeId(info.nodeid);

		double density = volumetricMesh->getNodeMaterial(nodeid)->getDensity();

		Vector3d v;
		v.data()[0] = velocity[nodeid * 3 + 0];
		v.data()[1] = velocity[nodeid * 3 + 1];
		v.data()[2] = velocity[nodeid * 3 + 2];

		Vector3d targetV;
		targetV.setZero();
		targetV.data()[2] = -0.002 * 650;
		v -= targetV;

		double vn = v.dot(info.norms);

		double j = -(1 + dampingRatio)*nodeVolume[nodeid] * density*vn;
		Vector3d forceJ = j*info.norms*650;

		externalforce[nodeid * 3 + 0] += forceJ.data()[0];
		externalforce[nodeid * 3 + 1] += forceJ.data()[1];
		externalforce[nodeid * 3 + 2] += forceJ.data()[2];
	}
}

void CollisionHandle::init()
{
	setDampingRatio(10.0);
	springForceRatio = 1000.0;
	nodeVolume = (double*)malloc(sizeof(double)*volumetricMesh->getNumVertices());
	volumetricMesh->computeNodeVolume(nodeVolume);
}
