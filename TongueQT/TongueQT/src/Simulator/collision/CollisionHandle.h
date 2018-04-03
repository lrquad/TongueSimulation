#pragma once
#include <Eigen/Dense>
using namespace Eigen;

class CollisionDetector;
class LoboVolumetricMesh;
class LoboVolumetricMeshTriMeshAdapter;
class LoboIntegrator;

class CollisionHandle
{
public:
	CollisionHandle(CollisionDetector* collisionDetector, LoboVolumetricMesh* volumetricMesh, LoboVolumetricMeshTriMeshAdapter* volumetricTriMeshAdapter, LoboIntegrator* integrator);
	~CollisionHandle();

	virtual VectorXd computeCollisionForce(double *velocity = NULL);
	virtual void computeCollisionImpulseAndDis(double *velocity, double *difdisplacement, double* externalforce);

	double getSpringForceRatio() const { return springForceRatio; }
	void setSpringForceRatio(double val) { springForceRatio = val; }
	double getDampingRatio() const { return dampingRatio; }
	void setDampingRatio(double val) { dampingRatio = val; }
protected:

	virtual void init();

	CollisionDetector* collisionDetector;
	LoboVolumetricMesh* volumetricMesh;
	LoboVolumetricMeshTriMeshAdapter* volumetricTriMeshAdapter;
	LoboIntegrator* integrator;

	double springForceRatio;
	double dampingRatio;
	double* nodeVolume;
};

