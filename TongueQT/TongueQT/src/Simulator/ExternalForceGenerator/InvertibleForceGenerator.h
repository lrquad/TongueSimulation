#pragma once
#include "FunnelForceGenerator.h"
class InvertibleForceGenerator:public FunnelForceGenerator
{
public:
	InvertibleForceGenerator(SparseMatrix<double>* massMatrix, LoboVolumetricMesh* volumetricmesh, VectorXd fullgravity);
	~InvertibleForceGenerator();


	virtual void readconfig(const char* filename);
	virtual void generateForce(int r, std::vector<VectorXd> &forces, MatrixXd* subspacePhi = NULL);

	double getInternalForceScale();

	Eigen::VectorXd getGravity() const { return gravity; }
	void setGravity(Eigen::VectorXd val) { gravity = val; }
	int getRestoreforceframe() const { return restoreforceframe; }
	void setRestoreforceframe(int val) { restoreforceframe = val; }
	double getInternalforce_restor_step() const { return internalforce_restor_step; }
	void setInternalforce_restor_step(double val) { internalforce_restor_step = val; }
	VectorXd getStart_shape() const { return start_shape; }
	void setStart_shape(VectorXd val) { start_shape = val; }

protected:

	int restoreforceframe;
	VectorXd gravity;
	double internalforce_restor_step;
	double scale;
	int step;

	VectorXd start_shape;

};

