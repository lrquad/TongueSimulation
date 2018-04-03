#pragma once
#include "ExternalForceGenerator.h"
class LoboVolumetricMesh;

class FunnelForceGenerator:public ExternalForceGenerator
{
public:
	FunnelForceGenerator(SparseMatrix<double>* massMatrix, LoboVolumetricMesh* volumetricmesh);
	~FunnelForceGenerator();

	virtual void readconfig(const char* filename);
	virtual void generateForce(int r, std::vector<VectorXd> &forces, MatrixXd* subspacePhi = NULL);

	double getNodeforce() const { return nodeforce_; }
	void setNodeforce(double val) { nodeforce_ = val; }
	int getPreFrames() const { return preFrames; }
	void setPreFrames(int val) { preFrames = val; }
	double getDecrese_ratio() const { return decrease_ratio; }
	void setDecrese_ratio(double val) { decrease_ratio = val; }
	int getFunnelkeyframe() const { return funnelkeyframe; }
	void setFunnelkeyframe(int val) { funnelkeyframe = val; }
	double getScale() const { return scale_; }
	void setScale(double val) { scale_ = val; }
protected:
	LoboVolumetricMesh* volumetricmesh;

	double nodeforce_;
	int preFrames;
	double decrease_ratio;
	double inscrease_ratio;
	double second_inscrease_ratio;
	int numFrames;
	int decreaseFrames;
	int secondPreFrames;
	int funnelkeyframe;
	double scale_;
};

