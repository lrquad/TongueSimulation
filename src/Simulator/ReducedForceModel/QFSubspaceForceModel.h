#pragma once
#include "FrameBasedForceModel.h"
class QuadraticFrameSubspaceModel;

class QFSubspaceForceModel : public FrameBasedForceModel
{
public:
	QFSubspaceForceModel(QuadraticFrameSubspaceModel* qfsubspacemodel);
	~QFSubspaceForceModel();
	
	virtual void getInternalForce(double* u, double* internalForce, std::vector<int>* ele_indices);

	virtual void getInternalForce(double* u, double* internalForce, std::vector<int>* ele_indices,std::vector<int>* node_indices);

protected:
	QuadraticFrameSubspaceModel* qfsubspacemodel;
};

