#pragma once
#include "Reduced/SubspaceModule/SubspaceModuleBase.h"

class SubspaceModesCreator;
class STVKModel;
class STVKForceModel;

class SubspaceModalDerivativesModule:public SubspaceModuleBase
{
public:
	SubspaceModalDerivativesModule(
		STVKModel* stvkmodel_,
		STVKForceModel* stvkforce_model_,
		SparseMatrix<double>* massMatrix_,
		int numConstrainedDOFs_,
		int* constrainedDOFs_,
		int modesk_ = 10,
		int modesr_ = 20
		);

	SubspaceModalDerivativesModule();

	~SubspaceModalDerivativesModule();

	virtual void configModule(
		STVKModel* stvkmodel_,
		STVKForceModel* stvkforce_model_,
		SparseMatrix<double>* massMatrix_,
		int numConstrainedDOFs_,
		int* constrainedDOFs_
		);


	virtual void generateSubspace();


	int getModesk() const { return modesk; }
	void setModesk(int val) { modesk = val; }
	int getModesr() const { return modesr; }
	void setModesr(int val) { modesr = val; }
protected:


	STVKModel* stvkmode;
	STVKForceModel* stvkforce_model;
	SubspaceModesCreator* modesCreator;

	SparseMatrix<double>* massMatrix;
	int numConstrainedDOFs;
	int* constrainedDOFs;

	int modesk;
	int modesr;
};

