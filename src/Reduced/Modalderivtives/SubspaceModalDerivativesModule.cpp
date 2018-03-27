#include "SubspaceModalDerivativesModule.h"
#include "Reduced/SubspaceModesCreator.h"
#include "Simulator/ForceModel/STVKForceModel.h"
#include <iostream>
#include <fstream>

SubspaceModalDerivativesModule::SubspaceModalDerivativesModule(STVKModel* stvkmodel_, STVKForceModel* stvkforce_model_, SparseMatrix<double>* massMatrix_, int numConstrainedDOFs_, int* constrainedDOFs_, int modesk_, int modesr_)
{
	this->stvkmode = stvkmodel_;
	this->stvkforce_model = stvkforce_model_;
	this->massMatrix = massMatrix_;
	this->numConstrainedDOFs = numConstrainedDOFs_;
	this->constrainedDOFs = constrainedDOFs_;
	this->setModesk(modesk_);
	this->setModesr(modesr_);
	modesCreator = NULL;
}

SubspaceModalDerivativesModule::SubspaceModalDerivativesModule()
{
	this->stvkmode = NULL;
	this->stvkforce_model = NULL;
	this->massMatrix = NULL;
	this->numConstrainedDOFs = 0;
	this->constrainedDOFs = NULL;
	modesCreator = NULL;
	this->setModesk(10);
	this->setModesr(20);
}

SubspaceModalDerivativesModule::~SubspaceModalDerivativesModule()
{
	delete modesCreator;
}

void SubspaceModalDerivativesModule::configModule(STVKModel* stvkmodel_, STVKForceModel* stvkforce_model_, SparseMatrix<double>* massMatrix_, int numConstrainedDOFs_, int* constrainedDOFs_)
{
	this->stvkmode = stvkmodel_;
	this->stvkforce_model = stvkforce_model_;
	this->massMatrix = massMatrix_;
	this->numConstrainedDOFs = numConstrainedDOFs_;
	this->constrainedDOFs = constrainedDOFs_;
}

void SubspaceModalDerivativesModule::generateSubspace()
{
	if (modesCreator != NULL)
	{
		delete modesCreator;
	}

	
	modesCreator = new SubspaceModesCreator(stvkforce_model, massMatrix, numConstrainedDOFs, constrainedDOFs, modesk, modesr);

	*ori_phi = *modesCreator->getFinalModes();
	*phi = *ori_phi;
	*pre_phi = *phi;
	this->r = phi->cols();
	ori_r = r;
}
