#pragma once
#include "Simulator/simulatorBase/LoboSimulatorBase.h"

class ReducedSTVKModelCubature;
class STVKForceModel;
class STVKModel;
class ReducedForceModel;
class SubspaceModalDerivativesModule;

class FloatingMDSTVKSimulator: public LoboSimulatorBase
{

public:
	FloatingMDSTVKSimulator(std::ifstream &readconfig, LoboTriMesh* obj_mesh, bool ifreadconfig);
	~FloatingMDSTVKSimulator();

	//preset all pointer to NULL
	void nullAllPointer();

	//call this function in destructor 
	void deleteAllPointer();

	virtual void initSimulator(int verbose = 0);

	//simulation
	virtual void updateSimulator(int verbose = 0);
	virtual void resetSimulator(int verbose = 0);

	//return current simulator type
	virtual LoboSimulatorBase::simulatorType getType();

	virtual void saveSimulator(const char* filename, fileFormatType formattype) const;
	virtual void readSimulator(const char* filename, fileFormatType formattype);

	virtual void readVolumetricMeshAscii(const char* filenamebase);

	virtual void initSimulatorGlobalConstraints();

	//subspace module control
	virtual void initSubspaceModes();
	virtual void generateModes();

	/* =============================
	mouse event
	=============================*/
	virtual void singleClickEvent();
	virtual void rectSelectEvent();
	virtual void mouseMoveEvent();
	virtual void mouseReleaseEvent();


	/* =============================
	get/set
	=============================*/

	bool getIsInvertible() const { return isInvertible; }
	void setIsInvertible(bool val) { isInvertible = val; }
	SubspaceModalDerivativesModule* getSubspaceModuleRef() const { return subspaceModule; }
	void setSubspaceModuleRef(SubspaceModalDerivativesModule* val) { subspaceModule = val; }
protected:

	virtual void initSparseMassMatrix();
	virtual void initIntegrator();

	virtual void readConfig(std::ifstream &instream);
	void mergeExternalForce();

	SparseMatrix<double>* massMatrix;
	STVKForceModel* stvkforce_model;
	STVKModel* stvkmodel;
	SubspaceModalDerivativesModule* subspaceModule;
	ReducedSTVKModelCubature* reducedSTVKModel;
	ReducedForceModel* reducedforcemodel;

	MatrixXd* reduced_massMatrix;

	std::string modesFileName;
	bool isInvertible;
};

FloatingMDSTVKSimulator* downCastReducedFloatingSTVKSimulator(LoboSimulatorBase* simulator);