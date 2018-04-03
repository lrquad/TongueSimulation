#pragma once
#include "Simulator/simulatorBase/LoboSimulatorBase.h"

class LoboSimpleSimulator:public LoboSimulatorBase
{
public:
	LoboSimpleSimulator(std::ifstream &readconfig, LoboTriMesh* tri_mesh, bool ifreadconfig);
	~LoboSimpleSimulator();
	
	virtual void initSimulator(int verbose = 0) = 0;
	virtual void updateSimulator(int verbose = 0) = 0;
	virtual void resetSimulator(int verbose = 0) = 0;

	virtual void readVolumetricMeshAscii(const char* filenamebase);

	virtual LoboSimulatorBase::simulatorType getType() = 0;

	/* ===========================================
	simulation export
	=========================================== */
	virtual void saveSimulator(const char* filename, fileFormatType formattype) const = 0;

	virtual void readSimulator(const char* filename, fileFormatType formattype) = 0;
};

