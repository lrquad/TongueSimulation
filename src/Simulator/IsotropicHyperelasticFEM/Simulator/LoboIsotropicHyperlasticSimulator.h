#pragma once
#include "Simulator/simulatorBase/LoboSimulatorBase.h"

class IsotropicHyperelasticModel;
class LoboneoHookeanIsotropicMaterial;
class IsotropicHyperlasticForceModel;
class LoboIsotropicMaterial;
class RotateForceField;

class LoboIsotropicHyperlasticSimulator : public LoboSimulatorBase
{
public:
	LoboIsotropicHyperlasticSimulator(std::ifstream &readconfig, LoboTriMesh* obj_mesh, bool ifreadconfig);
	~LoboIsotropicHyperlasticSimulator();

	typedef enum { NeoHookean = 0, STVK = 1} MaterialType;

	
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

	/* =============================
	mouse event
	=============================*/
	virtual void singleClickEvent();
	virtual void rectSelectEvent();
	virtual void mouseMoveEvent();
	virtual void mouseReleaseEvent();

	LoboIsotropicHyperlasticSimulator::MaterialType getMaterialtype() const { return materialtype; }
	void setMaterialtype(LoboIsotropicHyperlasticSimulator::MaterialType val) { materialtype = val; }

	bool getIsInvertible() const { return isInvertible; }
	void setIsInvertible(bool val) { isInvertible = val; }
protected:

	virtual void initIntegrator();
	virtual void readConfig(std::ifstream &instream);
	void mergeExternalForce();

	MaterialType materialtype;

	SparseMatrix<double>* massMatrix;

	bool isInvertible;

	IsotropicHyperelasticModel* isotropicHyperelasticModel;

	LoboIsotropicMaterial* isotropicMaterial;

	IsotropicHyperlasticForceModel* forcemodel;

	RotateForceField* rotateForceField;

	std::vector<double> nodetrajectory;

};

LoboIsotropicHyperlasticSimulator* downCastIsotropicHyperElasticSimulator(LoboSimulatorBase* simulator);