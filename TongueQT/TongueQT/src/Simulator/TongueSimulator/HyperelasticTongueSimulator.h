#pragma once
#include "Simulator/simulatorBase/LoboSimulatorBase.h"
#include "TongueModel.h"



class IsotropicHyperelasticModel;
class IsotropicHyperlasticForceModel;
class LoboIsotropicMaterial;
class ReducedSubspaceModel;
class ReducedSubspaceForceModel;
class CubatureModel;
class DomainDecomposition;
class WaveTongueMode;

class HyperelasticTongueSimulator : public LoboSimulatorBase
{
public:
	HyperelasticTongueSimulator(std::ifstream &readconfig, LoboTriMesh* obj_mesh, bool ifreadconfig);
	~HyperelasticTongueSimulator();


	void nullAllPointer();
	void deleteAllPointer();

	virtual void initSimulator(int verbose = 0);
	virtual void updateSimulator(int verbose = 0);
	virtual void resetSimulator(int verbose = 0);

	virtual void insertAudioWaveIn(double *wave);

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


	bool getIsInvertible() const { return isInvertible; }
	void setIsInvertible(bool val) { isInvertible = val; }

	TongueModel* getTonguemodelRef(){ return &tonguemodel; }

protected:


	virtual void readModes(const char* filename);
	virtual void initIntegrator();
	virtual void readConfig(std::ifstream &instream);
	void mergeExternalForce();
	virtual void readSensorData(const char* filename);
	virtual void monitorNextStep();
	virtual void monitorNextStepV2();

	TongueModel tonguemodel;
	WaveTongueMode* waveToTongue;


	bool useCubature;
	bool isInvertible;

	MatrixXd phi;

	IsotropicHyperelasticModel* isotropicHyperelasticModel;

	LoboIsotropicMaterial* isotropicMaterial;

	SparseMatrix<double>* massMatrix;
	MatrixXd reducedMassMatrix;

	ReducedSubspaceModel* reducedSubspaceModel;

	ReducedSubspaceForceModel* forcemodel;

	CubatureModel* cubaturemodel;

	DomainDecomposition* domaindecomposition;

};

HyperelasticTongueSimulator* downCastHyperTongueSimulator(LoboSimulatorBase* simulator);
