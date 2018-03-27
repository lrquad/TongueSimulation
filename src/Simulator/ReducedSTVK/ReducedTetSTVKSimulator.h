#pragma once
#include "Simulator/simulatorBase/LoboSimulatorBase.h"

class ReducedSTVKModel;
class STVKForceModel;
class STVKModel;
class ReducedForceModel;

class ReducedTetSTVKSimulator:public LoboSimulatorBase
{
public:
	ReducedTetSTVKSimulator(std::ifstream &readconfig, LoboTriMesh* obj_mesh);
	ReducedTetSTVKSimulator(LoboVolumetricMesh* volumetricmesh, std::ifstream & readconfig, LoboTriMesh* obj_mesh);
	~ReducedTetSTVKSimulator();

	/**
	 * @brief init Simulator model;
	 */
	 virtual void initSimulatorModel();
	 virtual void computeModesCoeff();
	 virtual void readModeCoeff(const char* filename);
	 virtual void saveModeCoeff(const char* filename); 

	/**
	 * @brief init simulator after start;
	 * @param[in,out] verbose
	 */

	 virtual void initSimulator(int verbose = 0);

	void initSubspaceModes();

	void computeSparseMassMatrix();

	void saveSubspaceModes(const char* filename);
	void readSubspaceModes(const char* filename);
	void saveSubspaceModesAscii(const char* filename);

	virtual void updateSimulator(int verbose = 0);
	virtual void resetSimulator(int verbose = 0);

	virtual LoboSimulatorBase::simulatorType getType();

	virtual void saveSimulator(const char* filename, fileFormatType formattype) const;

	virtual void readSimulator(const char* filename, fileFormatType formattype);

	virtual void readVolumetricMeshAscii(const char* filenamebase);

	virtual void initSimulatorGlobalConstraints();

	virtual void insertCurrentPose();

	//external force save
	virtual void recordExternalForce();
	virtual void computeFromExternalForce();

	/* =============================
	mouse event
	=============================*/
	virtual void rectSelectEvent();
	virtual void singleClickEvent();
	virtual void mouseMoveEvent();

	//modes control
	int getNumModes(){ return r;}
	
	/**
	 * @return number of LMA 
	 */
	 int getNumLMAModesk(){ return modesk; }

	/**
	 * @return number of total modes
	 */
	 int getNumModesr(){ return modesr; }

	 int setShowModes(int modesid);
	 void scaleShowedModes(int modesid, double scale);

	 void setNumLMAModesk(int k){ modesk = k; }
	 void setNumModesr(int r){ modesr = r; }

	 bool getSavePoseSequenceRealTime() const { return savePoseSequenceRealTime; }
	 void setSavePoseSequenceRealTime(bool val) { savePoseSequenceRealTime = val; }

	 int getSavePoseSkipFrame() const { return savePoseSkipFrame; }
	 void setSavePoseSkipFrame(int val) { savePoseSkipFrame = val; }

protected:

	void testcollisionforce();

	virtual void initPointerToNULL();
	virtual void deletePointers();

	virtual void initIntegrator();
	virtual void createIntegrator();
	
	/**
	 * @brief read config to follow the script to do some 
	 *
	 * At this version, this function will call compute mass matrix before end.
	 *
	 * @param[in,out] instream Put argument desc here
	 * @return Put return information here
	 */
	 virtual void readConfig(std::ifstream &instream);

	SparseMatrix<double>* massMatrix;
	MatrixXd * reducedMassMatrix;
	VectorXd fulldisplacement;

	STVKForceModel* stvkforce_model;

	ReducedSTVKModel* reducedSTVKmodel;
	STVKModel* stvkmodel;
	ReducedForceModel* reducedforcemodel;

	MatrixXd reduced_modes;
	
	int r;
	int modesr;
	int modesk;

	bool savePoseSequenceRealTime;
	int savePoseSkipFrame;

private:
	VectorXd externalForce;

};

ReducedTetSTVKSimulator* downCastReducedTetSTVKSimulator(LoboSimulatorBase* simulator);
