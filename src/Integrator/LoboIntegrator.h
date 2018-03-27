#pragma once
#include <Eigen/Dense>
#include <Eigen/Sparse>
using namespace Eigen;
class LoboIntegrator
{
public:
	LoboIntegrator(int r, double timestep, double dampingMassCoef = 0.0, double dampingStiffnessCoef = 0.0);
	~LoboIntegrator();

	//set integrator to zero
	virtual void resetToRest();
	virtual void resetToRestNoClearSequence();
	virtual void saveState(const char* filename);
	virtual void readState(const char* filename);
	virtual void dampVelocity(double damp);

	virtual int doTimeStep()  = 0;
	virtual void computeStaticDisplacement(VectorXd &externalForce){};


	inline virtual void setTimeStep(double timestep){ this->timestep = timestep;}
	inline double getTimeStep(){ return timestep; }
	virtual void setToStep(int stepNumber){};

	virtual void setInternalForceScalingFactor(double internalforcescalingfactor){ internalForceScalingFactor = internalforcescalingfactor; }

	inline virtual void setDampingMassCoef(double dampingMassCoef){ this->dampingMassCoef = dampingMassCoef; }
	inline virtual void setDampingStiffnessCoef(double dampingStiffnessCoef){ this->dampingStiffnessCoef = dampingStiffnessCoef; }

	inline double GetDampingMassCoef(){ return dampingMassCoef; }
	inline double GetDampingStiffnessCoef(){ return dampingStiffnessCoef; }

	virtual int setState(double *q,double *qvel = NULL) =0;
	virtual void setqState(const double *q, const double * qvel=NULL, const double* qaccel = NULL);
	virtual void getqState(double *q, double *qvel=NULL,double*qaccel = NULL);

	inline virtual double * getq(){ return q.data();}
	inline virtual double * getq_vel(){ return q_vel.data();}
	inline virtual double * getq_accel(){ return q_accel.data();}

	inline virtual VectorXd getVectorq(){
		return q;
	}
	inline virtual VectorXd getVectorq_vel(){ return q_vel; }
	inline virtual VectorXd getVectorq_accel(){ return q_accel; }

	inline virtual void resetVelAndAceel();


	/* ====================================
	set external forces (a vector of r numbers)
	   ==================================== */

	void setExternalForces(double *externalForces);
	void addExternalForces(double *externalForces);
	void getExternalForces(double *externalForces);
	double* getExternalForces(){ return externalForces.data();}
	double getExternalForcesNorm(){ return externalForces.norm(); }
	void setExternalForcesToZero();

	/* ====================================
	misc
	==================================== */

	inline int getNumDOFs(){ return r; }
	VectorXd getQdelta(){ return qdelta; }

	void saveResults(const char* filename);
	void readResult(const char* filename);

	virtual void saveResultsBinary(const char* filename);
	virtual void readResultsBinary(const char* filename);

	bool getUseLoadedStepResults() const { return useLoadedStepResults; }
	void setUseLoadedStepResults(bool val) { useLoadedStepResults = val; }
	bool getRecordLoadedStepResults() const { return recordLoadedStepResults; }
	void setRecordLoadedStepResults(bool val);
	int getSkipSteps() const { return skipSteps; }
	void setSkipSteps(int val) { skipSteps = val; }


	std::vector<VectorXd>* getSequence_q_ref();

	bool getSaveStepResidual() const { return saveStepResidual; }
	void setSaveStepResidual(bool val) { saveStepResidual = val; }
	Eigen::VectorXd getStep_residual() const { return step_residual; }
	void setStep_residual(Eigen::VectorXd val) { step_residual = val; }
	bool getShowPerformance() const { return showPerformance; }
	void setShowPerformance(bool val) { showPerformance = val; }
protected:

	std::vector<VectorXd> sequence_q;
	std::vector<VectorXd> sequence_q_vel;
	std::vector<VectorXd> sequence_q_acc;

	std::string stepDisplacementFilePath;
	bool useLoadedStepResults;
	bool recordLoadedStepResults;
	bool saveStepResidual;



	VectorXd q;
	VectorXd q_vel;
	VectorXd q_accel;
	
	VectorXd q_residual, qdelta;

	VectorXd q_1;
	VectorXd q_vel_1;
	VectorXd q_accel_1;

	VectorXd internalForces;
	VectorXd externalForces;

	VectorXd step_residual;

	double dampingMassCoef;
	double dampingStiffnessCoef;
	double internalForceScalingFactor;

	int r; // number of reduced DOFs;

	double timestep;
	int step_;
	int skipSteps;

	bool showPerformance;
};

