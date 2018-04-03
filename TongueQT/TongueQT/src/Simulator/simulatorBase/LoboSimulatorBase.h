#pragma once
#include "Render/LoboObjMesh.h"
#include "volumetricMesh.h"
#include "LoboVolumetricMesh/LoboVolumetricMesh.h"
#include "LoboVolumetricMesh/Graph/LoboVolumetricMeshGraph.h"
#include "LoboVolumetricMesh/LoboVolumetricMeshObjAdapter.h"
#include "LoboVolumetricMesh/LoboVolumetricMeshTriMeshAdapter.h"
#include "Integrator/LoboIntegrator.h"
#include "LoboTriMesh/LoboTriMeshH.h"

class MouseProjection;
class PoseDataSet;
class CollisionHandle;
class CollisionDetector;
class FunnelForceGenerator;
class ExternalForceGenerator;

/*!
 * \class LoboSimulatorBase
 *
 * \ingroup GroupName
 *
 * \brief 
 *
 * TODO: long description
 *
 * \note 
 *
 *
 * \version 1.0
 *
 * \date August 2016
 *
 * Contact: user@company.com
 *
 */
class LoboSimulatorBase
{
public:
	LoboSimulatorBase();

	LoboSimulatorBase(LoboVolumetricMesh* volumetricmesh, std::ifstream &readconfig, LoboTriMesh* tri_mesh);

	//new version
	LoboSimulatorBase(std::ifstream &readconfig, LoboTriMesh* tri_mesh);

	~LoboSimulatorBase();

	/* ===========================================
	simulation routine
	=========================================== */
	virtual void initConstructor();

	virtual void initSimulator(int verbose = 0) = 0;
	virtual void updateSimulator(int verbose = 0) = 0;
	virtual void resetSimulator(int verbose = 0) = 0;
	
	virtual void updateVolumetricMeshVolume(){};

	typedef enum { beforeinit, ready, start, pause } SimulationState;

	typedef enum { ASCII, BINARY, NUM_FILE_FORMATS } fileFormatType;
	typedef enum { LINEARFEM, REDUCEDSTVK, SUBSTRUCTURING, FRAMEBASED, QUADRATICFRAME, ADAPTIVEQUADRATICFRAME, AFFINEFRAME, MESHFREEMLS2D, TONGUESIMULATOR,MOVINGOBJ,FLOATINGMD,CONVEXHULLTEST,ISOTROPICHYPERLASTIC,RIGIDSIM,MULTIRIGIDSIM,MODALWARPING,HTONGUE,ENHANCEDREDUCE} simulatorType;

	void setSimulatorStart();
	void setSimulatorStop();

	virtual void initSimulatorGlobalConstraints(){};

	//force control
	virtual void resetExternalForce();
	
	//pose control
	virtual void insertCurrentPose();
	virtual void savePoseSet(const char* filename);
	virtual void readPoseSet(const char* filename);
	virtual void clearPoseSet();

	virtual void generateBendingForce();
	virtual void readExternalForce(const char* filename);
	virtual void recordExternalForce(){};
	virtual void saveExternalForce(const char* filename);
	virtual void computeFromExternalForce(){};

	//integrator pose control
	virtual int saveIntegratorResults(const char* filename);
	virtual int readIntegratorResults(const char* filename);
	virtual int setRecordIntegrator(bool b);

	//collision handle
	virtual void registerCollision(CollisionDetector* collisionDetector);
	virtual void registerTriMesh(LoboTriMesh* tri_mesh);


	/* ===========================================
	volumetric read /save
	=========================================== */
	virtual void readVolumetricMeshAscii(const char* filenamebase) = 0;
	virtual void saveSelectedVolumetricMeshAscii(const char* filenamebase);
	virtual void saveVolumtricMeshSurfaceTri(const char* filename);

	/* ===========================================
	Simulator access
	=========================================== */

	LoboVolumetricMesh* getVolumetricMesh();
	LoboObjMesh* getObjMesh();
	LoboTriMesh* getTriMesh();
	LoboIntegrator* getIntegrator();

	inline std::string getName(){ return simulatorname;};
	inline void setName(const char* name){ simulatorname = name;}

	virtual LoboSimulatorBase::simulatorType getType() = 0;

	SimulationState getSimulatorState(){ return simulationstate; }

	virtual void saveConstraints(const char* filename);
	virtual void saveConstraintsBinary(std::ofstream &outstream) const;
	virtual void readConstraintsBianry(std::ifstream &instream);

	virtual void readConstraints(const char* filename);


	std::vector<int> getSelectedNodes();
	std::vector<int> getConstrainedNodes();

	std::vector<int> getForcedNodeList(){ return loaded_nodeid; };

	Vector3d getExternalForce(int forceid){ return force_on_node[forceid]; };
	void setExternalForce(int forceid, Vector3d force){ force_on_node[forceid] = force; }

	MouseProjection* getMouseInfo(){ return mouseprojection; };

	/* ===========================================
	simulation export
	=========================================== */
	virtual void saveSimulator(const char* filename, fileFormatType formattype) const = 0;

	virtual void readSimulator(const char* filename, fileFormatType formattype) = 0;

	virtual void saveSimulatorBinary(std::ofstream &outstream) const;
	virtual void readSimulatorBinary(std::ifstream &instream);



	/* ===========================================
	mouse event
	=========================================== */
	virtual void singleClickEvent(){};
	virtual void mouseMoveEvent(){};
	virtual void rectSelectEvent();
	virtual void mouseReleaseEvent(){};
	virtual void clearSelect();

	virtual void setMouseProjection(MouseProjection* mouse){ this->mouseprojection = mouse; };

	virtual void cleanExternalForce();

	Eigen::Vector3d getSimulator_translate() const { return simulator_translate; }
	void setSimulator_translate(Eigen::Vector3d val);
	double getSimulator_scale() const { return simulator_scale; }
	void setSimulator_scale(double val);

	Eigen::Vector3d getSimulator_oritranslate() const { return simulator_oritranslate; }
	void setSimulator_oritranslate(Eigen::Vector3d val);

	bool getRecordExternalFroceFlag() const { return recordExternalFroceflag; }
	void setRecordExternalFroceFlag(bool val) { recordExternalFroceflag = val; }

	bool getUseExternalForceFlag() const { return useExternalForce; }
	void setUseExternalForceFlag(bool val) { useExternalForce = val; }

	double getMouseForceRatio() const { return mouseForceRatio; }
	void setMouseForceRatio(double val) { mouseForceRatio = val; }

	double getVolumtricmesh_volume() const;
	void setVolumtricmesh_volume(double val) { volumtricmesh_volume = val; }

	bool getRecordIntegratorflag() const { return recordIntegratorflag; }
	void setRecordIntegratorflag(bool val) { recordIntegratorflag = val; }

	LoboVolumetricMeshTriMeshAdapter* getVolumetric_obj_adapter() const { return volumetric_obj_adapter; }
	int getSimulation_steps() const { return simulation_steps; }
	void setSimulation_steps(int val) { simulation_steps = val; }

	bool getAutoInitSimulator() const { return autoInitSimulator; }
	void setAutoInitSimulator(bool val) { autoInitSimulator = val; }

	bool getUseGravity() const { return useGravity; }
	void setUseGravity(bool val) { useGravity = val; }

	double* getMouseExternalForce();
	double* getCollisionExternalForce();
	double* getALLExternalForce();
	double* getIntegratorExternalForce();

	CollisionHandle* getCollisionHandle() const { return collisionHandle; }
	void setCollisionHandle(CollisionHandle* val) { collisionHandle = val; }
	bool getUseBarMaterial() const { return useBarMaterial; }
	void setUseBarMaterial(bool val) { useBarMaterial = val; }
	bool getUseCollisionListVersion() const { return useCollisionListVersion; }
	void setUseCollisionListVersion(bool val) { useCollisionListVersion = val; }
	int getExternalForceType() const { return externalForceType; }
	void setExternalForceType(int val) { externalForceType = val; }
	bool getReadSimulatorFile() const { return readSimulatorFile; }
	void setReadSimulatorFile(bool val) { readSimulatorFile = val; }
	bool getUniformVolumtricMesh() const { return normalizaVolumtricMesh; }
	void setNormalizaVolumtricMesh(bool val) { normalizaVolumtricMesh = val; }
protected:

	bool ownVolumetric_mesh;
	virtual void addOnExternalForce(double* externalForce);
	virtual void computeExternalForce(double* externalForce);
	virtual void createIntegrator(){}; // derived class will create a inegrator
	virtual void readConfig(std::ifstream &inStream);
	virtual void initCollisionHandle();
	virtual void initVolumnMeshTriMeshAdapter();
	virtual void mallocVolumeMeshTriMeshAdapter();

	virtual void updateTriAndVolumeMesh(double *displacement);


	std::string simulatorname;
	std::string simulatorSaveFile;
	bool readSimulatorFile;
	fileFormatType file_type;

	Vector3d simulator_translate;
	Vector3d simulator_oritranslate;
	double simulator_scale;

	LoboTriMesh* tri_mesh;
	CollisionHandle* collisionHandle;
	CollisionDetector* collisionDetector;

	std::vector<LoboTriMesh*> tri_mesh_list;
	std::vector<LoboVolumetricMeshTriMeshAdapter*> volumetric_obj_adapter_list;
	std::vector<CollisionHandle*> collisionHandle_list;
	std::vector<CollisionDetector*> collisionDetector_list;
	bool useCollisionListVersion;


	LoboVolumetricMesh* volumetric_mesh;
	LoboVolumetricMeshGraph* volumetric_mesh_graph;
	LoboVolumetricMeshTriMeshAdapter* volumetric_obj_adapter;

	//special
	FunnelForceGenerator* forcegen;

	LoboIntegrator* integrator; // inegrator
	MouseProjection* mouseprojection;

	std::vector<int> selectedNodes;
	std::vector<int> constrainedNodes;
	int numConstrainedDOFs;
	int * constrainedDOFs;

	int simulation_steps;
	int integrator_skip;
	SimulationState simulationstate;
	int newtonIteration;

	//integrator setting
	double timestep;
	double dampingMassCoef;
	double dampingStiffnessCoef;
	double poisson;
	double youngmodulus;
	double density;

	double collisionRatio;
	double dampingRatio;

	double mouseForceRatio;

	PoseDataSet* runtimePoseDataSet;

	std::vector<int> loaded_nodeid;
	std::vector<Vector3d> force_on_node;
	int currentforceid;

	std::vector<VectorXd> externalforceSequence;
	bool recordExternalFroceflag;
	bool useExternalForce;

	bool recordIntegratorflag;
	bool useGravity;
	bool useBarMaterial;
	bool normalizaVolumtricMesh;

	int externalForceType;


	//if this is true, simulator will init after readconfig and ready for simulation
	bool autoInitSimulator;


	double volumtricmesh_volume;

	VectorXd externalForce;
	VectorXd mouseForce;
	VectorXd collisionExternalForce;

	Vector3d gravityDirection;

};

