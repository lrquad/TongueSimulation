#include "loboSimulatorBase.h"
#include <fstream>
#include "Simulator/PoseDataSet/PoseDataSet.h"
#include "Simulator/collision/CollisionHandle.h"
#include "Simulator/ExternalForceGenerator/ExternalForceGenerator.h"
#include "Render/MouseProjection.h"
#include "LoboVolumetricMesh/LoboElement.h"

LoboSimulatorBase::LoboSimulatorBase(LoboVolumetricMesh* volumetricmesh, std::ifstream &readconfig, LoboTriMesh* tri_mesh)
{
	ownVolumetric_mesh = false;
	this->volumetric_mesh = volumetricmesh;
	this->tri_mesh = tri_mesh;
	registerTriMesh(tri_mesh);
	initConstructor();
	readConfig(readconfig);
	simulationstate = SimulationState::beforeinit;
}

LoboSimulatorBase::LoboSimulatorBase()
{
	ownVolumetric_mesh = true;
	tri_mesh = NULL;
	initConstructor();
	simulationstate = SimulationState::beforeinit;
}

LoboSimulatorBase::LoboSimulatorBase(std::ifstream &readconfig, LoboTriMesh* tri_mesh)
{
	ownVolumetric_mesh = true;
	this->tri_mesh = tri_mesh;
	registerTriMesh(tri_mesh);
	initConstructor();
	readConfig(readconfig);
	simulationstate = SimulationState::beforeinit;
}

void LoboSimulatorBase::setSimulatorStart()
{
	simulationstate = SimulationState::start;
}

void LoboSimulatorBase::setSimulatorStop()
{
	simulationstate = SimulationState::pause;
}

LoboSimulatorBase::~LoboSimulatorBase()
{
	if (ownVolumetric_mesh)
	delete volumetric_mesh;

	if (integrator!=NULL)
	delete integrator;


	if (getUseCollisionListVersion())
	{
		for (int i = 0; i < collisionHandle_list.size(); i++)
		{
			delete collisionHandle_list[i];
		}
		collisionHandle_list.clear();
	}
	else
	{
		delete collisionHandle;
	}


	for (int i = 0; i < volumetric_obj_adapter_list.size(); i++)
	{
		delete volumetric_obj_adapter_list[i];
	}
	volumetric_obj_adapter_list.clear();
	
}

void LoboSimulatorBase::initConstructor()
{
	 timestep = 1.0 / 670;
	 dampingMassCoef = 0.01; // 0.3 for snake
	 dampingStiffnessCoef = 0.090; //0.05 for staypuft
	 youngmodulus = 1000;
	 poisson = 0.4;
	 externalForceType = 1; //funnel force
	 integrator_skip = 1;
	 newtonIteration = 5;
	 readSimulatorFile = false;
	 simulatorSaveFile = "";

	 setMouseForceRatio(10);

	 collisionRatio = 100;
	 dampingRatio = 0.5;
	 density = 1.0;

	 simulation_steps = 0;
	 simulatorname = "defaultName";
	 integrator = NULL;
	 volumetric_mesh = NULL;
	 constrainedDOFs = NULL;
	 runtimePoseDataSet = NULL;
	 collisionHandle = NULL;
	 collisionDetector = NULL;
	 mouseprojection = NULL;
	 volumetric_obj_adapter = NULL;
	 simulationstate = SimulationState::beforeinit;
	 forcegen = NULL;
	 recordExternalFroceflag = false;
	 useExternalForce = false;
	 recordIntegratorflag = false;
	 autoInitSimulator = false;
	 useGravity = false;
	 setUseBarMaterial(false);
	 setUseCollisionListVersion(false);
	 setNormalizaVolumtricMesh(false);
	 numConstrainedDOFs = 0;
	 constrainedDOFs = NULL;

	 simulator_translate = Vector3d::Zero();
	 simulator_oritranslate = Vector3d::Zero();

	 if (tri_mesh != NULL)
	 {
		 setSimulator_oritranslate(tri_mesh->getTranslateFromOri());
	 }

	 setSimulator_translate(Vector3d::Zero());
	 setSimulator_scale(1.0);
	 setVolumtricmesh_volume(0);
	 
	 gravityDirection = Vector3d(0, -1, 0);

}

void LoboSimulatorBase::addOnExternalForce(double* externalForce)
{
	int numVertex = volumetric_mesh->getNumVertices();
	int r = numVertex * 3;
	for (int i = 0; i < loaded_nodeid.size(); i++)
	{
		int nodeid = loaded_nodeid[i];

		externalForce[nodeid * 3 + 0] += force_on_node[i].data()[0];
		externalForce[nodeid * 3 + 1] += force_on_node[i].data()[1];
		externalForce[nodeid * 3 + 2] += force_on_node[i].data()[2];
	}
}

void LoboSimulatorBase::computeExternalForce(double* externalForce)
{
	int numVertex = volumetric_mesh->getNumVertices();
	int r = numVertex * 3;
	memset(externalForce, 0, sizeof(double)*r);
	for (int i = 0; i < loaded_nodeid.size(); i++)
	{
		int nodeid = loaded_nodeid[i];

		externalForce[nodeid * 3 + 0] = force_on_node[i].data()[0];
		externalForce[nodeid * 3 + 1] = force_on_node[i].data()[1];
		externalForce[nodeid * 3 + 2] = force_on_node[i].data()[2];
	}
}

void LoboSimulatorBase::readConfig(std::ifstream &inStream)
{
	std::string token;
	while (true)
	{
		inStream >> token;
		if (token == "base_end")
		{
			std::cout << "base config finished ..." << std::endl;
			break;
		}

		if (token[0] == '#')
		{
			inStream.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
		}

		if (token == "timestep")
		{
			inStream >> timestep;
		}

		if (token == "gravity")
		{
			int flag;
			inStream >> flag;
			this->setUseGravity(flag);
		}

		if (token == "dampingMassCoef")
		{
			inStream >> dampingMassCoef;
		}

		if (token == "dampingStiffnessCoef")
		{
			inStream >> dampingStiffnessCoef;
		}

		if (token == "simulator_name")
		{
			inStream >> simulatorname;
		}

		if (token == "simulator_translate")
		{
			double x, y, z;
			inStream >> x >> y >> z;
			Vector3d temp(x,y,z);
			setSimulator_translate(temp);
		}

		if (token == "simulator_scale")
		{
			double scale;
			inStream >> scale;
			setSimulator_scale(scale);
		}

		if (token == "recordIntegrator")
		{
			setRecordIntegratorflag(true);
			std::cout << "will record Integrator" << std::endl;
		}

		if (token == "autoInitSimulator")
		{
			autoInitSimulator = true;
		}

		if (token == "collisionRatio")
		{
			inStream >> collisionRatio;
		}

		if (token == "dampingRatio")
		{
			inStream >> dampingRatio;
		}

		if (token == "youngmodulus")
		{
			inStream >> youngmodulus;
		}

		if (token == "poisson")
		{
			inStream >> poisson;
		}

		if (token == "density")
		{
			inStream >> density;
		}

		if (token == "barmaterial")
		{
			setUseBarMaterial(true);
		}

		if (token == "externalForceType")
		{
			int index;
			inStream >> index;
			setExternalForceType(index);
		}

		if (token == "integrator_skip")
		{
			inStream >> integrator_skip;
		}

		if (token == "newtonIteration")
		{
			inStream >> newtonIteration;
		}

		if (token == "uniformVolumMesh")
		{
			inStream >> normalizaVolumtricMesh;
			std::cout << "uniform " << getUniformVolumtricMesh() << std::endl;
		}

		if (token == "gravityDirection")
		{
			inStream >> gravityDirection.data()[0];
			inStream >> gravityDirection.data()[1];
			inStream >> gravityDirection.data()[2];
			gravityDirection.normalize();
		}

		if (token == "forceratio")
		{
			double value;
			inStream >> value;
			this->setMouseForceRatio(value);
		}

	}
}

void LoboSimulatorBase::initCollisionHandle()
{
	
	if (!getUseCollisionListVersion())
	{
		if (collisionDetector != NULL)
		{
			collisionHandle = new CollisionHandle(collisionDetector, volumetric_mesh, volumetric_obj_adapter, integrator);
			collisionHandle->setSpringForceRatio(collisionRatio);
			collisionHandle->setDampingRatio(dampingRatio);
		}
	}
	else
	{
		for (int i = 0; i < collisionDetector_list.size(); i++)
		{
			CollisionHandle* handle = new CollisionHandle(collisionDetector_list[i], volumetric_mesh, volumetric_obj_adapter_list[i], integrator);
			handle->setSpringForceRatio(collisionRatio);
			handle->setDampingRatio(dampingRatio);
			collisionHandle_list.push_back(handle);
		}
	}

}

void LoboSimulatorBase::initVolumnMeshTriMeshAdapter()
{
	volumetric_obj_adapter = new LoboVolumetricMeshTriMeshAdapter(volumetric_mesh, tri_mesh);
	volumetric_obj_adapter->computeNodeBarycentricWeight();
	volumetric_obj_adapter_list.push_back(volumetric_obj_adapter);

	if (tri_mesh_list.size() > 1)
	{
		for (int i = 1; i < tri_mesh_list.size(); i++)
		{
			LoboVolumetricMeshTriMeshAdapter* adapter = new LoboVolumetricMeshTriMeshAdapter(volumetric_mesh, tri_mesh_list[i]);
			adapter->computeNodeBarycentricWeight();
			volumetric_obj_adapter_list.push_back(adapter);
		}
	}
}

void LoboSimulatorBase::mallocVolumeMeshTriMeshAdapter()
{
	volumetric_obj_adapter = new LoboVolumetricMeshTriMeshAdapter(volumetric_mesh, tri_mesh);
	volumetric_obj_adapter_list.push_back(volumetric_obj_adapter);

	if (tri_mesh_list.size() > 1)
	{
		for (int i = 1; i < tri_mesh_list.size(); i++)
		{
			LoboVolumetricMeshTriMeshAdapter* adapter = new LoboVolumetricMeshTriMeshAdapter(volumetric_mesh, tri_mesh_list[i]);
			volumetric_obj_adapter_list.push_back(adapter);
		}
	}
}

void LoboSimulatorBase::updateTriAndVolumeMesh(double *displacement)
{
	volumetric_mesh->setDisplacement(displacement);
	for (int i = 0; i < volumetric_obj_adapter_list.size(); i++)
	{
		volumetric_obj_adapter_list[i]->updateObjMesh();
	}
}

void LoboSimulatorBase::resetExternalForce()
{
	if (integrator!=NULL)
	this->integrator->setExternalForcesToZero();
	externalForce.setZero();
	//mouseForce.setZero();
	//collisionExternalForce.setZero();
}

void LoboSimulatorBase::insertCurrentPose()
{
	runtimePoseDataSet->insertNewPose(integrator->getq());
}

void LoboSimulatorBase::savePoseSet(const char* filename)
{
	std::cout << "pose set saved at " << filename << std::endl;
	runtimePoseDataSet->savePoseFile(filename);
}

void LoboSimulatorBase::readPoseSet(const char* filename)
{
	std::cout << "read pose set" << filename << std::endl;
	runtimePoseDataSet->loadPoseFile(filename);
}

void LoboSimulatorBase::clearPoseSet()
{
	runtimePoseDataSet->clearData();
}

void LoboSimulatorBase::generateBendingForce()
{
	ExternalForceGenerator* forcegen = new ExternalForceGenerator(NULL);
	int numVertex = volumetric_mesh->getNumVertices();
	forcegen->generateForce(100, numVertex * 3, externalforceSequence);

	this->setUseExternalForceFlag(true);

	delete forcegen;

}

void LoboSimulatorBase::readExternalForce(const char* filename)
{
	externalforceSequence.clear();
	int numVertex = volumetric_mesh->getNumVertices();
	int N;
	int forcesize;
	std::ifstream in(filename);
	in >> forcesize;
	in >> N;
	for (int i = 0; i < forcesize; i++)
	{
		VectorXd force(N);
		for (int j = 0; j < N; j++)
		{
			in >> force.data()[j];
		}
		externalforceSequence.push_back(force);
	}
	in.close();
}

void LoboSimulatorBase::saveExternalForce(const char* filename)
{
	int numVertex = volumetric_mesh->getNumVertices();
	std::ofstream output(filename);
	output.precision(32);
	output << externalforceSequence.size() << std::endl;
	output << numVertex * 3 << std::endl;

	for (int i = 0; i < externalforceSequence.size(); i++)
	{
		output << externalforceSequence[i].transpose() << std::endl;
	}

	output.close();
}

int LoboSimulatorBase::saveIntegratorResults(const char* filename)
{
	if (this->integrator == NULL)
	{
		return 0;
	}
	//integrator->saveResults(filename);
	integrator->saveResultsBinary(filename);
	return 1;
}

int LoboSimulatorBase::readIntegratorResults(const char* filename)
{
	if (this->integrator == NULL)
	{
		return 0;
	}
	//integrator->readResult(filename);
	integrator->readResultsBinary(filename);
	integrator->setUseLoadedStepResults(true);
	return 1;
}

int LoboSimulatorBase::setRecordIntegrator(bool b)
{
	if (this->integrator == NULL)
	{
		this->setRecordIntegratorflag(b);
		return 0;
	}

	integrator->setRecordLoadedStepResults(b);
	return 1;
}

void LoboSimulatorBase::registerCollision(CollisionDetector* collisionDetector_)
{
	if (collisionDetector_list.size() == 0)
	{
		this->collisionDetector = collisionDetector_;
		collisionDetector_list.push_back(collisionDetector_);
	}else
	if (collisionDetector_list.size() >= 1)
	{
		collisionDetector_list.push_back(collisionDetector_);
		setUseCollisionListVersion(true);
		std::cout << "System detected multi collision objects, Lobosimulator will use collision_list version" << std::endl;
	}
	std::cout <<"collisionDetector_list ==> "<< collisionDetector_list.size() << std::endl;
}

void LoboSimulatorBase::registerTriMesh(LoboTriMesh* tri_mesh)
{
	tri_mesh_list.push_back(tri_mesh);
	if (tri_mesh_list.size() > 1)
	{
		std::cout << "use multi tri mesh in one simulator." << std::endl;
	}
}

void LoboSimulatorBase::saveSelectedVolumetricMeshAscii(const char* filenamebase)
{
	int numVertex = volumetric_mesh->getNumVertices();
	std::vector<bool> nodemark(numVertex);
	std::fill(nodemark.begin(), nodemark.end(), false);
	for (int i = 0; i < selectedNodes.size(); i++)
	{
		int nodeid = selectedNodes[i];
		nodemark[nodeid] = true;
	}

	int numEle = volumetric_mesh->getNumElements();
	std::vector<bool> elemark(numEle);
	std::fill(elemark.begin(), elemark.end(), false);



	for (int i = 0; i < numEle; i++)
	{
		LoboElement* ele = volumetric_mesh->getElement(i);
		for (int j = 0; j < ele->node_indices.size(); j++)
		{
			int nodeid = ele->node_indices[j];
			if (nodemark[nodeid])
			{
				elemark[i] = true;
			}
		}
	}

	std::vector<int> ele_indices;
	for (int i = 0; i < numEle; i++)
	{
		if (elemark[i])
		{
			ele_indices.push_back(i);
		}
	}

	std::vector<int> submap(numVertex);
	std::fill(submap.begin(), submap.end(), -1);
	std::vector<bool> subnodemark(numVertex);
	std::fill(subnodemark.begin(), subnodemark.end(), false);

	for (int i = 0; i < ele_indices.size(); i++)
	{
		int eleid = ele_indices[i];

		LoboElement* ele = volumetric_mesh->getElement(eleid);
		for (int j = 0; j < ele->node_indices.size(); j++)
		{
			int nodeid = ele->node_indices[j];
			subnodemark[nodeid] = true;
		}
	}

	int offset = 0;
	for (int i = 0; i < subnodemark.size(); i++)
	{
		if (subnodemark[i])
		{
			submap[i] = offset;
			offset++;
		}
	}

	volumetric_mesh->saveSubMeshAsscii(filenamebase, ele_indices, subnodemark, submap);
}

void LoboSimulatorBase::saveVolumtricMeshSurfaceTri(const char* filename)
{
	volumetric_mesh->computeSurface();
	volumetric_mesh->saveSurfaceToObjAscii(filename);
}

LoboVolumetricMesh* LoboSimulatorBase::getVolumetricMesh()
{
	return volumetric_mesh;
}

LoboTriMesh* LoboSimulatorBase::getTriMesh()
{
	return tri_mesh;
}

LoboIntegrator* LoboSimulatorBase::getIntegrator()
{
	return integrator;
}

void LoboSimulatorBase::saveConstraints(const char* filename)
{
	std::ofstream output(filename);
	output << numConstrainedDOFs << std::endl;
	for (int i = 0; i < numConstrainedDOFs; i++)
	{
		output << constrainedDOFs[i] << std::endl;
	}
	output.close();
}

void LoboSimulatorBase::saveConstraintsBinary(std::ofstream &outstream) const
{
	outstream.write((char*)&numConstrainedDOFs, sizeof(int));
	outstream.write((char*)constrainedDOFs, sizeof(int)*numConstrainedDOFs);
}

void LoboSimulatorBase::readConstraintsBianry(std::ifstream &instream)
{
	instream.read((char*)&numConstrainedDOFs, sizeof(int));
	if (constrainedDOFs != NULL)
	{
		free(constrainedDOFs);
	}
	constrainedDOFs = (int*)malloc(sizeof(int)*numConstrainedDOFs);
	instream.read((char*)constrainedDOFs, sizeof(int)*numConstrainedDOFs);
}

void LoboSimulatorBase::readConstraints(const char* filename)
{
	if (constrainedDOFs != NULL)
	{
		free(constrainedDOFs);
	}

	int numVertex = volumetric_mesh->getNumVertices();
	std::vector<bool> constrained_mark(numVertex);
	std::fill(constrained_mark.begin(), constrained_mark.end(), false);
	
	std::ifstream input(filename);
	input >> numConstrainedDOFs;
	constrainedDOFs = (int*)malloc(sizeof(int) * numConstrainedDOFs);
	
	for (int i = 0; i < numConstrainedDOFs; i++)
	{
		input >> constrainedDOFs[i];
		constrained_mark[constrainedDOFs[i] / 3] = true;
	}

	this->constrainedNodes.clear();
	for (int i = 0; i < numVertex; i++)
	{
		if (constrained_mark[i])
		{
			constrainedNodes.push_back(i);
		}
	}
	std::cout << "created constrain: size " << numConstrainedDOFs << std::endl;
}

std::vector<int> LoboSimulatorBase::getSelectedNodes()
{
	return selectedNodes;
}

std::vector<int> LoboSimulatorBase::getConstrainedNodes()
{
	return constrainedNodes;
}

void LoboSimulatorBase::saveSimulatorBinary(std::ofstream &outstream) const
{
	std::cout << "LoboSimulatorBase save ... " << std::endl;
	outstream.write((char*)&timestep, sizeof(double));
	outstream.write((char*)&useGravity, sizeof(bool));
	outstream.write((char*)&dampingMassCoef, sizeof(double));
	outstream.write((char*)&dampingStiffnessCoef, sizeof(double));

	size_t len = simulatorname.size();
	outstream.write((char*)&len, sizeof(size_t));
	outstream.write(simulatorname.c_str(), len);

	outstream.write((char*)simulator_translate.data(), sizeof(Vector3d::Scalar)*simulator_translate.size());
	outstream.write((char*)&simulator_scale, sizeof(double));
	outstream.write((char*)&recordIntegratorflag, sizeof(bool));

	outstream.write((char*)&autoInitSimulator, sizeof(bool));
	outstream.write((char*)&collisionRatio,sizeof(double));
	outstream.write((char*)&dampingRatio, sizeof(double));


	outstream.write((char*)&youngmodulus, sizeof(double));
	outstream.write((char*)&poisson, sizeof(double));
	outstream.write((char*)&density, sizeof(double));

	outstream.write((char*)&useBarMaterial, sizeof(bool));
	outstream.write((char*)&externalForceType, sizeof(int));

	outstream.write((char*)&integrator_skip, sizeof(int));
	outstream.write((char*)&newtonIteration, sizeof(int));
}

void LoboSimulatorBase::readSimulatorBinary(std::ifstream &instream)
{
	std::cout << "readSimulatorBase read ... " << std::endl;
	instream.read((char*)&timestep, sizeof(double));

	instream.read((char*)&useGravity, sizeof(bool));
	instream.read((char*)&dampingMassCoef, sizeof(double));
	instream.read((char*)&dampingStiffnessCoef, sizeof(double));

	size_t len;
	instream.read((char*)&len, sizeof(size_t));
	char* temp = new char[len + 1];
	instream.read(temp, len);
	temp[len] = '\0';
	simulatorname = temp;
	delete[] temp;

	instream.read((char*)simulator_translate.data(), sizeof(Vector3d::Scalar)*simulator_translate.size());
	instream.read((char*)&simulator_scale, sizeof(double));
	instream.read((char*)&recordIntegratorflag, sizeof(bool));
	
	instream.read((char*)&autoInitSimulator, sizeof(bool));
	instream.read((char*)&collisionRatio, sizeof(double));
	instream.read((char*)&dampingRatio, sizeof(double));
	
	instream.read((char*)&youngmodulus, sizeof(double));
	instream.read((char*)&poisson, sizeof(double));
	instream.read((char*)&density, sizeof(double));
	
	instream.read((char*)&useBarMaterial, sizeof(bool));
	instream.read((char*)&externalForceType, sizeof(int));
	
	instream.read((char*)&integrator_skip, sizeof(int));
	instream.read((char*)&newtonIteration, sizeof(int));


}


void LoboSimulatorBase::rectSelectEvent()
{
	int numVertex = volumetric_mesh->getNumVertices();
	for (int i = 0; i < numVertex; i++)
	{
		Vector3d nodecur = volumetric_mesh->getNodePosition(i);
		Vector2d screenpos;
		double depth;
		mouseprojection->projectWorldToScreen(nodecur, screenpos, depth);
		if (mouseprojection->isInMouseRect(screenpos.x(), screenpos.y()))
		{
			selectedNodes.push_back(i);
		}
	}
}

void LoboSimulatorBase::clearSelect()
{
	selectedNodes.clear();
}

void LoboSimulatorBase::cleanExternalForce()
{
	loaded_nodeid.clear();
	force_on_node.clear();
	currentforceid = -1;

	if (integrator!=NULL)
	integrator->setExternalForcesToZero();

	externalForce.setZero();
	mouseForce.setZero();
	collisionExternalForce.setZero();

}

void LoboSimulatorBase::setSimulator_translate(Eigen::Vector3d val)
{
	simulator_translate = simulator_oritranslate+val;
}

void LoboSimulatorBase::setSimulator_scale(double val)
{
	simulator_scale = val;

	if (tri_mesh != NULL)
	{
		tri_mesh->scaleMeshAroundMeshcenter(simulator_scale);
	}

	if (volumetric_mesh != NULL)
	{
		volumetric_mesh->scaleVolumetricMesh(simulator_scale);
	}
}

void LoboSimulatorBase::setSimulator_oritranslate(Eigen::Vector3d val)
{
	simulator_oritranslate = val;
}

double LoboSimulatorBase::getVolumtricmesh_volume() const
{
	return volumtricmesh_volume;
}

double* LoboSimulatorBase::getMouseExternalForce()
{
	return mouseForce.data();
}

double* LoboSimulatorBase::getCollisionExternalForce()
{
	return collisionExternalForce.data();
}

double* LoboSimulatorBase::getALLExternalForce()
{
	return externalForce.data();
}

double* LoboSimulatorBase::getIntegratorExternalForce()
{
	return integrator->getExternalForces();
}
