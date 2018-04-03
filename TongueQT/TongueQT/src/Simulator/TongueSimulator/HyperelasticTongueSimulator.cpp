

#include "HyperelasticTongueSimulator.h"
#include "LoboVolumetricMesh/LoboVolumetriceMeshCore.h"
#include "Simulator/IsotropicHyperelasticFEM/isotropicHyperelasticCore.h"
#include "Simulator/ForceModel/IsotropicHyperlasticForceModel.h"
#include "Simulator/ReducedSubspaceModel/ReducedSubspaceModel.h"
#include "Integrator/ImpicitNewMarkDenseIntegrator.h"
#include "Simulator/ReducedForceModel/ReducedSubspaceForceModel.h"
#include "EigenMatrixIO\EigenMatrixIO.h"
#include <fstream>
#include "Render/MouseProjection.h"
#include "Functions/findElementInVector.h"
#include "cubature/cubatureCore.h"


#include "Simulator/TongueSimulator/WaveTongueMode.h"

HyperelasticTongueSimulator::HyperelasticTongueSimulator(std::ifstream &readconfig_, LoboTriMesh* obj_mesh, bool ifreadconfig) :LoboSimulatorBase(readconfig_, obj_mesh)
{
	nullAllPointer();
	if (ifreadconfig)
	{
		readConfig(readconfig_);
		if (volumetric_mesh != NULL)
		{
			selectedNodes.reserve(volumetric_mesh->getNumVertices());
		}
	}
}

HyperelasticTongueSimulator::~HyperelasticTongueSimulator()
{
	deleteAllPointer();
}

void HyperelasticTongueSimulator::nullAllPointer()
{
	waveToTongue = NULL;
}

void HyperelasticTongueSimulator::deleteAllPointer()
{

}



void HyperelasticTongueSimulator::initSimulator(int verbose /*= 0*/)
{
	waveToTongue = new WaveTongueMode();
	waveToTongue->setScale(1.0 / volumetric_mesh->getUniformScale());
	waveToTongue->setTranslate(-volumetric_mesh->getUniformT());
	waveToTongue->mapSensorDataSeed(volumetric_mesh);
	

	this->readModes("./data/SubspaceModes/tongueMD120.txt");

	int numVertex = volumetric_mesh->getNumVertices();
	int R = numVertex * 3;

	externalForce.resize(R);
	externalForce.setZero();

	mouseForce.resize(R);
	mouseForce.setZero();

	collisionExternalForce.resize(R);
	collisionExternalForce.setZero();

	((TetVolumetricMesh*)volumetric_mesh)->initElementShapeFunctionDerivate();

	volumetric_mesh_graph = new LoboVolumetricMeshGraph(volumetric_mesh);
	volumetric_mesh_graph->init();

	initVolumnMeshTriMeshAdapter();

	readSensorData("./data/tongue/sensordata/20150624_0_041_sync.txt");

	massMatrix = new SparseMatrix<double>(R, R);
	volumetric_mesh->computeMassMatrix(massMatrix);

	isotropicMaterial = new LoboStVKIsotropicMaterial(downCastTetVolMesh(volumetric_mesh), 0, 500);

	isotropicHyperelasticModel = new IsotropicHyperelasticModel(downCastTetVolMesh(volumetric_mesh), isotropicMaterial, massMatrix, 0.5, true, 500.0);

	isotropicHyperelasticModel->setIsInvertible(getIsInvertible());

	isotropicHyperelasticModel->setAddGravity(this->getUseGravity());
	isotropicHyperelasticModel->setGravityDirection(gravityDirection);


	reducedSubspaceModel = new ReducedSubspaceModel(volumetric_mesh, massMatrix, isotropicHyperelasticModel, &phi);

	reducedMassMatrix = phi.transpose()*(*massMatrix)*phi;

	forcemodel = new ReducedSubspaceForceModel(reducedSubspaceModel);


	std::cout << "init cubature" << std::endl;
	cubaturemodel = new CubatureModel(volumetric_mesh, reducedSubspaceModel, forcemodel,NULL);
	
	std::cout << " do trainin " << std::endl;

	cubaturemodel->readCubature("./data/CubatureData/tonguemd.txt");
	//cubaturemodel->doCubatureTraining();
	reducedSubspaceModel->setUseCubature(true);
	//cubaturemodel->saveCubature("./data/CubatureData/tonguemd.txt");

	initIntegrator();
	simulationstate = SimulationState::ready;

}

void HyperelasticTongueSimulator::updateSimulator(int verbose /*= 0*/)
{
	//fake input audio
	monitorNextStepV2();
	mergeExternalForce();

	integrator->doTimeStep();

	int r = phi.cols();
	VectorXd reducedq(r);
	memcpy(reducedq.data(), integrator->getq(), r*sizeof(double));

	VectorXd fullq = phi*reducedq;

	updateTriAndVolumeMesh(fullq.data());

	this->simulation_steps++;

}

void HyperelasticTongueSimulator::resetSimulator(int verbose /*= 0*/)
{

}

void HyperelasticTongueSimulator::insertAudioWaveIn(double *wave)
{
	if (waveToTongue == NULL)
	{
		return;
	}

	waveToTongue->appendNewFrame(wave);
}

LoboSimulatorBase::simulatorType HyperelasticTongueSimulator::getType()
{
	return LoboSimulatorBase::simulatorType::HTONGUE;
}

void HyperelasticTongueSimulator::saveSimulator(const char* filename, fileFormatType formattype) const
{

}

void HyperelasticTongueSimulator::readSimulator(const char* filename, fileFormatType formattype)
{

}

void HyperelasticTongueSimulator::readVolumetricMeshAscii(const char* filenamebase)
{
	TetVolumetricMesh* tet_volumetric_mesh = new TetVolumetricMesh();

	Vector3d translate = Vector3d::Zero();
	tet_volumetric_mesh->setUniformMeshAfterLoad(getUniformVolumtricMesh());
	tet_volumetric_mesh->readElementMesh(filenamebase, simulator_translate.data(), getSimulator_scale());

	volumetric_mesh = tet_volumetric_mesh;

	LoboVolumetricMesh::Material* materia = volumetric_mesh->getMaterialById(0);
	LoboVolumetricMesh::ENuMaterial* enmateria = (LoboVolumetricMesh::ENuMaterial*)materia;
	enmateria->setE(this->youngmodulus);
	enmateria->setNu(this->poisson);
	enmateria->setDensity(this->density);

	if (getUniformVolumtricMesh())
	{
		Vector3d tritranslate = -simulator_translate - volumetric_mesh->getUniformT();
		this->tri_mesh->translateMesh(tritranslate);
		this->tri_mesh->scaleAtCurrentUpdate(1.0 / volumetric_mesh->getUniformScale());
	}

	tonguemodel.setScale(1.0 / volumetric_mesh->getUniformScale());
	tonguemodel.setTranslate(-volumetric_mesh->getUniformT());

}

void HyperelasticTongueSimulator::initSimulatorGlobalConstraints()
{

}

void HyperelasticTongueSimulator::singleClickEvent()
{
	int numVertex = volumetric_mesh->getNumVertices();
	Vector2d mousescreen;
	mousescreen.x() = mouseprojection->getPreMouseX();
	mousescreen.y() = mouseprojection->getPreMouseY();

	double mindistance = DBL_MAX;
	double mindepth = DBL_MAX;

	int nodeid = -1;

	std::vector<int> surfaceNodes = volumetric_mesh->getSurface_node();
	numVertex = surfaceNodes.size();

	for (int i = 0; i < numVertex; i++)
	{
		int node_id = surfaceNodes[i];
		Vector3d nodecur = volumetric_mesh->getNodePosition(node_id);
		Vector2d screenpos;
		double depth;
		mouseprojection->projectWorldToScreenDepth(nodecur, screenpos, depth);

		Vector2d distance2d = mousescreen - screenpos;

		Vector3d distanceV3(distance2d.x(), distance2d.y(), 10 * depth);

		double distance = distanceV3.norm();

		if (distance <= mindistance)
		{
			mindistance = distance;
			nodeid = node_id;
		}
	}

	std::cout << "selected: " << nodeid << std::endl;
	std::cout << volumetric_mesh->getNodePosition(nodeid).transpose() << std::endl;

	mouseprojection->setBindedNodeid(nodeid);

	if (!findElement(loaded_nodeid, nodeid))
	{
		this->loaded_nodeid.push_back(nodeid);
		this->force_on_node.push_back(Vector3d::Zero());

	}

	currentforceid = loaded_nodeid.size() - 1;
}

void HyperelasticTongueSimulator::rectSelectEvent()
{

}

void HyperelasticTongueSimulator::mouseMoveEvent()
{
	if (simulationstate == LoboSimulatorBase::beforeinit)
	{
		return;
	}

	int numVertex = volumetric_mesh->getNumVertices();
	Vector2d mousescreen;
	mousescreen.x() = mouseprojection->getMouseX();
	mousescreen.y() = mouseprojection->getMouseY();
	int bindedNodeid = mouseprojection->getBindedNodeid();
	Vector3d nodecur = volumetric_mesh->getNodePosition(bindedNodeid);
	Vector2d nodescreen;
	double depth;
	mouseprojection->projectWorldToScreen(nodecur, nodescreen, depth);

	Vector3d mouseWorld;
	mouseprojection->projectScreenToWorld(mousescreen, depth, mouseWorld);
	Vector3d force = (mouseWorld - nodecur)*mouseForceRatio;
	//force.data()[1] *= 0.1;

	force_on_node[currentforceid] = force;

	computeExternalForce(mouseForce.data());
}

void HyperelasticTongueSimulator::mouseReleaseEvent()
{
	this->cleanExternalForce();
}

void HyperelasticTongueSimulator::readModes(const char* filename)
{
	EigenMatrixIO::read_binary(filename, phi);
}

void HyperelasticTongueSimulator::initIntegrator()
{
	if (integrator != NULL)
	{
		delete integrator;
	}

	int r = phi.cols();
	integrator = new ImpicitNewMarkDenseIntegrator(r, timestep, &reducedMassMatrix, forcemodel, 0, 0, dampingMassCoef, dampingStiffnessCoef, this->newtonIteration);

}

void HyperelasticTongueSimulator::readConfig(std::ifstream &inStream)
{
	std::string token;
	while (true)
	{
		inStream >> token;

		if (inStream.eof())
		{
			break;
		}

		if (token == "end")
		{
			std::cout << "simulator config finished ..." << std::endl;
			std::cout << std::endl;
			break;
		}

		if (token[0] == '#')
		{
			inStream.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
		}

		if (token == "load")
		{
			if (!ownVolumetric_mesh)
			{
				std::cout << "This simulator will use exist volumetric mesh." << std::endl;
				continue;
			}

			std::cout << "load volumetric mesh ...";
			std::string filebase;
			inStream >> filebase;
			std::cout << filebase << std::endl;
			this->readVolumetricMeshAscii(filebase.c_str());
		}



		if (token == "invertible")
		{
			setIsInvertible(true);
		}
	}
}

void HyperelasticTongueSimulator::mergeExternalForce()
{
	VectorXd mergedExternalForce = externalForce + mouseForce + collisionExternalForce;
	VectorXd reducedexternalforce = phi.transpose()*mergedExternalForce;

	integrator->setExternalForces(reducedexternalforce.data());
}

void HyperelasticTongueSimulator::readSensorData(const char* filename)
{
	tonguemodel.readSensorData(filename);
	tonguemodel.mapSensorDataSeed(volumetric_mesh);

	if (getSimulatorState() != LoboSimulatorBase::beforeinit)
	{
		resetSimulator();
	}
}

void HyperelasticTongueSimulator::monitorNextStep()
{
	int numdata = tonguemodel.getNumData();
	double volume = volumetric_mesh->computeCurrentMeshVolume();
	
	int numConstrainedSensor = 4;
	int constrainedSensorDofs = numConstrainedSensor * 3;
	int numVertex = volumetric_mesh->getNumVertices();

	MatrixXd ConstrainMatrix(constrainedSensorDofs, numVertex * 3);
	ConstrainMatrix.setZero();
	VectorXd constrainVector(constrainedSensorDofs);
	constrainVector.setZero();

	std::vector<int> seed_index = tonguemodel.getSensorseedindex();

	for (int i = 0; i < numConstrainedSensor; i++)
	{
		Vector3d restposition = volumetric_mesh->getNodePosition(seed_index[i]);
		restposition = tonguemodel.getSensorPosition(i, simulation_steps);
		Vector3d currentposition = tonguemodel.getSensorPosition(i, simulation_steps + 1);
		
		Vector3d stepu = currentposition - restposition;
		/*stepu.setZero();
		stepu.data()[1] = -0.01;*/

		for (int x = 0; x < 3; x++)
		{
			int row = i * 3 + x;
			int col = seed_index[i] * 3 + x;
			ConstrainMatrix.data()[col*ConstrainMatrix.rows() + row] = 1;
			constrainVector.data()[row] = stepu.data()[x];
		}
	}

	MatrixXd reducedConstrainMatrix = ConstrainMatrix*(phi);

	((ImpicitNewMarkDenseIntegrator*)integrator)->setConstrainByLagrangeMulti(reducedConstrainMatrix, constrainVector);

	tonguemodel.updateSensorNodePosition(simulation_steps + 1);
}

void HyperelasticTongueSimulator::monitorNextStepV2()
{
	int numConstrainedSensor = 4;
	int constrainedSensorDofs = numConstrainedSensor * 3;
	int numVertex = volumetric_mesh->getNumVertices();

	MatrixXd ConstrainMatrix(constrainedSensorDofs, numVertex * 3);
	ConstrainMatrix.setZero();
	VectorXd constrainVector(constrainedSensorDofs);
	constrainVector.setZero();

	std::vector<int> seed_index = waveToTongue->getSensorseedindex();


	for (int i = 0; i < numConstrainedSensor; i++)
	{
		Vector3d restposition = volumetric_mesh->getNodePosition(seed_index[i]);

		restposition = waveToTongue->getCurFrame(i);
		Vector3d currentposition = waveToTongue->getNextFrame(i);

		Vector3d stepu = currentposition - restposition;
		/*stepu.setZero();
		stepu.data()[1] = -0.01;*/

		for (int x = 0; x < 3; x++)
		{
			int row = i * 3 + x;
			int col = seed_index[i] * 3 + x;
			ConstrainMatrix.data()[col*ConstrainMatrix.rows() + row] = 1;
			constrainVector.data()[row] = stepu.data()[x];
		}
	}

	waveToTongue->movefowardCurframe();
	MatrixXd reducedConstrainMatrix = ConstrainMatrix*(phi);

	((ImpicitNewMarkDenseIntegrator*)integrator)->setConstrainByLagrangeMulti(reducedConstrainMatrix, constrainVector);

}

HyperelasticTongueSimulator* downCastHyperTongueSimulator(LoboSimulatorBase* simulator)
{
	return (HyperelasticTongueSimulator*)simulator;
}
