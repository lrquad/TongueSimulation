#include "LoboIsotropicHyperlasticSimulator.h"
#include "LoboVolumetricMesh/LoboVolumetriceMeshCore.h"
#include "Render/MouseProjection.h"
#include "Functions/findElementInVector.h"


#include "Simulator/IsotropicHyperelasticFEM/isotropicHyperelasticCore.h"

#include "Simulator/PoseDataSet/PoseDataSet.h"
#include "Simulator/ForceModel/IsotropicHyperlasticForceModel.h"
#include "Simulator/corotational/CorotationalModel.h"
#include <fstream>
#include "Integrator/ImplicitNewMatkSparseIntegrator.h"
#include "vegatest.h"
#include <QElapsedTimer>

#include "Simulator/ForceField/RotateForceField.h"

LoboIsotropicHyperlasticSimulator::LoboIsotropicHyperlasticSimulator(std::ifstream &readconfig_, LoboTriMesh* obj_mesh, bool ifreadconfig) :LoboSimulatorBase(readconfig_, obj_mesh)
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

LoboIsotropicHyperlasticSimulator::~LoboIsotropicHyperlasticSimulator()
{
	deleteAllPointer();
}

void LoboIsotropicHyperlasticSimulator::nullAllPointer()
{
	setIsInvertible(true);

	isotropicHyperelasticModel = NULL;
	isotropicMaterial = NULL;
	massMatrix = NULL;
	forcemodel = NULL;
	materialtype = NeoHookean;
}

void LoboIsotropicHyperlasticSimulator::deleteAllPointer()
{
	delete isotropicHyperelasticModel;
	delete isotropicMaterial;
	delete massMatrix;
	delete forcemodel;
}

void LoboIsotropicHyperlasticSimulator::initSimulator(int verbose /*= 0*/)
{
	int numVertex = volumetric_mesh->getNumVertices();
	int R = numVertex * 3;
	runtimePoseDataSet = new PoseDataSet(R);

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

	massMatrix = new SparseMatrix<double>(R, R);
	volumetric_mesh->computeMassMatrix(massMatrix);

	

	switch (getMaterialtype())
	{
	case NeoHookean:
		isotropicMaterial = new LoboneoHookeanIsotropicMaterial(downCastTetVolMesh(volumetric_mesh), 0, 500);
		std::cout << "use neo hookean" << std::endl;
		break;
	case STVK:
		isotropicMaterial = new LoboStVKIsotropicMaterial(downCastTetVolMesh(volumetric_mesh), 0, 500);
		std::cout << "use stvk" << std::endl;
		break;
	default:
		break;
	}

	isotropicHyperelasticModel = new IsotropicHyperelasticModel(downCastTetVolMesh(volumetric_mesh), isotropicMaterial, massMatrix, 0.5, true, 500.0);

	isotropicHyperelasticModel->setIsInvertible(getIsInvertible());

	isotropicHyperelasticModel->setAddGravity(this->getUseGravity());
	isotropicHyperelasticModel->setGravityDirection(gravityDirection);

	forcemodel = new IsotropicHyperlasticForceModel(isotropicHyperelasticModel);

	rotateForceField = new RotateForceField(volumetric_mesh, Vector3d(-1, 0, 0));
	rotateForceField->centeraxis_position = Vector3d(0.5, 0, 0);
	rotateForceField->setForceMagnitude(0.001);

	initIntegrator();

	simulationstate = SimulationState::ready; 
	std::cout << "Simulator is ready." << std::endl;

	//std::ifstream test("extforce.txt");
	//int dimension;
	//test >> dimension;
	//VectorXd g(dimension);
	//for (int i = 0; i < dimension; i++)
	//{
	//	test >> g.data()[i];
	//}
	//isotropicHyperelasticModel->setGravityForce(g);

	//test
	/*Vegatest* vegatest = new Vegatest("./data/ElementMesh/staypuftveg.veg", 5000, 0.3, 1);
	srand(0);
	VectorXd randomdisplacement(R);
	randomdisplacement.setRandom();
	vegatest->test(randomdisplacement.data());
	VectorXd internalforce(R);
	internalforce.setZero();
	SparseMatrix<double>* stiffnessmatrix = new SparseMatrix<double>(R, R);
	forcemodel->getTangentStiffnessMatrixTopology(stiffnessmatrix);
	QElapsedTimer timer;
	timer.start();
	forcemodel->getForceAndMatrix(randomdisplacement.data(), internalforce.data(), stiffnessmatrix);
	std::cout << "my " << timer.nsecsElapsed()*1e-6 << std::endl;
	std::ofstream test("test2.txt");
	test << *stiffnessmatrix << std::endl;
	test.close();*/

}

void LoboIsotropicHyperlasticSimulator::updateSimulator(int verbose /*= 0*/)
{
	mergeExternalForce();
	integrator->doTimeStep();

	int numVertex = volumetric_mesh->getNumVertices();
	int R = numVertex * 3;
	VectorXd fullq(R);
	memcpy(fullq.data(), integrator->getq(), R*sizeof(double));
	updateTriAndVolumeMesh(fullq.data());


	this->simulation_steps++;
}

void LoboIsotropicHyperlasticSimulator::resetSimulator(int verbose /*= 0*/)
{
	runtimePoseDataSet->clearData();
	integrator->resetToRestNoClearSequence();
	simulation_steps = 0;
	nodetrajectory.clear();
}

LoboSimulatorBase::simulatorType LoboIsotropicHyperlasticSimulator::getType()
{
	return LoboSimulatorBase::ISOTROPICHYPERLASTIC;
}

void LoboIsotropicHyperlasticSimulator::saveSimulator(const char* filename, fileFormatType formattype) const
{

}

void LoboIsotropicHyperlasticSimulator::readSimulator(const char* filename, fileFormatType formattype)
{

}

void LoboIsotropicHyperlasticSimulator::readVolumetricMeshAscii(const char* filenamebase)
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
}

void LoboIsotropicHyperlasticSimulator::initSimulatorGlobalConstraints()
{
	if (constrainedDOFs != NULL)
	{
		free(constrainedDOFs);
	}
	int numVertex = volumetric_mesh->getNumVertices();

	std::vector<bool> nodemark(numVertex);
	std::fill(nodemark.begin(), nodemark.end(), false);

	for (int i = 0; i < selectedNodes.size(); i++)
	{
		int nodeid = selectedNodes[i];
		nodemark[nodeid] = true;
	}

	constrainedNodes.clear();
	for (int i = 0; i < numVertex; i++)
	{
		if (nodemark[i])
		{
			constrainedNodes.push_back(i);
		}
	}

	numConstrainedDOFs = constrainedNodes.size() * 3;
	constrainedDOFs = (int*)malloc(sizeof(int) * numConstrainedDOFs);
	for (int i = 0; i < numConstrainedDOFs / 3; i++)
	{
		constrainedDOFs[i * 3 + 0] = constrainedNodes[i] * 3 + 0;
		constrainedDOFs[i * 3 + 1] = constrainedNodes[i] * 3 + 1;
		constrainedDOFs[i * 3 + 2] = constrainedNodes[i] * 3 + 2;
	}

	std::cout << "created constrain: size " << numConstrainedDOFs << std::endl;
}

void LoboIsotropicHyperlasticSimulator::singleClickEvent()
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

void LoboIsotropicHyperlasticSimulator::rectSelectEvent()
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

void LoboIsotropicHyperlasticSimulator::mouseMoveEvent()
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

void LoboIsotropicHyperlasticSimulator::mouseReleaseEvent()
{
	this->cleanExternalForce();
}

void LoboIsotropicHyperlasticSimulator::initIntegrator()
{
	if (integrator != NULL)
	{
		delete integrator;
	}
	int r = volumetric_mesh->getNumVertices() * 3;
	integrator = new ImplicitNewMatkSparseIntegrator(r, timestep, massMatrix, forcemodel, 1, numConstrainedDOFs, constrainedDOFs, dampingMassCoef, dampingStiffnessCoef, newtonIteration);

	std::cout << "integrator_skip" << integrator_skip << std::endl;
	integrator->setSkipSteps(integrator_skip);
}

void LoboIsotropicHyperlasticSimulator::readConfig(std::ifstream &inStream)
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

		if (token == "constraints")
		{
			std::string filebase;
			inStream >> filebase;
			this->readConstraints(filebase.c_str());
		}

		if (token == "invertible")
		{
			setIsInvertible(true);
		}

		if (token == "Material")
		{
			int materialid;
			inStream >> materialid;
			switch (materialid)
			{
			case 0:
				setMaterialtype(NeoHookean);
				break;
			case 1:
				setMaterialtype(STVK);
				break;
			default:
				break;
			}
		}

	}
}

void LoboIsotropicHyperlasticSimulator::mergeExternalForce()
{
	//rotateForceField->computeCurExternalForce(externalForce);

	VectorXd mergedExternalForce = externalForce + mouseForce + collisionExternalForce;

	/*if (simulation_steps == 90)
	{
		isotropicHyperelasticModel->setAddGravity(false);
	}*/

	integrator->setExternalForces(mergedExternalForce.data());
}

LoboIsotropicHyperlasticSimulator* downCastIsotropicHyperElasticSimulator(LoboSimulatorBase* simulator)
{
	return (LoboIsotropicHyperlasticSimulator*)simulator;
}
