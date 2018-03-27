#include "FloatingMDSTVKSimulator.h"
#include "LoboVolumetricMesh/LoboVolumetriceMeshCore.h"
#include "Render/MouseProjection.h"
#include "Functions/findElementInVector.h"
#include "Simulator/STVK/STVKModel.h"
#include "Simulator/ForceModel/STVKForceModel.h"
#include "Simulator/ReducedSTVK/ReducedSTVKModel.h"
#include "Simulator/ReducedForceModel/ReducedSTVKForceModel.h"
#include "Reduced/Modalderivtives/SubspaceModalDerivativesModule.h"
#include "Simulator/PoseDataSet/PoseDataSet.h"
#include "Simulator/ReducedSTVK/ReducedSTVKModelCubature.h"
#include "Integrator/ImpicitNewMarkDenseIntegrator.h"
#include "Integrator/ImplicitNMDenseModifiedIntegrator.h"
#include <fstream>

FloatingMDSTVKSimulator::FloatingMDSTVKSimulator(std::ifstream &readconfig, LoboTriMesh* obj_mesh, bool ifreadconfig) :LoboSimulatorBase(readconfig, obj_mesh)
{
	nullAllPointer();
	if (ifreadconfig)
	{
		readConfig(readconfig);
		if (volumetric_mesh != NULL)
		{
			selectedNodes.reserve(volumetric_mesh->getNumVertices());
		}

	}
}

FloatingMDSTVKSimulator::~FloatingMDSTVKSimulator()
{
	deleteAllPointer();
}

void FloatingMDSTVKSimulator::nullAllPointer()
{
	subspaceModule = NULL;
	stvkmodel = NULL;
	massMatrix = NULL;
	stvkforce_model = NULL;
	modesFileName = "";
	reduced_massMatrix = new MatrixXd();
	reducedSTVKModel = NULL;
	reducedforcemodel = NULL;
	subspaceModule = new SubspaceModalDerivativesModule();
	

	setIsInvertible(false);
}

void FloatingMDSTVKSimulator::deleteAllPointer()
{
	delete subspaceModule;
	delete stvkmodel;
	delete massMatrix;
	delete stvkforce_model;
	delete reduced_massMatrix;
	delete reducedSTVKModel;
	delete reducedforcemodel;
}

void FloatingMDSTVKSimulator::initSimulator(int verbose /*= 0*/)
{
	int numVertex = volumetric_mesh->getNumVertices();
	int R = numVertex * 3;
	runtimePoseDataSet = new PoseDataSet(numVertex * 3);

	externalForce.resize(numVertex * 3);
	externalForce.setZero();

	mouseForce.resize(numVertex * 3);
	mouseForce.setZero();

	collisionExternalForce.resize(numVertex * 3);
	collisionExternalForce.setZero();

	((TetVolumetricMesh*)volumetric_mesh)->initElementShapeFunctionDerivate();

	volumetric_mesh_graph = new LoboVolumetricMeshGraph(volumetric_mesh);
	volumetric_mesh_graph->init();

	initVolumnMeshTriMeshAdapter();

	 
	*reduced_massMatrix = subspaceModule->phi->transpose()*(*massMatrix)*(*subspaceModule->phi);

	//init SimulatorModle
	if (stvkmodel == NULL)
	{
		stvkmodel = new STVKModel(volumetric_mesh, massMatrix);
	}
	reducedSTVKModel = new ReducedSTVKModelCubature(volumetric_mesh,stvkmodel,massMatrix,subspaceModule->phi);

	int r = subspaceModule->r;
	VectorXd reducedgraivty(r);
	reducedSTVKModel->computeGravity(&reducedgraivty, massMatrix, R);
	reducedSTVKModel->setGravityForce(reducedgraivty.data());
	reducedSTVKModel->setGravity(this->getUseGravity());

	reducedforcemodel = new ReducedSTVKForceModel(reducedSTVKModel);

	initIntegrator();
	simulationstate = SimulationState::ready;
	std::cout << "Simulator is ready." << std::endl;
}

void FloatingMDSTVKSimulator::updateSimulator(int verbose /*= 0*/)
{
	mergeExternalForce();
	integrator->doTimeStep();
	int r = subspaceModule->r;
	VectorXd reducedq(r);
	memcpy(reducedq.data(), integrator->getq(), r*sizeof(double));
	VectorXd fullq = (*subspaceModule->phi)*reducedq;
	
	updateTriAndVolumeMesh(fullq.data());
	this->simulation_steps++;
}

void FloatingMDSTVKSimulator::resetSimulator(int verbose /*= 0*/)
{
	runtimePoseDataSet->clearData();
	integrator->resetToRestNoClearSequence();
	simulation_steps = 0;
}

LoboSimulatorBase::simulatorType FloatingMDSTVKSimulator::getType()
{
	return LoboSimulatorBase::FLOATINGMD;
}

void FloatingMDSTVKSimulator::saveSimulator(const char* filename, fileFormatType formattype) const
{

}

void FloatingMDSTVKSimulator::readSimulator(const char* filename, fileFormatType formattype)
{

}

void FloatingMDSTVKSimulator::readVolumetricMeshAscii(const char* filenamebase)
{
	TetVolumetricMesh* tet_volumetric_mesh = new TetVolumetricMesh();

	Vector3d translate = Vector3d::Zero();

	tet_volumetric_mesh->readElementMesh(filenamebase, simulator_translate.data(), getSimulator_scale());

	volumetric_mesh = tet_volumetric_mesh;

	LoboVolumetricMesh::Material* materia = volumetric_mesh->getMaterialById(0);
	LoboVolumetricMesh::ENuMaterial* enmateria = (LoboVolumetricMesh::ENuMaterial*)materia;
	enmateria->setE(this->youngmodulus);
	enmateria->setNu(this->poisson);
	enmateria->setDensity(this->density);

	
}

void FloatingMDSTVKSimulator::initSimulatorGlobalConstraints()
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

void FloatingMDSTVKSimulator::initSubspaceModes()
{
	//init stvk and stvk force model first
	stvkmodel = new STVKModel(volumetric_mesh, massMatrix);

	//compute pure internal force, becuase vega's style combined the gravity into internal force
	stvkmodel->setGravity(false);

	stvkforce_model = new STVKForceModel(stvkmodel);


	//use default k = 10 r = 20
	subspaceModule->configModule(stvkmodel, stvkforce_model, massMatrix, numConstrainedDOFs, constrainedDOFs);
}

void FloatingMDSTVKSimulator::generateModes()
{
	subspaceModule->generateSubspace();
}

void FloatingMDSTVKSimulator::singleClickEvent()
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

void FloatingMDSTVKSimulator::rectSelectEvent()
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

void FloatingMDSTVKSimulator::mouseMoveEvent()
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

void FloatingMDSTVKSimulator::mouseReleaseEvent()
{
	this->cleanExternalForce();
}

void FloatingMDSTVKSimulator::initSparseMassMatrix()
{
	if (massMatrix != NULL)
	{
		return;
	}

	int R = volumetric_mesh->getNumVertices() * 3;
	massMatrix = new SparseMatrix<double>(R, R);
	volumetric_mesh->computeMassMatrix(massMatrix);
}

void FloatingMDSTVKSimulator::initIntegrator()
{
	if (integrator != NULL)
	{
		delete integrator;
	}

	integrator = new ImplicitNMDenseModifiedIntegrator(subspaceModule->r, timestep, reduced_massMatrix, reducedforcemodel, 0, 0, dampingMassCoef, dampingStiffnessCoef, this->newtonIteration);
	integrator->setSkipSteps(integrator_skip);

}

void FloatingMDSTVKSimulator::readConfig(std::ifstream &inStream)
{
	std::string token;
	while (true)
	{
		inStream >> token;
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
			initSparseMassMatrix();
		}

		if (token == "constraints")
		{
			std::string filebase;
			inStream >> filebase;
			this->readConstraints(filebase.c_str());
		}

		if (token == "subspaceMatrix")
		{
			inStream >> modesFileName;
			this->getSubspaceModuleRef()->loadSubspace(modesFileName.c_str());
		}

		if (token == "invertible")
		{
			setIsInvertible(true);
		}

	}

}

void FloatingMDSTVKSimulator::mergeExternalForce()
{
	MatrixXd *phi = subspaceModule->phi;
	VectorXd mergedExternalForce = externalForce + mouseForce + collisionExternalForce;
	VectorXd reducedexternalforce = phi->transpose()*mergedExternalForce;
	integrator->setExternalForces(reducedexternalforce.data());
}

FloatingMDSTVKSimulator* downCastReducedFloatingSTVKSimulator(LoboSimulatorBase* simulator)
{
	return (FloatingMDSTVKSimulator*)simulator;
}
