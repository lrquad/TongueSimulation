#include "ReducedTetSTVKSimulator.h"
#include "LoboVolumetricMesh/TetVolumetricMesh.h"
#include "Integrator/ImplicitNewMatkSparseIntegrator.h"
#include "Integrator/ImpicitNewMarkDenseIntegrator.h"
#include <iostream>
#include <fstream>
#include "Reduced/SubspaceModesCreator.h"
#include "Simulator/ForceModel/STVKForceModel.h"
#include "Simulator/STVK/STVKModel.h"
#include "Simulator/ReducedSTVK/ReducedSTVKModel.h"
#include "Simulator/ReducedForceModel/ReducedForceModel.h"
#include "Simulator/ReducedForceModel/ReducedSTVKForceModel.h"
#include "Render/MouseProjection.h"
#include "EigenMatrixIO/EigenMatrixIO.h"
#include "Simulator/PoseDataSet/PoseDataSet.h"
#include "Functions/findElementInVector.h"
#include "LoboVolumetricMesh/LoboVolumetriceMeshCore.h"

ReducedTetSTVKSimulator::ReducedTetSTVKSimulator(std::ifstream &readconfig, LoboTriMesh* obj_mesh) :LoboSimulatorBase(readconfig, obj_mesh)
{
	initPointerToNULL();
	modesk = 5;
	modesr = 15;
	numConstrainedDOFs = 0;
	savePoseSkipFrame = 0;
	constrainedDOFs = NULL;
	volumetric_obj_adapter = NULL;
	savePoseSequenceRealTime = false;
	this->readConfig(readconfig);
}

ReducedTetSTVKSimulator::ReducedTetSTVKSimulator(LoboVolumetricMesh* volumetricmesh, std::ifstream &readconfig, LoboTriMesh* obj_mesh) :
LoboSimulatorBase(volumetricmesh, readconfig, obj_mesh)
{
	initPointerToNULL();
	modesk = 5;
	modesr = 15;
	numConstrainedDOFs = 0;
	savePoseSkipFrame = 0;
	constrainedDOFs = NULL;
	volumetric_obj_adapter = NULL;
	savePoseSequenceRealTime = false;
	this->readConfig(readconfig);
}

ReducedTetSTVKSimulator::~ReducedTetSTVKSimulator()
{
	deletePointers();
	free(constrainedDOFs);
	delete volumetric_obj_adapter;
}

void ReducedTetSTVKSimulator::initSimulatorModel()
{
	if (reducedSTVKmodel!=NULL)
	delete reducedSTVKmodel;

	std::cout << "init reduced STVK Model ." << std::endl;
	reducedSTVKmodel = new ReducedSTVKModel(this->volumetric_mesh, massMatrix, &reduced_modes);
}

void ReducedTetSTVKSimulator::computeModesCoeff()
{
	reducedSTVKmodel->computeReducedModesCoefficients();
}

void ReducedTetSTVKSimulator::readModeCoeff(const char* filename)
{
	reducedSTVKmodel->loadCoefMatrix(filename);
}

void ReducedTetSTVKSimulator::saveModeCoeff(const char* filename)
{
	reducedSTVKmodel->saveCoefMatrix(filename);
}

void ReducedTetSTVKSimulator::initSimulator(int verbose)
{
	int numVertex = volumetric_mesh->getNumVertices();
	externalForce.resize(numVertex * 3);

	((TetVolumetricMesh*)volumetric_mesh)->initElementShapeFunctionDerivate();

	volumetric_obj_adapter = new LoboVolumetricMeshTriMeshAdapter(volumetric_mesh, tri_mesh);
	volumetric_obj_adapter->computeNodeBarycentricWeight();

	int R = volumetric_mesh->getNumVertices() * 3;
	fulldisplacement.resize(R);
	fulldisplacement.setZero();
	runtimePoseDataSet = new PoseDataSet(R);

	reducedMassMatrix = new MatrixXd();
	*reducedMassMatrix = reduced_modes.transpose()*(*massMatrix)*reduced_modes;

	VectorXd reducedgraivty(r);
	reducedSTVKmodel->computeGravity(&reducedgraivty, massMatrix, R);
	reducedSTVKmodel->setGravityForce(reducedgraivty.data());
	reducedSTVKmodel->setGravity(true);

	reducedforcemodel = new ReducedSTVKForceModel(reducedSTVKmodel);

	initIntegrator();
	simulationstate = SimulationState::ready;
}

void ReducedTetSTVKSimulator::initSubspaceModes()
{
	stvkmodel = new STVKModel(this->volumetric_mesh, massMatrix);
	stvkmodel->setGravity(false);

	stvkforce_model = new STVKForceModel(stvkmodel);

	SubspaceModesCreator* modesCreater = new SubspaceModesCreator(stvkforce_model, massMatrix, numConstrainedDOFs, constrainedDOFs, modesk, modesr);
	//modesCreater->generateLinearModes();
	modesCreater->generateModes();
	//std::cout << "just export LMA!" << std::endl;

	//reduced_modes = *(modesCreater->getFinalModes());
	reduced_modes = *(modesCreater->getFinalModes());

	r = reduced_modes.cols();
}

void ReducedTetSTVKSimulator::computeSparseMassMatrix()
{
	if (massMatrix != NULL)
	{
		return;
	}

	int R = volumetric_mesh->getNumVertices() * 3;
	massMatrix = new SparseMatrix<double>(R, R);
	volumetric_mesh->computeMassMatrix(massMatrix);

}

void ReducedTetSTVKSimulator::saveSubspaceModes(const char* filename)
{
	EigenMatrixIO::write_binary(filename, reduced_modes);
}

void ReducedTetSTVKSimulator::readSubspaceModes(const char* filename)
{
	EigenMatrixIO::read_binary(filename, reduced_modes);
	r = reduced_modes.cols();
	std::cout << r << std::endl;
}

void ReducedTetSTVKSimulator::saveSubspaceModesAscii(const char* filename)
{
	std::ofstream outputstream(filename);
	outputstream << reduced_modes << std::endl;
	outputstream.close();
}

void ReducedTetSTVKSimulator::updateSimulator(int verbose)
{
	//testcollisionforce();
	integrator->doTimeStep();

	VectorXd reducedq(r);
	memcpy(reducedq.data(), integrator->getq(), r*sizeof(double));

	fulldisplacement = reduced_modes*reducedq;
	volumetric_mesh->setDisplacement(fulldisplacement.data());
	volumetric_obj_adapter->updateObjMesh();

	if (savePoseSequenceRealTime)
	{
		if (simulation_steps%(savePoseSkipFrame+1) == 0)
		{
			this->insertCurrentPose();
		}
	}

	simulation_steps++;
}

void ReducedTetSTVKSimulator::resetSimulator(int verbose)
{

}

LoboSimulatorBase::simulatorType ReducedTetSTVKSimulator::getType()
{
	return LoboSimulatorBase::simulatorType::REDUCEDSTVK;
}

void ReducedTetSTVKSimulator::saveSimulator(const char* filename, fileFormatType formattype) const
{

}

void ReducedTetSTVKSimulator::readSimulator(const char* filename, fileFormatType formattype)
{

}

void ReducedTetSTVKSimulator::createIntegrator()
{

}

void ReducedTetSTVKSimulator::testcollisionforce()
{
	int numVertex = volumetric_mesh->getNumVertices();
	VectorXd externalforce(numVertex * 3);
	externalforce.setZero();

	VectorXd nodevel = (reduced_modes)*integrator->getVectorq_vel();
	for (int i = 0; i < numVertex; i++)
	{
		Vector3d position = volumetric_mesh->getNodeDisplacement(i);
		if (position.y() < -1)
		{
			externalforce.data()[i * 3 + 1] = -10.0*(position.y() + 1) - 1 * nodevel.data()[i * 3 + 1];
		}
	}

	VectorXd reducedexternalforce = reduced_modes.transpose()*externalforce;
	integrator->setExternalForces(reducedexternalforce.data());
}

void ReducedTetSTVKSimulator::initPointerToNULL()
{
	massMatrix = NULL;
	reducedMassMatrix = NULL;
	stvkforce_model = NULL;
	reducedSTVKmodel = NULL;
	stvkmodel = NULL;
	reducedforcemodel = NULL;
}

void ReducedTetSTVKSimulator::deletePointers()
{
	delete massMatrix;
	delete reducedMassMatrix;
	delete stvkforce_model;
	delete reducedSTVKmodel;
	delete stvkmodel;
	delete reducedforcemodel;
}

void ReducedTetSTVKSimulator::initIntegrator()
{
	if (integrator != NULL)
	{
		delete integrator;
	}
	integrator = new ImpicitNewMarkDenseIntegrator(r, timestep, reducedMassMatrix, reducedforcemodel, 0, 0, dampingMassCoef, dampingStiffnessCoef, 1);
}

void ReducedTetSTVKSimulator::readConfig(std::ifstream &inStream)
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
			computeSparseMassMatrix();
		}

		//modes
		if (token == "readmodes")
		{
			std::string filename;
			inStream >> filename;
			this->readSubspaceModes(filename.c_str());
		}

		if (token == "readmodescoeff")
		{
			std::string filename;
			inStream >> filename;
			this->initSimulatorModel();
			this->readModeCoeff(filename.c_str());
		}

		if (token == "constraints")
		{
			std::string filebase;
			inStream >> filebase;
			this->readConstraints(filebase.c_str());
		}
	}

}

void ReducedTetSTVKSimulator::readVolumetricMeshAscii(const char* filenamebase)
{
	TetVolumetricMesh* tet_volumetric_mesh = new TetVolumetricMesh();

	tet_volumetric_mesh->setUniformMeshAfterLoad(getUniformVolumtricMesh());
	tet_volumetric_mesh->setUnifromUseScaleEx(false);

	tet_volumetric_mesh->readElementMesh(filenamebase, simulator_translate.data(), simulator_scale);

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

void ReducedTetSTVKSimulator::initSimulatorGlobalConstraints()
{
	if (constrainedDOFs != NULL)
	{
		free(constrainedDOFs);
	}

	constrainedNodes.clear();
	for (int i = 0; i < selectedNodes.size(); i++)
	{
		int nodeid = selectedNodes[i];
		constrainedNodes.push_back(nodeid);
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

void ReducedTetSTVKSimulator::insertCurrentPose()
{
	VectorXd reducedq(r);
	memcpy(reducedq.data(), integrator->getq(), r*sizeof(double));

	fulldisplacement = reduced_modes* reducedq;
	this->runtimePoseDataSet->insertNewPose(fulldisplacement.data(),1);

}

void ReducedTetSTVKSimulator::recordExternalForce()
{
	computeExternalForce(externalForce.data());
	externalforceSequence.push_back(externalForce);
}

void ReducedTetSTVKSimulator::computeFromExternalForce()
{
	if (simulation_steps < externalforceSequence.size())
	{
		externalForce = externalforceSequence[simulation_steps];
		VectorXd reducedexternalforce = reduced_modes.transpose()*externalForce;
		integrator->setExternalForces(reducedexternalforce.data());
	}
}

void ReducedTetSTVKSimulator::rectSelectEvent()
{
	selectedNodes.clear();
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

void ReducedTetSTVKSimulator::singleClickEvent()
{
	int numVertex = volumetric_mesh->getNumVertices();
	Vector2d mousescreen;
	mousescreen.x() = mouseprojection->getPreMouseX();
	mousescreen.y() = mouseprojection->getPreMouseY();
	double mindistance = DBL_MAX;
	double mindepth = DBL_MAX;
	int nodeid = -1;
	for (int i = 0; i < numVertex; i++)
	{
		Vector3d nodecur = volumetric_mesh->getNodePosition(i);
		Vector2d screenpos;
		double depth;
		mouseprojection->projectWorldToScreen(nodecur, screenpos, depth);
		double distance = (mousescreen - screenpos).norm();
		if (distance < mindistance)
		{
			mindistance = distance;
			nodeid = i;
			if (depth < mindepth)
			{
				mindepth = depth;
				nodeid = i;
			}
		}
	}
	mouseprojection->setBindedNodeid(nodeid);

	if (!findElement(loaded_nodeid, nodeid))
	{
		this->loaded_nodeid.push_back(nodeid);
		this->force_on_node.push_back(Vector3d::Zero());
	}
	currentforceid = loaded_nodeid.size() - 1;
}

void ReducedTetSTVKSimulator::mouseMoveEvent()
{
	if (simulationstate != LoboSimulatorBase::start)
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

	
	externalForce.setZero();

	Vector3d mouseWorld;
	mouseprojection->projectScreenToWorld(mousescreen, depth, mouseWorld);
	Vector3d force = (mouseWorld - nodecur) * mouseForceRatio;
	force_on_node[currentforceid] = force;

	computeExternalForce(externalForce.data());

	VectorXd reducedexternalforce = reduced_modes.transpose()*externalForce;

	integrator->setExternalForces(reducedexternalforce.data());
}

int ReducedTetSTVKSimulator::setShowModes(int modesid)
{
	volumetric_mesh->setDisplacement(reduced_modes.col(modesid).data());
	if (volumetric_obj_adapter!=NULL)
	this->volumetric_obj_adapter->updateObjMesh();
	return 0;
}

void ReducedTetSTVKSimulator::scaleShowedModes(int modesid, double scale)
{
	VectorXd mode = reduced_modes.col(modesid)*scale;
	volumetric_mesh->setDisplacement(mode.data());
	if (volumetric_obj_adapter != NULL)
		this->volumetric_obj_adapter->updateObjMesh();
}

ReducedTetSTVKSimulator* downCastReducedTetSTVKSimulator(LoboSimulatorBase* simulator)
{
	if (simulator->getType() == LoboSimulatorBase::REDUCEDSTVK)
	{
		return (ReducedTetSTVKSimulator*)simulator;
	}
	else
	{
		std::cout << "simulator type not match reduced stvk model" << std::endl;
		return NULL;
	}
}
