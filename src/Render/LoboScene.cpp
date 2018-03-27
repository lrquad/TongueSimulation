#include "LoboScene.h"
#include <iostream>
#include "LoboFunctions.h"
#include <fstream>
#include <cassert>
#include "Simulator/InvertibleQuadraticFrameSimulator.h"
#include "Simulator/STVKSimulator.h"
#include "Simulator/SubspaceSTVKSimulator.h"
#include "Simulator/DomainDecomReducedSimulator.h"
#include "Simulator/InvQuadMultiweightFrameSimulator.h"
#include "Simulator/RigidSimulator.h"
#include "Simulator/DualQuaternionsFramebased.h"
#include <string>
#include <iomanip>

LoboScene::LoboScene() :
simulation_step(0),
skip_steps(1),
when_nail_back_(-1)
{
	initCollisionWorld();
}

LoboScene::~LoboScene()
{
	lobo::deleteStdvectorPointer<LoboObjMesh*>(model_list_);
}

bool LoboScene::addObjModel(const char* filename)
{
	LoboObjMesh* render = new LoboObjMesh(filename);
	model_list_.push_back(render);

	return true;
}

void LoboScene::moveObjModel(double x, double y, double z)
{
	if (model_list_.size() != 0)
	{
		model_list_.back()->translateObjModel(x, y, z);
	}
	else
	{
		std::cout << "model is empty, add model first" << std::endl;
	}
}

void LoboScene::scaleObjModel(double scale)
{
	if (model_list_.size() != 0)
	{
		model_list_.back()->scaleObjModel(scale);
	}
	else
	{
		std::cout << "model is empty, add model first" << std::endl;
	}
}

void LoboScene::updateObjCollisionShape()
{
	for (int i = 0; i < model_list_.size(); i++)
	{
		model_list_[i]->updateCollisionShape();
	}
}

void LoboScene::simulate()
{
	if (!config.b_start_simulate)
	{
		return;
	}

	for (int i = 0; i < model_list_.size(); i++)
	{
		if (model_list_[i]->getMovingFlag())
		{
			if (simulation_step < when_nail_back_)
				model_list_[i]->objUniformMotion(Vector3d(0, 0, -0.0008)); else
			if (simulation_step >= when_nail_back_)
			{
				model_list_[i]->objUniformMotion(Vector3d(0, 0, 0.0008));
			}
		}
	}

	for (int i = 0; i < simulator_list_.size(); i++)
	{
		if (simulator_list_[i]->config_.b_simulate)
		{
			simulator_list_[i]->simulate();
			simulator_list_[i]->updateTriMesh();
			simulator_list_[i]->forceManager.update();
		}
	}

	checkCollision();

	if (config.b_save_animation)
	{
		saveAnimation(skip_steps);
	}
	simulation_step++;
}

void LoboScene::drawScene(QOpenGLShaderProgram *program)
{
	if (config.b_render_model)
	for (int i = 0; i < model_list_.size(); i++)
	{
		model_list_[i]->drawModel(program);
	}

	if (config.b_render_simulator)
	for (int i = 0; i < simulator_list_.size(); i++)
	{
		simulator_list_[i]->drawSimulator(program);
	}
}

void LoboScene::paintScene(QPainter &painter)
{
	for (int i = 0; i < simulator_list_.size(); i++)
	{
		simulator_list_[i]->paintSimulator(painter);
	}
}

void LoboScene::saveAnimation(int skipframe)
{
	if (simulation_step > config.number_of_frames_to_save)
	{
		config.b_start_simulate = false;
		return;
	}

	if (simulation_step%skipframe != 0)
	{
		return;
	}

	std::ostringstream oss;
	oss << "D:/DemoOutput/Animation/frame" << std::setfill('0') << std::setw(4) << simulation_step / skipframe << ".obj";
	std::string var = oss.str();
	std::ofstream outStream(var.c_str());
	outStream << "#This file created by lobosimulator." << std::endl;

	int offset = 1;
	int vtoffset = 1;
	for (int i = model_list_.size()-1; i >=0; i--)
	{
		if (model_list_[i]->getOutputFlag())
		{
			/*if (i == 0)
			{
				model_list_[i]->saveShape(outStream, offset, vtoffset,true);
			}else*/
			model_list_[i]->saveShape(outStream, offset, vtoffset);
		}
	}

	outStream.close();
}

void LoboScene::initCollisionWorld()
{
	btCollisionConfiguration* bt_collision_configuration;
	btCollisionDispatcher* bt_dispatcher;
	btBroadphaseInterface* bt_broadphase;
	double scene_size = 1000;
	unsigned int max_objects = 10000;
	bt_collision_configuration = new btDefaultCollisionConfiguration();
	bt_dispatcher = new btCollisionDispatcher(bt_collision_configuration);
	btGImpactCollisionAlgorithm::registerAlgorithm(bt_dispatcher);

	btScalar sscene_size = (btScalar)scene_size;
	btVector3 worldAabbMin(-sscene_size, -sscene_size, -sscene_size);
	btVector3 worldAabbMax(sscene_size, sscene_size, sscene_size);

	bt_broadphase = new bt32BitAxisSweep3(worldAabbMin, worldAabbMax, max_objects, 0, true);  // true for disabling raycast accelerator

	bt_collision_world = new btCollisionWorld(bt_dispatcher, bt_broadphase, bt_collision_configuration);

}

void LoboScene::checkCollision()
{
	//Create two collision objects
	for (int i = 0; i < model_list_.size(); i++)
	{
		if (model_list_[i]->getCollisionFlag())
		bt_collision_world->addCollisionObject(model_list_[i]->getTri_Mesh()->bt_collision_object);
	}

	bt_collision_world->performDiscreteCollisionDetection();

	int numManifolds = bt_collision_world->getDispatcher()->getNumManifolds();
	std::vector<Vector2i> collision_pair;
	for (int i = 0; i < numManifolds; i++) {
		btPersistentManifold* contactManifold = bt_collision_world->getDispatcher()->getManifoldByIndexInternal(i);
		btCollisionObject* obA = (btCollisionObject*)(contactManifold->getBody0());
		btCollisionObject* obB = (btCollisionObject*)(contactManifold->getBody1());
		Vector2i pair;
		for (int i = 0; i < model_list_.size(); i++)
		{
			if (model_list_[i]->getTri_Mesh()->bt_collision_object == obA)
			{
				model_list_[i]->setAABBupdateFlag(true);
				pair.data()[0] = i;
			}

			if (model_list_[i]->getTri_Mesh()->bt_collision_object == obB)
			{
				model_list_[i]->setAABBupdateFlag(true);
				pair.data()[1] = i;
			}
		}
		collision_pair.push_back(pair);
	}

	for (int i = 0; i < model_list_.size(); i++)
	{
		model_list_[i]->getTri_Mesh()->tri_mesh_force.clear();
		model_list_[i]->updateAABBtree();
		model_list_[i]->selfCollisionDetection();

		if (model_list_[i]->getCollisionFlag())
		bt_collision_world->removeCollisionObject(model_list_[i]->getTri_Mesh()->bt_collision_object);
	}
	
	//COLDET 3D lib to detect tri face tri.
	for (int i = 0; i < collision_pair.size(); i++)
	{
		int mode0 = collision_pair[i].data()[0];
		int mode1 = collision_pair[i].data()[1];
		checkCollisionTT(model_list_[mode0], model_list_[mode1]);
	}

}

void LoboScene::checkCollisionTT(LoboObjMesh *mode0, LoboObjMesh *mode1)
{
	
	if (mode0->getTri_Mesh()->coldet_model->collision(mode1->getTri_Mesh()->coldet_model))
	{
		int triid1, triid2;
		mode0->getTri_Mesh()->coldet_model->getCollidingTriangleList(
			mode0->getTri_Mesh()->coollide_tri_list,
			mode1->getTri_Mesh()->coollide_tri_list
			);

		assert(mode0->getTri_Mesh()->coollide_tri_list.size() == mode1->getTri_Mesh()->coollide_tri_list.size());

		for (int i = 0; i < mode0->getTri_Mesh()->coollide_tri_list.size(); i++)
		{
			int tri1 = mode0->getTri_Mesh()->coollide_tri_list[i];
			int tri2 = mode1->getTri_Mesh()->coollide_tri_list[i];
			int hack_force_direction = false;

			if (simulation_step < when_nail_back_)
			{
				hack_force_direction = 0;
			}
			else if (simulation_step >= when_nail_back_)
			{
				hack_force_direction = 1;
			}
			else
			{
				hack_force_direction = 2;
			}

			bool if_hack = true;
			if (when_nail_back_ == -1)
			{
				if_hack = false;
			}

			computeIntersectionForce(tri1, mode0->getTri_Mesh(), tri2, mode1->getTri_Mesh(), hack_force_direction, if_hack);
		}

	}
	else
	{
		mode0->getTri_Mesh()->coollide_tri_list.clear();
		mode1->getTri_Mesh()->coollide_tri_list.clear();
	}
}

void LoboScene::mousePressEvent(QMouseEvent *mouse_event)
{
	for (int i = 0; i < simulator_list_.size(); i++)
	{
		simulator_list_[i]->mousePressEvent(mouse_event);
	}
}

void LoboScene::mouseMoveEvent(QMouseEvent *mouse_event)
{
	for (int i = 0; i < simulator_list_.size(); i++)
	{
		simulator_list_[i]->mouseMoveEvnet(mouse_event);
	}
}

void LoboScene::mouseReleaseEvent(QMouseEvent *mouse_event)
{
	for (int i = 0; i < simulator_list_.size(); i++)
	{
		simulator_list_[i]->mouseReleaseEvnet(mouse_event);
	}
}

void LoboScene::keyPressEvent(QKeyEvent *key_event)
{
	for (int i = 0; i < simulator_list_.size(); i++)
	{
		simulator_list_[i]->keyPressEvent(key_event);
	}
}

void LoboScene::keyReleaseEvent(QKeyEvent *key_event)
{
	for (int i = 0; i < simulator_list_.size(); i++)
	{
		simulator_list_[i]->keyReleaseEvent(key_event);
	}
}

void LoboScene::setMouseStruct(lobo::MouseDescribe select)
{
	for (int i = 0; i < simulator_list_.size(); i++)
	{
		simulator_list_[i]->setMouseStruct(select);
	}
}

void LoboScene::setMVPMatrix(Matrix4d &MVP)
{
	for (int i = 0; i < simulator_list_.size(); i++)
	{
		simulator_list_[i]->setMVPMatrix(MVP);
	}
}

void LoboScene::reciveSelectedInfo(lobo::MouseDescribe select)
{
	for (int i = 0; i < simulator_list_.size(); i++)
	{
		simulator_list_[i]->reciveSelectedInfo(select);
	}
}

void LoboScene::readConfig(const char* filename)
{
	std::ifstream inStream(filename);
	std::string token;
	while (true)
	{
		inStream >> token;
		if (inStream.eof())
		{
			break;
		}

		if (token[0] == '#')
		{
			inStream.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
		}

		if (token == "readobj")
		{
			std::string filename;
			inStream >> filename;
			this->addObjModel(filename.c_str());
		}

		if (token == "animation_skip_steps")
		{
			inStream >> skip_steps;
		}

		if (token == "when_nail_back_")
		{
			inStream >> when_nail_back_;
		}

		if (token == "scale")
		{
			double scale;
			inStream >> scale;
			this->scaleObjModel(scale);
		}

		if (token == "moveobj")
		{
			double x, y, z;
			inStream >> x >> y >> z;
			this->moveObjModel(x, y, z);
		}

		if (token == "collision")
		{
			int b;
			inStream >> b;
			model_list_.back()->setCollisionFlag(b);
		}
		if (token == "funnel_radius")
		{
			double funnel_radius;
			inStream >> funnel_radius;
			model_list_.back()->funnel_radius = funnel_radius;
		}

		if (token == "moving_obj")
		{
			bool moving;
			inStream >> moving;
			model_list_.back()->setMovingFlag(moving);
		}

		if (token == "selfcollision")
		{
			int b;
			inStream >> b;
			model_list_.back()->setSelfCollisionFlag(b);
		}

		if (token == "selfimpactratio")
		{
			double ratio;
			inStream >> ratio;
			model_list_.back()->getTri_Mesh()->selfimpactratio = ratio;
		}

		if (token == "impactratio")
		{
			double ratio;
			inStream >> ratio;
			model_list_.back()->getTri_Mesh()->impactratio = ratio;
		}

		if (token == "dampimpact")
		{
			double ratio;
			inStream >> ratio;
			model_list_.back()->getTri_Mesh()->dampimpact = ratio;
		}

		if (token == "frictionratio")
		{
			double ratio;
			inStream >> ratio;
			model_list_.back()->getTri_Mesh()->frictionratio = ratio;
		}

		if (token == "outputmesh")
		{
			bool b;
			inStream >> b;
			model_list_.back()->setOutputFlag(b);
		}

		if (token == "collision_render")
		{
			int b;
			inStream >> b;
			for (int i = 0; i < model_list_.size(); i++)
				model_list_[i]->setCollisionRender(b);
		}

		if (token == "saveimage")
		{
			int b;
			inStream >> b;
			this->config.b_save_screen_image = b;
		}

		if (token == "saveanimation")
		{
			inStream >> config.b_save_animation;
			inStream >> config.number_of_frames_to_save;
		}

		if (token == "rendermode")
		{
			std::string mode;
			inStream >> mode;
			if (mode == "wireframe")
			{
				config.rendermode = wireframe;
			}else
			if (mode == "fill")
			{
				config.rendermode = fill;
			}
			else
			if (mode == "point")
			{
				config.rendermode = point;
			}
		}

		if (token == "render_model")
		{
			int turnoff;
			inStream >> turnoff;
			config.b_render_model = turnoff;
		}

		if (token == "render_simulator")
		{
			int turnoff;
			inStream >> turnoff;
			config.b_render_simulator = turnoff;
		}

		if (token == "quadframeSim")
		{
			int frame_type;

			inStream >> frame_type;
			std::cout << std::endl;
			std::cout << "create quadratic frame based simulator ... "<<std::endl;
			InvertibleQuadraticFrameSimulator *qs = new InvertibleQuadraticFrameSimulator(frame_type);
			qs->readConfig(inStream,this);
			this->simulator_list_.push_back(qs);
		}

		if (token == "multiweightSim")
		{
			int frame_type;
			std::cout << "create multi weight quadratic frame based simulator ... " << std::endl;
			std::cout << std::endl;
			InvQuadMultiweightFrameSimulator *iqmws = new InvQuadMultiweightFrameSimulator();
			iqmws->readConfig(inStream, this);
			this->simulator_list_.push_back(iqmws);
		}

		if (token == "STVKSim")
		{
			std::cout << std::endl;
			std::cout << "create STVK FEM simulator ... " << std::endl;
			STVKSimulator *ss = new STVKSimulator();
			ss->readConfig(inStream, this);
			this->simulator_list_.push_back(ss);
		}

		if (token == "SubspaceSTVKSim")
		{
			std::cout << std::endl;
			std::cout << "create subspace STVK FEM simulator ... " << std::endl;
			SubspaceSTVKSimulator *sss = new SubspaceSTVKSimulator();
			sss->readConfig(inStream, this);
			this->simulator_list_.push_back(sss);
		}

		if (token == "domainDecSim")
		{
			std::cout << std::endl;
			std::cout << "create domain decomposition tool and simulator ... " << std::endl;
			DomainDecomReducedSimulator* ddrs = new DomainDecomReducedSimulator();
			ddrs->readConfig(inStream, this);
			this->simulator_list_.push_back(ddrs);
		}

		if (token == "rigidSim")
		{
			std::cout << std::endl;
			std::cout << "create rigid body ..." << std::endl;
			RigidSimulator *rs = new RigidSimulator();
			rs->readConfig(inStream, this);
			this->simulator_list_.push_back(rs);
		}

		if (token == "qualframeSim")
		{
			std::cout << std::endl;
			std::cout << "create qualframeSim ..." << std::endl;
			DualQuaternionsFramebased *rs = new DualQuaternionsFramebased();
			rs->readConfig(inStream, this);
			this->simulator_list_.push_back(rs);
		}

		if (token == "start")
		{
			int simulatorid;
			inStream >> simulatorid;
			if (simulatorid == -1)
			{
				for (int i = 0; i < simulator_list_.size(); i++)
				{
					simulator_list_[i]->init();
					simulator_list_[i]->preCompute();
					simulator_list_[i]->config_.b_simulate = true;
				}
			}
			else
			{
				assert(simulatorid >= 0 && simulatorid < simulator_list_.size());
				simulator_list_[simulatorid]->init();
				simulator_list_[simulatorid]->preCompute();
				simulator_list_[simulatorid]->config_.b_simulate = true;
			}
			config.b_start_simulate = true;
		}
		
	}

}