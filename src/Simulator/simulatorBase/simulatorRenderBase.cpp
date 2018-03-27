#include "simulatorRenderBase.h"
#include "LoboVolumetricMesh/VolumetricMeshRender.h"
#include "ColorTable/Colortable.h"
#include "Render/MouseProjection.h"
#include <iostream>

SimulatorRenderBase::SimulatorRenderBase(LoboSimulatorBase* simulator_)
{
	this->simulator = simulator_;
	volumetricMeshRender = new VolumetricMeshRender();
	renderselected = true;
	renderConstrained = false;
	b_render_volume_mesh = false;
	b_render_nodes = false;
	sphere1 = new SphereRender("3dmodel/sphere.obj");
	spherelow = new SphereRender("3dmodel/spherelow.obj");
	mesh_color_index = 3;
}


SimulatorRenderBase::~SimulatorRenderBase()
{
	delete volumetricMeshRender;
}

void SimulatorRenderBase::renderSimulator(QOpenGLShaderProgram *program)
{
	if (simulator->getVolumetricMesh() == NULL)
	{
		return;
	}
	volumetricMeshRender->renderVolumetricMesh(program, simulator->getVolumetricMesh());
}

void SimulatorRenderBase::paintSimulator(QPainter &painter)
{

}

void SimulatorRenderBase::saveRenderInfo(const char* filename, fileFormatType filetype) const
{

}

void SimulatorRenderBase::drawForce(QOpenGLShaderProgram* program)
{
	if (simulator->getSimulatorState() == LoboSimulatorBase::beforeinit)
	{
		return;
	}
	int nodeid = simulator->getMouseInfo()->getBindedNodeid();
	if (nodeid == -1)
	{
		return;
	}

	int numVertex = simulator->getVolumetricMesh()->getNumVertices();
	double* externalforce = simulator->getMouseExternalForce();


	Vector3d position = simulator->getVolumetricMesh()->getNodePosition(nodeid);
	Vector3d nodeforce;
	nodeforce.data()[0] = externalforce[nodeid * 3 + 0];
	nodeforce.data()[1] = externalforce[nodeid * 3 + 1];
	nodeforce.data()[2] = externalforce[nodeid * 3 + 2];

	Vector3d force = position + 1.0 / simulator->getMouseForceRatio()*nodeforce;
	drawArrow(program, position, force);

	if (nodeforce.norm()>1e-8)
	sphere1->drawMesh(program, force, 0.05, 0);

	/*for (int i = 0; i < numVertex; i++)
	{

	}*/
}

void SimulatorRenderBase::drawAllExForce(QOpenGLShaderProgram* program)
{
	if (simulator->getSimulatorState() == LoboSimulatorBase::beforeinit)
	{
		return;
	}
	int numVertex = simulator->getVolumetricMesh()->getNumVertices();
	double* externalforce = simulator->getIntegratorExternalForce();

	for (int i = 0; i < numVertex; i++)
	{
		Vector3d position = simulator->getVolumetricMesh()->getNodePosition(i);
		Vector3d nodeforce;
		nodeforce.data()[0] = externalforce[i * 3 + 0];
		nodeforce.data()[1] = externalforce[i * 3 + 1];
		nodeforce.data()[2] = externalforce[i * 3 + 2];

		Vector3d force = position + 1.0 / simulator->getMouseForceRatio()*nodeforce;

		drawArrow(program, position, force);
	}
}

void SimulatorRenderBase::drawArrow(QOpenGLShaderProgram* program, Vector3d start, Vector3d end, int colorindex /*= 0*/)
{
	int DiffuseProduct_loc = program->uniformLocation("DiffuseProduct");
	int BoolPoint_loc = program->uniformLocation("point");
	program->setUniformValue(BoolPoint_loc, 1);
	glLineWidth(5);
	double defaultcolor[3];
	getColor(colorindex, defaultcolor);
	QVector4D color = QVector4D(defaultcolor[0], defaultcolor[1], defaultcolor[2], 0.5);
	program->setUniformValue(DiffuseProduct_loc, color);

	glBegin(GL_LINES);
	glVertex3f(start.x(), start.y(), start.z());
	glVertex3f(end.x(), end.y(), end.z());
	glEnd();

	glLineWidth(1);
	program->setUniformValue(BoolPoint_loc, 0);
}

void SimulatorRenderBase::drawMeshSelected(QOpenGLShaderProgram *program)
{
	if (simulator->getVolumetricMesh())
	{
		std::vector<int> nodeindices = simulator->getSelectedNodes();
		volumetricMeshRender->renderRegionNodes(program, (simulator->getVolumetricMesh()), nodeindices, 2);

		/*for (int i = 0; i < nodeindices.size(); i++)
		{
		int nodeid = nodeindices[i];
		Vector3d nodep = simulator->getVolumetricMesh()->getNodePosition(nodeid);
		spherelow->drawMesh(program, nodep, 0.01, 5);
		}*/
	}
}

void SimulatorRenderBase::drawSingleSelected(QOpenGLShaderProgram *program)
{
	if (simulator->getMouseInfo() == NULL)
	{
		return;
	}

	int nodeid = simulator->getMouseInfo()->getBindedNodeid();
	if (nodeid == -1)
	{
		return;
	}

	if (simulator->getSimulatorState() == LoboSimulatorBase::beforeinit)
	{
		return;
	}


	std::vector<int> nodeindices(1);
	nodeindices[0] = nodeid;
	//volumetricMeshRender->renderRegionNodes(program, (simulator->getVolumetricMesh()), nodeindices, 7);
	Vector3d curposition = simulator->getVolumetricMesh()->getNodePosition(nodeid);


	double* externalforce = simulator->getMouseExternalForce();

	Vector3d nodeforce;
	nodeforce.data()[0] = externalforce[nodeid * 3 + 0];
	nodeforce.data()[1] = externalforce[nodeid * 3 + 1];
	nodeforce.data()[2] = externalforce[nodeid * 3 + 2];

	if (nodeforce.norm()>1e-8)
	sphere1->drawMesh(program, curposition, 0.05, 0);

}

void SimulatorRenderBase::drawMeshNodes(QOpenGLShaderProgram* program)
{
	if (simulator->getVolumetricMesh())
	{
		int numVertex = simulator->getVolumetricMesh()->getNumVertices();
		std::vector<int> nodeindices(numVertex);
		for (int i = 0; i < numVertex; i++)
		{
			nodeindices[i] = i;
		}
		volumetricMeshRender->renderRegionNodes(program, (simulator->getVolumetricMesh()), nodeindices, 4);
	}
}

void SimulatorRenderBase::drawMeshConstrainedNodes(QOpenGLShaderProgram *program)
{
	if (simulator->getVolumetricMesh())
	{
		std::vector<int> nodeindices = simulator->getConstrainedNodes();
		volumetricMeshRender->renderRegionNodes(program, (simulator->getVolumetricMesh()), nodeindices, 0);



	}
}
