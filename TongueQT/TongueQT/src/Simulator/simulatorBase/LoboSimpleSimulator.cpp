#include "LoboSimpleSimulator.h"
#include "LoboVolumetricMesh/LoboVolumetriceMeshCore.h"



LoboSimpleSimulator::LoboSimpleSimulator(std::ifstream &readconfig, LoboTriMesh* tri_mesh, bool ifreadconfig)
	:LoboSimulatorBase(readconfig, tri_mesh)
{

}

LoboSimpleSimulator::~LoboSimpleSimulator()
{
}

void LoboSimpleSimulator::readVolumetricMeshAscii(const char* filenamebase)
{
	TetVolumetricMesh* tet_volumetric_mesh = new TetVolumetricMesh();

	Vector3d translate = Vector3d::Zero();

	tet_volumetric_mesh->readElementMesh(filenamebase, simulator_translate.data(), getSimulator_scale());

	volumetric_mesh = tet_volumetric_mesh;
}
