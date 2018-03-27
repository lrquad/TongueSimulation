#include "ENuMaterial.h"

LoboVolumetricMesh::ENuMaterial::~ENuMaterial()
{
}

LoboVolumetricMesh::Material * LoboVolumetricMesh::ENuMaterial::clone() const
{
	return new LoboVolumetricMesh::ENuMaterial(*this);
}

LoboVolumetricMesh::Material::materialType LoboVolumetricMesh::ENuMaterial::getType()
{
	return LoboVolumetricMesh::Material::ENU;
}

LoboVolumetricMesh::ENuMaterial * downcastENuMaterial(LoboVolumetricMesh::Material * material)
{
	if (material->getType() != LoboVolumetricMesh::Material::ENU)
		return NULL;

	return (LoboVolumetricMesh::ENuMaterial*) material;
}