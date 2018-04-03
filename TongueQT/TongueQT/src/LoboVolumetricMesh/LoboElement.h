#pragma once
#include <vector>
#include "LoboVolumetricMesh.h"

class LoboElement
{
public:
	LoboElement();
	~LoboElement();

	typedef enum { TET,CUBE } elementType;

	std::vector<int> node_indices;
	std::vector<int> neighbors;

	bool is_selected;
	int index_;

	virtual elementType getType() const = 0;
	void setMaterial(LoboVolumetricMesh::Material* material_);
	void setOriCenterPosition(Vector3d postion);
	Vector3d getOriCenterPosition();

protected:

	LoboVolumetricMesh::Material* material;
	Vector3d ori_center_position;

};

