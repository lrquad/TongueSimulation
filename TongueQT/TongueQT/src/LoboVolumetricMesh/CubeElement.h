#pragma once
#include "LoboElement.h"

class CubeElement:public LoboElement
{
public:
	CubeElement();
	~CubeElement();

	virtual LoboElement::elementType getType() const;

	int face_indices[24];
	int inverface_indeices[24];

	double edge_length;
	Vector3d cube_center;
	double volume_;
	int region_;
};


CubeElement* downCastCubeElement(LoboElement* element);

