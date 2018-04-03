#include "CubeElement.h"

/*
0 1 2 3
4 5 6 7
*/


CubeElement::CubeElement()
{
	//face 0 front
	face_indices[0 * 4 + 0] = 7;
	face_indices[0 * 4 + 1] = 4;
	face_indices[0 * 4 + 2] = 0;
	face_indices[0 * 4 + 3] = 3;

	inverface_indeices[0 * 4 + 0] = 6;
	inverface_indeices[0 * 4 + 1] = 5;
	inverface_indeices[0 * 4 + 2] = 1;
	inverface_indeices[0 * 4 + 3] = 2;


	//face 1 back
	face_indices[1 * 4 + 0] = 2;
	face_indices[1 * 4 + 1] = 1;
	face_indices[1 * 4 + 2] = 5;
	face_indices[1 * 4 + 3] = 6;

	inverface_indeices[1 * 4 + 0] = 3;
	inverface_indeices[1 * 4 + 1] = 0;
	inverface_indeices[1 * 4 + 2] = 4;
	inverface_indeices[1 * 4 + 3] = 7;

	//face 2 left
	face_indices[2 * 4 + 0] = 3;
	face_indices[2 * 4 + 1] = 2;
	face_indices[2 * 4 + 2] = 6;
	face_indices[2 * 4 + 3] = 7;

	inverface_indeices[2 * 4 + 0] = 0;
	inverface_indeices[2 * 4 + 1] = 1;
	inverface_indeices[2 * 4 + 2] = 5;
	inverface_indeices[2 * 4 + 3] = 4;

	//face 3 right
	face_indices[3 * 4 + 0] = 0;
	face_indices[3 * 4 + 1] = 4;
	face_indices[3 * 4 + 2] = 5;
	face_indices[3 * 4 + 3] = 1;

	inverface_indeices[3 * 4 + 0] = 3;
	inverface_indeices[3 * 4 + 1] = 7;
	inverface_indeices[3 * 4 + 2] = 6;
	inverface_indeices[3 * 4 + 3] = 2;

	//face 4 top
	face_indices[4 * 4 + 0] = 1;
	face_indices[4 * 4 + 1] = 2;
	face_indices[4 * 4 + 2] = 3;
	face_indices[4 * 4 + 3] = 0;

	inverface_indeices[4 * 4 + 0] = 5;
	inverface_indeices[4 * 4 + 1] = 6;
	inverface_indeices[4 * 4 + 2] = 7;
	inverface_indeices[4 * 4 + 3] = 4;

	//face 5 botton
	face_indices[5 * 4 + 0] = 4;
	face_indices[5 * 4 + 1] = 7;
	face_indices[5 * 4 + 2] = 6;
	face_indices[5 * 4 + 3] = 5;

	inverface_indeices[5 * 4 + 0] = 0;
	inverface_indeices[5 * 4 + 1] = 3;
	inverface_indeices[5 * 4 + 2] = 2;
	inverface_indeices[5 * 4 + 3] = 1;
}


CubeElement::~CubeElement()
{
}

LoboElement::elementType CubeElement::getType() const
{
	return LoboElement::CUBE;
}

CubeElement* downCastCubeElement(LoboElement* element)
{
	if (element->getType() == LoboElement::CUBE)
	{
		return (CubeElement*)element;
	}
	else
	{
		return NULL;
	}
}
