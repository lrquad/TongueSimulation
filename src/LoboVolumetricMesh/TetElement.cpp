#include "TetElement.h"
#include <vector>


TetElement::TetElement()
{
	face_indices[0] = 3;
	face_indices[1] = 1;
	face_indices[2] = 0;

	face_indices[3] = 3;
	face_indices[4] = 2;
	face_indices[5] = 1;

	face_indices[6] = 3;
	face_indices[7] = 0;
	face_indices[8] = 2;

	face_indices[9] = 2;
	face_indices[10] = 1;
	face_indices[11] = 0;


}


TetElement::~TetElement()
{
}

LoboElement::elementType TetElement::getType() const
{
	return LoboElement::TET;
}

void TetElement::computeElementDeformationshapeMatrix(Matrix3d &Ds, double* dis)
{
	Ds.setZero();
	int ne[4];
	for (int i = 0; i < 4; i++)
	{
		ne[i] = this->node_indices[i];
	}

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			Ds.data()[j * 3 + i] = dis[ne[j] * 3 + i] - dis[ne[3] * 3 + i];
		}
	}

	Ds += this->Dm;
}

TetElement* downCastTetelement(LoboElement* element)
{
	if (element->getType() == LoboElement::TET)
	{
		return (TetElement*)element;
	}
	else
	{
		return NULL;
	}
}
