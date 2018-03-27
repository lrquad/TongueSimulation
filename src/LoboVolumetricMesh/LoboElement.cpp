#include "LoboElement.h"


LoboElement::LoboElement()
{
	is_selected = false;
}


LoboElement::~LoboElement()
{
}

void LoboElement::setOriCenterPosition(Vector3d postion)
{
	this->ori_center_position = postion;
}

Vector3d LoboElement::getOriCenterPosition()
{
	return ori_center_position;
}
