#include "LoboNodeBase.h"


LoboNodeBase::LoboNodeBase():
b_constraint(0)
{
	ori_position.setZero();
	displacement.setZero();
	cur_position.setZero();
}


LoboNodeBase::~LoboNodeBase()
{

}


