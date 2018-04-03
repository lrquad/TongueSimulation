#include "MouseProjection.h"
#include <iostream>

MouseProjection::MouseProjection()
{
	MVP.setIdentity();
	mouseXY.setZero();
	x1 = 0;
	x2 = 0;
	y1 = 0;
	y2 = 0;
	width = 0;
	height = 0;
	selectedNodeIndex = -1;
}


MouseProjection::~MouseProjection()
{
}

bool MouseProjection::isInMouseRect(double &x, double &y)
{
	if ((x < x2&&x > x1) && (y<y2&&y>y1))
	{
		return true;
	}
	else
	{
		return false;
	}
}

void MouseProjection::projectScreenToWorld(Vector2d screenpos, double depth, Vector3d &resultposition)
{
	Vector4d vector;
	vector.data()[0] = screenpos.data()[0] * 2.0 / width - 1;
	vector.data()[1] = (height - screenpos.data()[1])*2.0 / height - 1;
	vector.data()[2] = depth;
	vector.data()[3] = 1;

	vector = MVP.inverse()*vector;
	resultposition.data()[0] = vector.data()[0] / vector.data()[3];
	resultposition.data()[1] = vector.data()[1] / vector.data()[3];
	resultposition.data()[2] = vector.data()[2] / vector.data()[3];
}

void MouseProjection::projectWorldToScreen(Vector3d position, Vector2d &screenresult, double &depth)
{
	Vector4d vector;
	vector.data()[0] = position.data()[0];
	vector.data()[1] = position.data()[1];
	vector.data()[2] = position.data()[2];
	vector.data()[3] = 1;

	vector = this->MVP*vector;

	vector.data()[0] /= vector.data()[3];
	vector.data()[1] /= vector.data()[3];
	depth = vector.data()[2] / vector.data()[3];
	//depth = vector.data()[2];

	screenresult.data()[0] = (vector.data()[0] + 1)*width / 2.0;
	screenresult.data()[1] = height - (vector.data()[1] + 1)*height / 2.0;
}

void MouseProjection::projectWorldToScreenDepth(Vector3d position, Vector2d &screenresult, double &depth)
{
	Vector4d vector;
	vector.data()[0] = position.data()[0];
	vector.data()[1] = position.data()[1];
	vector.data()[2] = position.data()[2];
	vector.data()[3] = 1;

	vector = this->MVP*vector;

	vector.data()[0] /= vector.data()[3];
	vector.data()[1] /= vector.data()[3];
	//depth = vector.data()[2] / vector.data()[3];
	depth = vector.data()[2];

	screenresult.data()[0] = (vector.data()[0] + 1)*width / 2.0;
	screenresult.data()[1] = height - (vector.data()[1] + 1)*height / 2.0;
}
