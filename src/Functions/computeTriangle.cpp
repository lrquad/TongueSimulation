#include "computeTriangle.h"

void computeTriangleNorm(Vector3d &v1, Vector3d &v2, Vector3d &v3, Vector3d &normal)
{
	normal = (v2 - v1).cross(v3 - v1);
	normal.normalize();
}

double computeTriangleArea(Vector3d n0, Vector3d n1, Vector3d n2)
{
	n1 = n1 - n0;
	n2 = n2 - n0;

	n0 = n1.cross(n2);

	double area = 0.5 * n0.norm();
	return area;
}

double computeDistancePointToTriangle(Vector3d &v1, Vector3d &v2, Vector3d &v3, Vector3d &normal, Vector3d &p)
{
	Vector3d pv1 = (p - v1);
	return (p - v1).norm()*pv1.dot(normal) / (pv1.norm());
}
