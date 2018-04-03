#pragma once
#include <Eigen/Dense>
#include <Eigen/Sparse>
using namespace Eigen;

void computeTriangleNorm(Vector3d &v1, Vector3d &v2, Vector3d &v3, Vector3d &normal);

double computeTriangleArea(Vector3d n0, Vector3d n1, Vector3d n2);

double computeDistancePointToTriangle(Vector3d &v1, Vector3d &v2, Vector3d &v3, Vector3d &normal, Vector3d &p);