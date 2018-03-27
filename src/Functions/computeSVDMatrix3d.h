#pragma once
#include <Eigen/Dense>
using namespace Eigen;


void computeSVD(Matrix3d &F, Matrix3d &U, Matrix3d &V, Matrix3d &singularF, double singularValue_eps, int modifiedSVD);

void eigen_sym(Matrix3d & M, Vector3d & eig_val, Vector3d(&eig_vec)[3]);

void findOrthonormalVector(Vector3d &v1, Vector3d &v2);
