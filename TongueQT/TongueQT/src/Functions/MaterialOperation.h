#pragma once
#include <Eigen/Dense>
#include <Eigen/Sparse>
using namespace Eigen;

void computeStrain(Matrix3d &inputF, Matrix3d &outputE);
void computePiolaStress(Matrix3d &E, Matrix3d &F, Matrix3d &P, double &mu_, double &lambda_);