#pragma once
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <vector>
using namespace Eigen;

void lagrange_basis(VectorXd&N , MatrixXd& dNdxi,Vector2d coord);