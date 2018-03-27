#pragma once
#include <Eigen/Dense>
#include <Eigen/Sparse>
using namespace Eigen;
typedef Eigen::Triplet<double> EIGEN_TRI;
int matrixNormalize(SparseMatrix<double> *matrix, VectorXd *vector);