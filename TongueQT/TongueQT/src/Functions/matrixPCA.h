#pragma once
#include <Eigen/Dense>
#include <Eigen/Sparse>
using namespace Eigen;

int MatrixPCA(MatrixXd *A,int r, SparseMatrix<double> *weights = NULL);

