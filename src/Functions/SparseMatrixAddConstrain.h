#pragma once
#include <Eigen/Dense>
#include <Eigen/Sparse>
using namespace Eigen;
void SparseMatrixAddConstrain(SparseMatrix<double>* originMatrix, std::vector<Eigen::Triplet<double>> &constrain,int numConstrain,SparseMatrix<double>* result);