#pragma once
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <vector>

using namespace Eigen;

int Quadrature(int quadrature_order, int dimension, std::vector<double> &weights, std::vector<double> &quadratures);