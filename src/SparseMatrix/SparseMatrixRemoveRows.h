#pragma once
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "SparseMatrixTopology.h"
#include <iostream>

using namespace Eigen;
typedef Eigen::Triplet<double> EIGEN_TRI;

void subSparseMatrix(SparseMatrix<double> &source, SparseMatrix<double> &result, std::vector<int> &map)
;

void SparseMatrixRemoveRows(SparseMatrix<double>* sparseMatrix, SparseMatrix<double>* resultMatrix, std::vector<int>& entry_map, int r, int numConstrainedDOFs_, int* constrainedDOFs_)
;

void VectorRemoveRows(std::vector<int> &map, VectorXd &target, VectorXd &result, int numConstrainedDOFs_ = NULL, int* constrainedDOFs_ = NULL)
;

void VectorInsertRows(std::vector<int> &map, VectorXd &target, VectorXd &result, int numConstrainedDOFs_ = NULL, int* constrainedDOFs_ = NULL)
;

void MatrixRemoveDofs(std::vector<int> &map, MatrixXd& target,MatrixXd &result);

void MatrixInsertDofs(std::vector<int> &map, MatrixXd& target, MatrixXd &result);

void MatrixRemoveRows(std::vector<int> &map, MatrixXd& target, MatrixXd &result);

void MatrixRemoveCols(std::vector<int> &map, MatrixXd& target, MatrixXd &result);

void createMapByConstrains(std::vector<int> &map, int r, int numConstrainedDOFs_, int* constrainedDOFs_);

/// <summary>
/// Creates the sparse map by topology.
/// </summary>
/// <param name="sparseMatrix">The sparse matrix.</param>
/// <param name="subsparseMatrix">The subsparse matrix.</param>
/// <param name="entry_map">The entry_map.</param>
/// <param name="rowmap">The rowmap.</param>
/// <param name="r">The r.</param>
/// <param name="numConstrainedDOFs_">The number constrained do FS_.</param>
/// <param name="constrainedDOFs_">The constrained do FS_.</param>
void createSparseMapbyTopology(SparseMatrix<double>* sparseMatrix, SparseMatrix<double>* subsparseMatrix, std::vector<int>& entry_map, std::vector<int>& rowmap, int r, int numConstrainedDOFs_, int* constrainedDOFs_);