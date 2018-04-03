#pragma once
#include <map>
#include <Eigen/Dense>
#include <Eigen/Sparse>
using namespace Eigen;

class SparseMatrixTopology
{
public:
	SparseMatrixTopology(SparseMatrix<double>* sparseMatrix);
	~SparseMatrixTopology();

	int getValueIndex(int row, int col);
protected:
	
	SparseMatrix<double> sparse_matrix;
	std::map<int, int> mapOfSparse ;
	int rows_;
	int cols_;

};

