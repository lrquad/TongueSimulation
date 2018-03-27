#include "SparseMatrixAddConstrain.h"

void SparseMatrixAddConstrain(SparseMatrix<double>* originMatrix, std::vector<Eigen::Triplet<double>> &constrain, int numConstrain, SparseMatrix<double>* result)
{
	for (int i = 0; i < originMatrix->outerSize(); i++)
		for (SparseMatrix<double>::InnerIterator it(*originMatrix, i); it; ++it)
	{
		constrain.push_back(Eigen::Triplet<double>(it.row(), it.col(), it.value()));
	}

	result->setFromTriplets(constrain.begin(), constrain.end());
}
