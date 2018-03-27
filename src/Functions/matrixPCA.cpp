#include "matrixPCA.h"

int MatrixPCA(MatrixXd *A, int r, SparseMatrix<double> *weights)
{
	int rows = A->rows();
	int cols = A->cols();

	MatrixXd dataset(rows,cols);
	
	dataset = A->transpose();

	JacobiSVD<MatrixXd> svd(dataset, ComputeFullU | ComputeFullV);
	
	dataset = svd.matrixV();

	A->resize(rows, r);
	for (int i = 0; i < r; i++)
	{
		A->col(i) = dataset.col(i);
	}
	//*A = dataset;
	return 1;
}