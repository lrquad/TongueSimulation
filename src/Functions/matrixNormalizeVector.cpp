#include "matrixNormalizeVector.h"

int matrixNormalize(SparseMatrix<double> *matrix, VectorXd *vector)
{
	double result = 0;
	for (int j = 0; j < matrix->outerSize(); ++j)
		for (SparseMatrix<double>::InnerIterator it(*matrix, j); it; ++it)
		{
			int r = it.row(); int c = it.col();
			if (c < r)
			{
				continue;
			}

			if (c == r)
			{
				result += it.value()*vector->data()[r] * vector->data()[c];
			}
			else
			{
				result += 2.0*it.value()*vector->data()[r] * vector->data()[c];
			}
		}
	result = std::sqrt(result);
	*vector /= result;
	return 1;
}