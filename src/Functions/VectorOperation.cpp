#include "VectorOperation.h"
#include <numeric>

Eigen::VectorXd computeVectorGradient(Eigen::VectorXd &inputVector)
{

	int size = inputVector.rows();
	assert(size > 1);

	Eigen::VectorXd result(size);
	result.setZero();
	for (int i = 0; i < size; i++)
	{
		if (!(i == 0 || i == size - 1))
			result[i] = (inputVector[i + 1] - inputVector[i - 1]) / 2.0;
		else if (i == 0)
		{
			result[i] = inputVector[i + 1] - inputVector[i];
		}
		else if (i == size - 1)
		{
			result[i] = inputVector[i] - inputVector[i - 1];
		}
	}
	return result;
}

std::vector<int> sortVector(Eigen::VectorXd &inputVector, int order)
{
	std::vector<int> idx(inputVector.size());
	iota(idx.begin(), idx.end(), 0);

	if (order == 1)
	{
		sort(idx.begin(), idx.end(),
			[&inputVector](int i1, int i2) {return inputVector[i1] < inputVector[i2]; });
	}
	else
	{
		sort(idx.begin(), idx.end(),
			[&inputVector](int i1, int i2) {return inputVector[i1] > inputVector[i2]; });
	}

	return idx;
}

void lowPassFilter(double* data, int size, double alpha)
{
	Eigen::VectorXd output(size);
	output[0] = data[0];
	for (int i = 1; i < size; i++)
	{
		output[i] = output[i - 1] * (1 - alpha) + alpha*data[i];
	}
	memcpy(data, output.data(), sizeof(double)*size);
}
