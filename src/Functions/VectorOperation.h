#pragma once
#include <Eigen/Dense>
#include <vector>

/**
 * @brief Put brief description here
 *
 * compute vector's gradient by differential
 *
 * @param[in,out] inputVector Put argument desc here
 * @return Put return information here
 */
 Eigen::VectorXd computeVectorGradient(Eigen::VectorXd &inputVector);


/**
 * @brief Put brief description here
 *
 * Put full description here
 *
 * @param[in,out] inputVector Put argument desc here
 * @param[in,out] order 0 descending  1 ascending
 * @return Put return information here
 */
 std::vector<int> sortVector(Eigen::VectorXd &inputVector,int order);

 void lowPassFilter(double*, int size, double alpha);