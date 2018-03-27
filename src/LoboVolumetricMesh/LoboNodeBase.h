#pragma once
#include <Eigen/Dense>
#include <Eigen/Sparse>
using namespace Eigen;

class LoboNodeBase
{
public:
	LoboNodeBase();
	~LoboNodeBase();

	Vector3d ori_position;
	Vector3d displacement;
	Vector3d cur_position;

	std::vector<int> neighbor;
	std::vector<int> element_list;

	int index_;
	bool b_constraint;
};

