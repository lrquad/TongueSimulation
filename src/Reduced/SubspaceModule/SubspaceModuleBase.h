#pragma once
#include <Eigen/Dense>
#include <Eigen/Sparse>
using namespace Eigen;

class SubspaceModuleBase
{
public:
	SubspaceModuleBase();
	~SubspaceModuleBase();

	virtual void generateSubspace() = 0;
	virtual void loadSubspace(const char* filename);
	virtual void saveSubspace(const char* filename);
	virtual void saveSubspace(std::ofstream& outstream);
	virtual void loadSubspaceStream(std::ifstream& instream);

	MatrixXd* ori_phi;
	MatrixXd* pre_phi;
	
	MatrixXd* phi;

	int r;
	int ori_r;

protected:
	virtual void init();

};

