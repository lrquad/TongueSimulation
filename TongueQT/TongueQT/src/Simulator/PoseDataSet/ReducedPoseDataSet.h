#pragma once
#include "PoseDataSet.h"

class ReducedPoseDataSet:public PoseDataSet
{
public:
	//************************************
	// Method:    ReducedPoseDataSet
	// FullName:  ReducedPoseDataSet::ReducedPoseDataSet
	// Access:    public 
	// Returns:   
	// Qualifier:
	// Parameter: int R
	// Parameter: int r this will be reset by subspacePhi's column
	// Parameter: MatrixXd subspacePhi
	//************************************
	ReducedPoseDataSet(int R,int r,MatrixXd subspacePhi);
	ReducedPoseDataSet(int R);
	
	~ReducedPoseDataSet();

	//************************************
	// Method:    projectFromFullpose will compute reduced pose by inside subspace matrix member.
	// FullName:  ReducedPoseDataSet::projectFromFullpose
	//************************************
	void fitFromFullpose();
	void computeFitError();

	void setSubspacePhi(MatrixXd phi);

	virtual void loadReducedPoseFile(const char* filename);
	virtual void saveReducedPoseFile(const char* filename);

	VectorXd getReducedPose(int poseid);
	VectorXd getFitedFullPose(int poseid);

	virtual void clearData();

protected:

	int r;
	MatrixXd subspacePhi;
	std::vector<VectorXd*> reduced_pose;
	std::vector<VectorXd*> fited_full_pose;

	Eigen::LDLT<MatrixXd> leastsqure_solver;

public:
	int getr();
};

