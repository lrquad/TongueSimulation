#pragma once
#include "SparseJacobiSolver.h"

class BlockwiseSparseJacobiSolver:public SparseJacobiSolver
{
public:
	BlockwiseSparseJacobiSolver(SparseMatrix<double> &systemMatrix,int numBlock,std::vector<int> blockinfo, int maxiteration = 10, double threshold = 1e-10);
	~BlockwiseSparseJacobiSolver();


protected:

	virtual void decomposeMatrix();

	int numBlock;
	//4*n block
	std::vector<int> blockinfo;

};

