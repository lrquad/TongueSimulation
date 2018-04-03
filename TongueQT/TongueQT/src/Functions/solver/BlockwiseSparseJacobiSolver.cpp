#include "BlockwiseSparseJacobiSolver.h"
#include <iostream>
#include <fstream>

BlockwiseSparseJacobiSolver::BlockwiseSparseJacobiSolver(SparseMatrix<double> &systemMatrix, int numBlock, std::vector<int> blockinfo, int maxiteration /*= 10*/, double threshold /*= 1e-10*/) :SparseJacobiSolver(systemMatrix,maxiteration,threshold,false)
{
	this->numBlock = numBlock;
	this->blockinfo = blockinfo;

	decomposeMatrix();

}

BlockwiseSparseJacobiSolver::~BlockwiseSparseJacobiSolver()
{
}

void BlockwiseSparseJacobiSolver::decomposeMatrix()
{
	std::vector<MatrixXd*> blockmatrix(numBlock);
	std::vector<int> mapDofsToBlock(systemMatrix.rows());
	std::fill(mapDofsToBlock.begin(), mapDofsToBlock.end(),-1);

	for (int i = 0; i < numBlock; i++)
	{
		int rows = blockinfo[i*4+2];
		int cols = blockinfo[i*4+3];
		blockmatrix[i] = new MatrixXd(rows, cols);
		blockmatrix[i]->setZero();

		int rowindex = blockinfo[i * 4 + 0];
		for (int j = 0; j < rows; j++)
		{
			mapDofsToBlock[rowindex + j] = i;
		}
	}
	
	//assign all block matrix
	for (int i = 0; i < systemMatrix.outerSize(); ++i)
	{
		for (SparseMatrix<double>::InnerIterator it(systemMatrix, i); it; ++it)
		{
			int row = it.row();
			int col = it.col();
			
			if (mapDofsToBlock[row] == mapDofsToBlock[col])
			{
				int blockid = mapDofsToBlock[row];

				int inside_row = row - blockinfo[blockid * 4 + 0];
				int inside_col = col - blockinfo[blockid * 4 + 1];

				blockmatrix[blockid]->data()[inside_col*blockmatrix[blockid]->rows() + inside_row]
					= it.value();
			}
		}
	}

	//compute inverse matrix;
	std::vector<Eigen::Triplet<double>> coeff_inverdiag;
	for (int i = 0; i < numBlock; i++)
	{
		MatrixXd IdentityM(blockmatrix[i]->rows(), blockmatrix[i]->cols());
		IdentityM.setIdentity();
		MatrixXd inversed = blockmatrix[i]->inverse();

		int rows = blockinfo[i * 4 + 2];
		int cols = blockinfo[i * 4 + 3];

		int rowoffset = blockinfo[i * 4 + 0];
		int coloffset = blockinfo[i * 4 + 1];

		for (int r = 0; r < rows; r++)
		{
			for (int c = 0; c < cols; c++)
			{
				double val = inversed.data()[c*inversed.rows() + r];

				coeff_inverdiag.push_back(Eigen::Triplet<double>(rowoffset+r,coloffset+c,val));
			}
		}
	}

	inverseD.setFromTriplets(coeff_inverdiag.begin(), coeff_inverdiag.end());

}
