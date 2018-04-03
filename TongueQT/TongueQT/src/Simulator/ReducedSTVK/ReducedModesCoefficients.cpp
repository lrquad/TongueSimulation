#include "ReducedModesCoefficients.h"
#include <iostream>
#include "LoboVolumetricMesh/TetElement.h"

ReducedModesCoefficients::ReducedModesCoefficients(MatrixXd* modes, LoboVolumetricMesh* volumetric_mesh, MatrixXd* linearCoef, MatrixXd* quadraticCoef, MatrixXd* cubicCoef, double *lambdaLame_, double *muLame_, bool verbose)
{
	this->lambdaLame = lambdaLame_;
	this->muLame = muLame_;

	modes_ = modes;
	volumetric_mesh_ = volumetric_mesh;

	linearCoef_ = linearCoef;
	quadraticCoef_ = quadraticCoef;
	cubicCoef_ = cubicCoef;

	r = modes_->cols();
	linearSize = r;
	quadraticSize = r*r;
	cubicSize = r*r*r;

	linearCoef_->resize(r, linearSize);
	quadraticCoef_->resize(r, quadraticSize);
	cubicCoef_->resize(r, cubicSize);

	linearCoef_->setZero();
	quadraticCoef_->setZero();
	cubicCoef_->setZero();

	if (verbose)
	{
		std::cout << "init modes, compute coefficients ... " <<std::endl;
	}

	std::streamsize ss = std::cout.precision();
	std::cout.setf(std::ios::fixed);
	std::cout.precision(2);

	int numElements = volumetric_mesh->getNumElements();
	for (int i = 0; i < numElements; i++)
	{
		TetElement* element = downCastTetelement(volumetric_mesh->getElement(i));
		computeSubspaceCoefficients(*element, r, modes);
		if (verbose)
		{
			std::cout << i*(100.0 / (double)numElements) << "%" << "\r";
		}
	}

	std::cout.precision(ss);
	if (verbose)
	{
		std::cout << std::endl;
		std::cout << "finished" << std::endl;
	}

}


ReducedModesCoefficients::~ReducedModesCoefficients()
{
}

void ReducedModesCoefficients::computeSubspaceCoefficients(TetElement& te, int r, MatrixXd *modes)
{
	double lambda = lambdaLame[te.index_];
	double mu = muLame[te.index_];

	if (modes == NULL)
	{
		modes = this->modes_;
	}

	int rows = modes->rows();

	//loop r for linear term P
	for (int i = 0; i < r; i++)
	{
		for (int c = 0; c < 4; c++)
		{
			int vc = te.node_indices[c];
			Vector3d force = Vector3d::Zero();
			for (int a = 0; a < 4; a++)
			{
				int va = te.node_indices[a];
				Vector3d ua(
					modes->data()[i*rows + va * 3 + 0],
					modes->data()[i*rows + va * 3 + 1],
					modes->data()[i*rows + va * 3 + 2]
					);
				force += lambda * te.A[c][a] * ua + mu*te.B[a][c] * ua + mu*te.A[a][c] * ua;
			}
			//contribution to global coefficients
			linearCoef_->col(i) += modes->block(vc * 3, 0, 3, r).transpose()*force;
		}
	}

	int r2 = r*r;
	//compute quadratic term
	for (int i = 0; i < r; i++)
	{
		for (int j = 0; j < r; j++)
		{
			for (int c = 0; c < 4; c++)
			{
				int vc = te.node_indices[c];
				Vector3d force = Vector3d::Zero();

				for (int a = 0; a < 4; a++)
				{
					int va = te.node_indices[a];
					Vector3d posa_i(
						modes->data()[i*rows + va * 3 + 0],
						modes->data()[i*rows + va * 3 + 1],
						modes->data()[i*rows + va * 3 + 2]
						);
					Vector3d posa_j(
						modes->data()[j*rows + va * 3 + 0],
						modes->data()[j*rows + va * 3 + 1],
						modes->data()[j*rows + va * 3 + 2]
						);

					for (int b = 0; b < 4; b++)
					{
						Vector3d vec1 = 0.5*lambda*te.C[c][a][b] + mu*te.C[a][b][c];
						Vector3d C = lambda*te.C[a][b][c] + mu*te.C[c][a][b] + mu*te.C[b][a][c];

						int vb = te.node_indices[b];
						Vector3d posb_i(
							modes->data()[i*rows + vb * 3 + 0],
							modes->data()[i*rows + vb * 3 + 1],
							modes->data()[i*rows + vb * 3 + 2]
							);
						Vector3d posb_j(
							modes->data()[j*rows + vb * 3 + 0],
							modes->data()[j*rows + vb * 3 + 1],
							modes->data()[j*rows + vb * 3 + 2]
							);
						double buffer = posa_i.dot(posb_j);
						force += buffer*vec1 + (posb_i*posa_j.transpose())*C;
					}
				}

				int pos = i*r + j;
				quadraticCoef_->col(pos) += modes->block(vc * 3, 0, 3, r).transpose()*force;
			}
		}
	}

	//cubic term
	for (int i = 0; i<r; i++)
		for (int j = 0; j<r; j++)
			for (int k = 0; k < r; k++)
			{
				for (int c = 0; c < 4; c++)
				{
					int vc = te.node_indices[c];
					Vector3d force = Vector3d::Zero();
					for (int a = 0; a < 4; a++)
					{
						int va = te.node_indices[a];
						Vector3d posa_i(
							modes->data()[i*rows + va * 3 + 0],
							modes->data()[i*rows + va * 3 + 1],
							modes->data()[i*rows + va * 3 + 2]
							);
						for (int b = 0; b < 4; b++)
						{
							int vb = te.node_indices[b];
							Vector3d posb_j(
								modes->data()[j*rows + vb * 3 + 0],
								modes->data()[j*rows + vb * 3 + 1],
								modes->data()[j*rows + vb * 3 + 2]
								);

							for (int d = 0; d < 4; d++)
							{
								int vd = te.node_indices[d];
								Vector3d posd_k(
									modes->data()[k*rows + vd * 3 + 0],
									modes->data()[k*rows + vd * 3 + 1],
									modes->data()[k*rows + vd * 3 + 2]
									);

								double D = 0.5*lambda*te.D[a][b][c][d] + mu*te.D[a][c][b][d];
								force += D*posa_i.dot(posb_j)*posd_k;
							}
						}
					}
					int pos = i*r*r + j*r + k;
					cubicCoef_->col(pos) += modes->block(vc * 3, 0, 3, r).transpose()*force;
				}
			}

}
