#include "SubspaceModesCreator.h"
#include "Simulator/ForceModel/LoboForceModel.h"
#include "Simulator/ARPACKSolver.h"
#include "SparseMatrix/SparseMatrixRemoveRows.h"
#include "Functions/matrixPCA.h"
#include "Functions/matrixNormalizeVector.h"

SubspaceModesCreator::SubspaceModesCreator(LoboForceModel* forceModel_, SparseMatrix<double>* massMatrix_, int numConstrainedDOFs_, int* constrainedDOFs_, int k_, int r_) : forceModel(forceModel_), massMatrix(massMatrix_), k(k_), r(r_)
{
	this->numConstrainedDOFs = numConstrainedDOFs_;
	this->constrainedDOFs = (int*)malloc(sizeof(int) * numConstrainedDOFs);
	memcpy(this->constrainedDOFs, constrainedDOFs_, sizeof(int) * numConstrainedDOFs);


	int dofs = forceModel->getr();
	createMapByConstrains(mapOldNew, dofs, numConstrainedDOFs, constrainedDOFs);

	tangentStiffnessMatrix = new SparseMatrix<double>();
	forceModel->getTangentStiffnessMatrixTopology(tangentStiffnessMatrix);

}


SubspaceModesCreator::SubspaceModesCreator(LoboForceModel* forceModel_, SparseMatrix<double>* massMatrix_, int numConstrainedDOFs_, int* constrainedDOFs_, int k_, int r_, MatrixXd* linearmodes_) :forceModel(forceModel_), massMatrix(massMatrix_), k(k_), r(r_)
{
	this->numConstrainedDOFs = numConstrainedDOFs_;
	this->constrainedDOFs = (int*)malloc(sizeof(int) * numConstrainedDOFs);
	memcpy(this->constrainedDOFs, constrainedDOFs_, sizeof(int) * numConstrainedDOFs);

	int dofs = forceModel->getr();
	createMapByConstrains(mapOldNew, dofs, numConstrainedDOFs, constrainedDOFs);

	tangentStiffnessMatrix = new SparseMatrix<double>();
	forceModel->getTangentStiffnessMatrixTopology(tangentStiffnessMatrix);

	generateModesFromInput(linearmodes_);
}

SubspaceModesCreator::~SubspaceModesCreator()
{
	free(constrainedDOFs);
	delete tangentStiffnessMatrix;
}

void SubspaceModesCreator::generateModes()
{
	buildFreeLMAMode(k);
	modalDerivaties();
	modesPCA(r); 
}

void SubspaceModesCreator::generateLinearModes()
{
	buildFreeLMAMode(k);
	modalDerivaties();
	MatrixXd temp = derivativemodes.block(0, 0, derivativemodes.rows(), r);
	derivativemodes = temp;
	modesPCA(r);
}

void SubspaceModesCreator::generateModesFromInput(MatrixXd* linearModes)
{
	initLinearModes(linearModes);
	modalDerivaties();
	modesPCA(r);
}

void SubspaceModesCreator::insertZeros(MatrixXd* modes, MatrixXd* output)
{
	int dofs = forceModel->getr();
	int vec_len = dofs - numConstrainedDOFs;

	output->resize(dofs, modes->cols());
	output->setZero();

	for (int i = 0; i < mapOldNew.size(); i++)
	{
		if (mapOldNew[i] != -1)
		{
			output->row(i) = modes->row(mapOldNew[i]);
		}
	}
}

void SubspaceModesCreator::buildFreeLMAMode(int numLMAmodes)
{
	int dofs = forceModel->getr();
	int vec_len = dofs - numConstrainedDOFs;
	int constrained_nodes = numConstrainedDOFs / 3;

	if (constrained_nodes == 0)
	{
		rigid_modes_number = 6;
	}
	else if (constrained_nodes == 1)
	{
		rigid_modes_number = 3;
	}
	else if (constrained_nodes == 2)
	{
		rigid_modes_number = 1;
	}
	else if (constrained_nodes >= 3)
	{
		rigid_modes_number = 0;
	}
	std::cout << "rigid_modes_number " << rigid_modes_number << std::endl;

	linearmodes.resize(vec_len, numLMAmodes + rigid_modes_number);
	eigen_values.resize(numLMAmodes + rigid_modes_number);

	ARPACKSolver gen_eige_wrapper;

	MatrixXd full_eigen_vectors(vec_len, numLMAmodes + rigid_modes_number);
	VectorXd full_eigen_values(vec_len);

	SparseMatrix<double> ordered_linear_stiffness_matrix(vec_len, vec_len);
	SparseMatrix<double> ordered_mass_matrix(vec_len, vec_len);
	
	
	VectorXd u(dofs);
	u.setZero();
	forceModel->getTangentStiffnessMatrix(u.data(),tangentStiffnessMatrix);
	subSparseMatrix(*tangentStiffnessMatrix, ordered_linear_stiffness_matrix, mapOldNew);
	subSparseMatrix(*massMatrix, ordered_mass_matrix, mapOldNew);

	//solve eigen problem
	gen_eige_wrapper.SolveGenEigShInv(ordered_linear_stiffness_matrix,
		ordered_mass_matrix,
		numLMAmodes + rigid_modes_number,
		full_eigen_values.data(),
		full_eigen_vectors.data(),
		-1.0,
		0, 1);

	memcpy(eigen_values.data(), full_eigen_values.data(), sizeof(double)* (numLMAmodes + rigid_modes_number));
	memcpy(linearmodes.data(), full_eigen_vectors.data(), sizeof(double)* (numLMAmodes + rigid_modes_number) * vec_len);
	
	int numlinearmodes = numLMAmodes + rigid_modes_number;

	//constrained
	removedRigidLinearmodes = linearmodes.block(0,rigid_modes_number,vec_len,numLMAmodes);
	rigidmodes = linearmodes.block(0,0,vec_len,rigid_modes_number);
}

void SubspaceModesCreator::initLinearModes(MatrixXd* linearModes_)
{
	int dofs = forceModel->getr();
	int vec_len = dofs - numConstrainedDOFs;
	int constrained_nodes = numConstrainedDOFs / 3;

	if (constrained_nodes == 0)
	{
		rigid_modes_number = 6;
	}
	else if (constrained_nodes == 1)
	{
		rigid_modes_number = 3;
	}
	else if (constrained_nodes == 2)
	{
		rigid_modes_number = 1;
	}
	else if (constrained_nodes >= 3)
	{
		rigid_modes_number = 0;
	}

	int numLMAmodes = linearModes_->cols();

	linearmodes.resize(vec_len, numLMAmodes + rigid_modes_number);

	eigen_values.resize(numLMAmodes + rigid_modes_number);
	eigen_values.setConstant(1);
	linearmodes = *linearModes_;

	removedRigidLinearmodes = linearmodes.block(0, rigid_modes_number, vec_len, numLMAmodes);
	rigidmodes = linearmodes.block(0, 0, vec_len, rigid_modes_number);
}

void SubspaceModesCreator::modalDerivaties()
{
	double mineigenvalue = DBL_MAX;
	for (int i = rigid_modes_number; i < eigen_values.size(); i++)
	{
		eigen_values.data()[i] = std::abs(eigen_values.data()[i]);

		if (eigen_values.data()[i] < mineigenvalue)
		{
			mineigenvalue = eigen_values.data()[i];
		}
	}

	int dofs = forceModel->getr();
	int vec_len = dofs - numConstrainedDOFs;

	VectorXd u(dofs);
	u.setZero();
	forceModel->getTangentStiffnessMatrix(u.data(), tangentStiffnessMatrix);
	SparseMatrix<double> ordered_linear_stiffness_matrix_rest(vec_len, vec_len);
	subSparseMatrix(*tangentStiffnessMatrix, ordered_linear_stiffness_matrix_rest, mapOldNew);

#ifdef EIGEN_USE_MKL_ALL
	PardisoLDLT<SparseMatrix<double>> LDLT(ordered_linear_stiffness_matrix_rest);
#else
	SimplicialLDLT<SparseMatrix<double>> LDLT(ordered_linear_stiffness_matrix_rest);
#endif

	if (rigid_modes_number == 6)
	{
		// create left
		//double eigenvaluei = local_predata.eigen_values.data()[6 + i];
		SparseMatrix<double> lefth;// = ordered_linear_stiffness_matrix_rest - eigenvaluei*pre_data.Mass_M_;
	
		this->constraintsInsert(lefth, ordered_linear_stiffness_matrix_rest, rigidmodes);

#ifdef EIGEN_USE_MKL_ALL
		LDLT.compute(lefth);
#else
		LDLT.compute(lefth);
#endif

	}

	int num_modes_derivaties = ((k*k) + k) / 2;

	numDerivatesModes = num_modes_derivaties;

	derivativemodes.resize(vec_len, k + num_modes_derivaties);

	for (int i = 0; i < k; i++)
	{
		derivativemodes.col(i) = removedRigidLinearmodes.col(i);
	}

	int offset = k;
	

	for (int i = 0; i < k; i++)
	{

		VectorXd ordered_displace = removedRigidLinearmodes.col(i);
		VectorXd displace(dofs);
		displace.setZero();
		VectorInsertRows(mapOldNew, ordered_displace,displace);

		forceModel->getTangentStiffnessMatrix(displace.data(), tangentStiffnessMatrix, false, true, false);

		SparseMatrix<double> ordered_stiffnessmatrix(vec_len, vec_len);

		subSparseMatrix(*tangentStiffnessMatrix, ordered_stiffnessmatrix, mapOldNew);

		for (int j = i; j < k; j++)
		{
			std::cout << "Psi_" << i << "," << j << std::endl;
			std::cout << offset << std::endl;
			//compute H:phi_i*phi_j
			VectorXd orderd_displace_j = -ordered_stiffnessmatrix*removedRigidLinearmodes.col(j);

			if (rigid_modes_number == 6)
			{
				VectorXd lefh(vec_len+rigid_modes_number);
				lefh.setZero();
				lefh.block(0, 0, vec_len, 1) = orderd_displace_j;

				VectorXd result = LDLT.solve(lefh);
				derivativemodes.col(offset) = result.block(0, 0, vec_len, 1);
			}
			else
			{
				derivativemodes.col(offset) = LDLT.solve(orderd_displace_j);
			}

			offset++;
		}
	}

	// insert zeros

	MatrixXd fullmodes(dofs, k + num_modes_derivaties);
	fullmodes.setZero();

	for (int i = 0; i < mapOldNew.size(); i++)
	{
		if (mapOldNew[i] != -1)
		{
			fullmodes.row(i) = derivativemodes.row(mapOldNew[i]);
		}
	}

	//rigid modes
	if (rigid_modes_number != 0)
	{
		MatrixXd fullrigidmodes(dofs, rigid_modes_number);
		fullrigidmodes.setZero();

		for (int i = 0; i < mapOldNew.size(); i++)
		{
			if (mapOldNew[i] != -1)
			{
				fullrigidmodes.row(i) = rigidmodes.row(mapOldNew[i]);
			}
		}
		rigidmodes = fullrigidmodes;
	}

	derivativemodes = fullmodes;

	//Mass normlize
	for (int i = 0; i < derivativemodes.cols(); i++)
	{
		VectorXd vector = derivativemodes.col(i);
		matrixNormalize(massMatrix, &vector);
		derivativemodes.col(i) = vector;
	}

	//scale
	offset = k;
	for (int i = 0; i < k; i++)
	{
		double scale = mineigenvalue / eigen_values.data()[i];
		derivativemodes.col(i) *= scale;
		for (int j = i; j < k; j++)
		{
			double scale = (mineigenvalue*mineigenvalue) / eigen_values.data()[i] / eigen_values.data()[j];
			derivativemodes.col(offset) *= scale;
			offset++;
		}
	}

}

void SubspaceModesCreator::modesPCA(int r)
{
	int num_modes_derivaties = ((k*k) + k) / 2;
	if (r > derivativemodes.cols())
	{
		r = derivativemodes.cols();
	}

	//mass PCA
	if (!MatrixPCA(&derivativemodes, r, massMatrix))
	{
		std::cout << "mass-PCA error!" << std::endl;
	}
	else
	{
		r2 = r*r;
	}

}

void SubspaceModesCreator::constraintsInsert(SparseMatrix<double> &result, SparseMatrix<double> &K, MatrixXd &U)
{
	std::vector<EIGEN_TRI> reusltcoef;
	for (int j = 0; j < K.outerSize(); ++j)
		for (SparseMatrix<double>::InnerIterator it(K, j); it; ++it)
		{
			it.row();
			it.col();
			reusltcoef.push_back(EIGEN_TRI(it.row(), it.col(), it.value()));
		}

	for (int i = 0; i < U.rows(); i++)
	{
		for (int j = 0; j < U.cols(); j++)
		{
			int row = i;
			int col = K.cols() + j;
			reusltcoef.push_back(EIGEN_TRI(row, col, U.data()[j*U.rows() + i]));
			reusltcoef.push_back(EIGEN_TRI(col, row, U.data()[j*U.rows() + i]));
		}
	}
	result.resize(K.rows() + U.cols(), K.cols() + U.cols());
	result.setFromTriplets(reusltcoef.begin(), reusltcoef.end());
}

MatrixXd* SubspaceModesCreator::getLinearModes()
{
	return &linearmodes;
}

MatrixXd* SubspaceModesCreator::getDerivativeModes()
{
	return &derivativemodes;
}

Eigen::MatrixXd* SubspaceModesCreator::getNonRigidLinearModes()
{
	return &removedRigidLinearmodes;
}

MatrixXd* SubspaceModesCreator::getFinalModes()
{
	return &derivativemodes;
}

MatrixXd* SubspaceModesCreator::getRigidModes()
{
	return &rigidmodes;
}