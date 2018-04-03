#include "SubspaceModuleBase.h"
#include <iostream>
#include <fstream>
#include "EigenMatrixIO/EigenMatrixIO.h"

SubspaceModuleBase::SubspaceModuleBase()
{
	init();
}


SubspaceModuleBase::~SubspaceModuleBase()
{
}

void SubspaceModuleBase::loadSubspace(const char* filename)
{
	std::cout << "load subspace " << filename << std::endl;
	EigenMatrixIO::read_binary(filename, *ori_phi);

	*phi = *ori_phi;
	*pre_phi = *ori_phi;
	r = ori_phi->cols();
	ori_r = r;
}

void SubspaceModuleBase::saveSubspace(const char* filename)
{
	std::cout << "save subspace. " << filename << std::endl;
	EigenMatrixIO::write_binary(filename, *ori_phi);
}

void SubspaceModuleBase::saveSubspace(std::ofstream& outstream)
{
	int row, col;
	row = ori_phi->rows();
	col = ori_phi->cols();
	outstream.write((char*)&row, sizeof(int));
	outstream.write((char*)&col, sizeof(int));
	outstream.write((char*)ori_phi->data(), sizeof(MatrixXd::Scalar)*row*col);
}



void SubspaceModuleBase::loadSubspaceStream(std::ifstream& instream)
{
	int row, col;
	instream.read((char*)&row, sizeof(int));
	instream.read((char*)&col, sizeof(int));
	ori_phi->resize(row, col);
	instream.read((char*)ori_phi->data(), sizeof(MatrixXd::Scalar)*row*col);

	*phi = *ori_phi;
	*pre_phi = *ori_phi;
	r = ori_phi->cols();
	ori_r = r;
}

void SubspaceModuleBase::init()
{
	ori_phi = new MatrixXd();
	phi = new MatrixXd();
	pre_phi = new MatrixXd();
}

