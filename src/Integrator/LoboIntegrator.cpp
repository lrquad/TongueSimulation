#include "LoboIntegrator.h"
#include <iostream>
#include <fstream>

LoboIntegrator::LoboIntegrator(int r, double timestep, double dampingMassCoef , double dampingStiffnessCoef)
{
	this->showPerformance = false;
	this->r = r;
	this->timestep = timestep;
	this->dampingMassCoef = dampingMassCoef;
	this->dampingStiffnessCoef = dampingStiffnessCoef;
	internalForceScalingFactor = 1.0;
	step_ = 0;
	skipSteps = 1;

	q.resize(r);
	q_vel.resize(r); 
	q_accel.resize(r);

	q_1.resize(r);
	q_vel_1.resize(r);
	q_accel_1.resize(r);

	internalForces.resize(r);
	externalForces.resize(r);

	q_residual.resize(r);
	qdelta.resize(r);

	resetToRest();

	internalForces.setZero();
	externalForces.setZero();

	setUseLoadedStepResults(false);
	setRecordLoadedStepResults(false);
	setSaveStepResidual(false);
}

LoboIntegrator::~LoboIntegrator()
{

}

void LoboIntegrator::resetToRest()
{
	q.setZero();
	q_vel.setZero();
	q_accel.setZero();
	q_1.setZero();
	q_vel_1.setZero();
	q_accel_1.setZero();
	step_ = 0;
	sequence_q.clear();
}

void LoboIntegrator::resetToRestNoClearSequence()
{
	q.setZero();
	q_vel.setZero();
	q_accel.setZero();
	q_1.setZero();
	q_vel_1.setZero();
	q_accel_1.setZero();
	step_ = 0;
}

void LoboIntegrator::saveState(const char* filename)
{
	std::ofstream out(filename, std::ios::out | std::ios::binary | std::ios::trunc);
	out.precision(32);

	out.write((char*)(&r), sizeof(int));
	out.write((char*)q.data(), r*sizeof(VectorXd::Scalar));
	out.write((char*)q_vel.data(), r*sizeof(VectorXd::Scalar));
	out.write((char*)q_accel.data(), r*sizeof(VectorXd::Scalar));

	out.write((char*)q_1.data(), r*sizeof(VectorXd::Scalar));
	out.write((char*)q_vel_1.data(), r*sizeof(VectorXd::Scalar));
	out.write((char*)q_accel_1.data(), r*sizeof(VectorXd::Scalar));

	out.close();
}

void LoboIntegrator::readState(const char* filename)
{
	std::ifstream in(filename, std::ios::in | std::ios::binary);
	if (!in.good())
	{
		std::cout << "file not open" << std::endl;
		return;
	}
	int readr;

	in.read((char*)(&readr), sizeof(int));

	if (readr != r)
	{
		std::cout << "integrator file not matched. Please check dimension of the integrator." << std::endl;
	}

	in.read((char*)q.data(), r*sizeof(VectorXd::Scalar));
	in.read((char*)q_vel.data(), r*sizeof(VectorXd::Scalar));
	in.read((char*)q_accel.data(), r*sizeof(VectorXd::Scalar));

	in.read((char*)q_1.data(), r*sizeof(VectorXd::Scalar));
	in.read((char*)q_vel_1.data(), r*sizeof(VectorXd::Scalar));
	in.read((char*)q_accel_1.data(), r*sizeof(VectorXd::Scalar));

	in.close();
}

void LoboIntegrator::dampVelocity(double damp)
{
	this->q_vel *= damp;
}

void LoboIntegrator::resetVelAndAceel()
{
	q_vel.setZero();
	q_accel.setZero();
}

void LoboIntegrator::setExternalForces(double *externalForces_)
{
	memcpy(this->externalForces.data(), externalForces_, sizeof(double)*r);
}

void LoboIntegrator::addExternalForces(double *externalForces)
{
	for (int i = 0; i < r; i++)
	{
		this->externalForces.data()[i] += externalForces[i];
	}
}

void LoboIntegrator::setExternalForcesToZero()
{
	externalForces.setZero();
}

void LoboIntegrator::saveResults(const char* filename)
{
	std::ofstream outStream(filename);
	std::cout << sequence_q.size() << std::endl;
	outStream << sequence_q.size() << std::endl;
	for (int i = 0; i < sequence_q.size(); i++)
	{
		outStream << sequence_q[i].transpose() << std::endl;
	}
	outStream.close();
	std::cout << "Integrator loaded." << std::endl;
}

void LoboIntegrator::readResult(const char* filename)
{
	sequence_q.clear();
	std::ifstream inStream(filename);
	int dataSize;
	inStream >> dataSize;
	for (int i = 0; i < dataSize; i++)
	{
		VectorXd readdata(r);
		for (int j = 0; j < r; j++)
		{
			inStream >> readdata[j];
		}
		sequence_q.push_back(readdata);
	}
	inStream.close();
	std::cout << "Integrator saved." << std::endl;
}

void LoboIntegrator::saveResultsBinary(const char* filename)
{
	std::ofstream out(filename, std::ios::out | std::ios::binary | std::ios::trunc);
	out.precision(32);
	int sizeofsequence = sequence_q.size();
	std::cout << sizeofsequence << std::endl;
	out.write((char*)(&sizeofsequence), sizeof(int));
	for (int i = 0; i < sequence_q.size(); i++)
	{
		int vectorsize = sequence_q[i].size();
		out.write((char*)(&vectorsize), sizeof(int));
		out.write((char*)sequence_q[i].data(), vectorsize*sizeof(VectorXd::Scalar));
	}
	out.close();
}

void LoboIntegrator::readResultsBinary(const char* filename)
{
	sequence_q.clear();
	std::ifstream in(filename, std::ios::in | std::ios::binary);
	if (!in.good())
	{
		std::cout << "file not open" << std::endl;
		return;
	}

	int sizeofsequence;
	in.read((char*)(&sizeofsequence), sizeof(int));

	for (int i = 0; i < sizeofsequence; i++)
	{
		int vectorsize;
		in.read((char*)(&vectorsize), sizeof(int));
		VectorXd temp(vectorsize);
		in.read((char*)temp.data(), vectorsize*sizeof(VectorXd::Scalar));
		sequence_q.push_back(temp);
	}

	in.close();
}

void LoboIntegrator::setRecordLoadedStepResults(bool val)
{
	recordLoadedStepResults = val;
}

std::vector<VectorXd>* LoboIntegrator::getSequence_q_ref()
{
	return &sequence_q;
}

void LoboIntegrator::getExternalForces(double * externalForces_copy)
{
	memcpy(externalForces_copy,externalForces.data(),sizeof(double)*r);
}

void LoboIntegrator::setqState(const double *q, const double * qvel, const double* qaccel)
{
	memcpy(this->q.data(),q,sizeof(double)*r);
	if (qvel != NULL)
	{
		memcpy(this->q_vel.data(), qvel, sizeof(double)*r);
	}

	if (qaccel != NULL)
	{
		memcpy(this->q_accel.data(), qaccel, sizeof(double)*r);
	}
}

void LoboIntegrator::getqState(double * q, double * qvel, double * qaccel)
{
	if (q != NULL)
	{
		memcpy(q, this->q.data(), sizeof(double)*r);
	}
	if (qvel != NULL)
	{
		memcpy(qvel, this->q_vel.data(), sizeof(double)*r);
	}
	if (qaccel != NULL)
	{
		memcpy(qaccel, this->q_accel.data(), sizeof(double)*r);
	}
}
