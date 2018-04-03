#pragma once
#include <Eigen/Dense>
#include <Eigen/Sparse>
using namespace Eigen;

class CubatureDataSet;
class CubaturePartition;

class CubatureTraining
{
public:
	CubatureTraining();
	CubatureTraining(CubatureDataSet* cubaturedateset,CubaturePartition * cubaturepartition);
	CubatureTraining(const char* filename);
	~CubatureTraining();


	virtual void trainingData() = 0;


	virtual std::vector<int> getCubatureSet();
	virtual std::vector<double> getCubatureWeightSet();
	virtual std::vector<int>* getCubatureSetRef();
	virtual std::vector<double>* getCubatureWeightSetRef();

	inline void setThreshold(double value){ greedyThreshold = value; }

	//I/O
	virtual void saveCubatures(const char* filename);
	virtual void saveCubaturesBinary(std::ofstream &outstream);
	virtual void readCubaturesBinary(std::ifstream &instream);
	virtual int readCubatures(const char* filename);

protected:

	virtual void mergeCubaturePoints();

	int numCubatures;

	std::vector<int> cubatureIndices;
	std::vector<double> cubatureWeight;

	std::vector<std::vector<int>> partitionCubatureIndices;
	std::vector<std::vector<double>> partitionCubatureWeights;

	CubaturePartition* cubaturePartition;
	CubatureDataSet* cubatureDataSet;

	double greedyThreshold;

};

