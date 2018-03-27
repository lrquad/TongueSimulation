#include "HTKInterface/WavMFCC.h"
#include "WaveTongueMode.h"
#include "LoboNeuralNetwork/LoboNeuralNetwork.h"
#include <fstream>
#include <iostream>
#include "LoboVolumetricMesh/LoboVolumetriceMeshCore.h"
WaveTongueMode::WaveTongueMode()
{
	neuralNetwork = new LoboNeuralNetwork();
	neuralNetwork->loadDNNV2("./data/tongue/DNNmatrix/tongueDNN.txt", "relu");
	loadNormlizeMatrix("./data/tongue/DNNmatrix/tongueChineseScore.txt");
	sensorData.clear();
	
	wavmfcc = new WavMFCC();

	totalSensors = 7;
	numSensors = 4;

	numFrame = 0;
	translate.setZero();
	scale = 1.0;

	curFrame = 0;
	insertFrame(arf_mean.data());

}


WaveTongueMode::~WaveTongueMode()
{
	delete neuralNetwork;
}

void WaveTongueMode::movefowardCurframe()
{
	curFrame++;
}

void WaveTongueMode::loadNormlizeMatrix(const char* filename)
{
	std::ifstream inputstream(filename);
	int x;
	int y;
	inputstream >> x >> y;
	acf_mean.resize(y, 1);
	for (int i = 0; i < y; i++)
	{
		inputstream >> acf_mean[i];
	}

	inputstream >> x >> y;
	acf_std.resize(y, 1);
	for (int i = 0; i < y; i++)
	{
		inputstream >> acf_std[i];
	}

	inputstream >> x >> y;
	arf_mean.resize(y, 1);
	for (int i = 0; i < y; i++)
	{
		inputstream >> arf_mean[i];
	}

	inputstream >> x >> y;
	arf_std.resize(y, 1);
	for (int i = 0; i < y; i++)
	{
		inputstream >> arf_std[i];
	}


	inputstream.close();
}

void WaveTongueMode::appendNewFrame(double* waveframe)
{
	int frsize = wavmfcc->getFrSize();
	for (int i = 0; i < frsize; i++)
	{
		//waveframe[i] *= 32768;
	}

	VectorXd features(39);
	features.setZero();
	wavmfcc->streamWavFrameToMFCC(waveframe, features.data());

	VectorXd inputVector(39 * 7);
	inputVector.setZero();
	int totalframes = wavmfcc->getTotalframescount();
	int col = 0;
	for (int k = -3; k <= 3; k++)
	{
		col = (k + 3) * 39;
		if (k + totalframes <= 0)
		{

		}
		else if (k + totalframes >= 1 && k + totalframes <= totalframes)
		{
			for (int j = 0; j < 39; j++)
			{
				inputVector.data()[col + j] = features.data()[j];
			}
		}
	}

	for (int i = 0; i < inputVector.size(); i++)
	{
		inputVector[i] = (inputVector[i] - acf_mean[i]) / acf_std[i];
	}

	VectorXd output = neuralNetwork->predictV2(inputVector);

	//14X1;
	//renormlize
	for (int i = 0; i < output.size(); i++)
	{
		output.data()[i] = output.data()[i] * arf_std.data()[i] + arf_mean.data()[i];
	}

	//output *= 2; //in my coordinate
	insertFrame(output.data());
}

void WaveTongueMode::mapSensorDataSeed(LoboVolumetricMesh* volumetricmesh)
{
	sensorseedindex.resize(numSensors);
	for (int i = 0; i < numSensors; i++)
	{
		Vector3d sensorposition = getSensorPositionOri(i);
		sensorseedindex[i] = volumetricmesh->searchCloseNode(sensorposition);
	}
}

void WaveTongueMode::insertFrame(double* sensorP)
{
	double offsetx = 0;
	if (numFrame>0)
	offsetx = sensorP[4 * 2 + 0] - arf_mean.data()[4 * 2 + 0];

	double offsety = 0;
	if (numFrame>0)
	offsety = sensorP[4 * 2 + 1] - arf_mean.data()[4 * 2 + 1];

	for (int i = 0; i < numSensors; i++)
	{
		double x = sensorP[i * 2 + 0] - offsetx;
		double y = -(sensorP[i * 2 + 1] - offsety);
		sensorData.push_back(x);
		sensorData.push_back(y);
	}

	numFrame++;
}

Vector3d WaveTongueMode::getSensorPositionOri(int sensorId)
{
	Vector3d p;
	p.setZero();

	p.data()[0] = 2*sensorData.data()[sensorId * 2 + 0];
	p.data()[1] = 2*sensorData.data()[sensorId * 2 + 1];

	p += translate;
	p *= scale;

	return p;
}

Vector3d WaveTongueMode::getSensorPosition(int sensorId, int frameid)
{
	Vector3d p;
	p.setZero();

	if (frameid < 0 || frameid >= numFrame)
	{
		frameid = numFrame;
	}

	p.data()[0] = 2*sensorData.data()[frameid*numSensors * 2 + sensorId * 2 + 0];
	p.data()[1] = 2*sensorData.data()[frameid*numSensors * 2 + sensorId * 2 + 1];

	p += translate;
	p *= scale;

	return p;
}

Vector3d WaveTongueMode::getCurFrame(int sensorId)
{
	Vector3d p;
	p.setZero();

	if (curFrame >= numFrame)
	{
		curFrame = numFrame-1;
	}

	p.data()[0] = 2 * sensorData.data()[curFrame*numSensors * 2 + sensorId * 2 + 0];
	p.data()[1] = 2 * sensorData.data()[curFrame*numSensors * 2 + sensorId * 2 + 1];

	p += translate;
	p *= scale;

	return p;

}

Vector3d WaveTongueMode::getNextFrame(int sensorId)
{
	Vector3d p;
	p.setZero();

	int nextFrame = curFrame + 1;
	if (nextFrame >= numFrame)
	{
		nextFrame = curFrame = numFrame-1;
	}

	p.data()[0] = 2 * sensorData.data()[nextFrame*numSensors * 2 + sensorId * 2 + 0];
	p.data()[1] = 2 * sensorData.data()[nextFrame*numSensors * 2 + sensorId * 2 + 1];

	p += translate;
	p *= scale;
	return p;
}

