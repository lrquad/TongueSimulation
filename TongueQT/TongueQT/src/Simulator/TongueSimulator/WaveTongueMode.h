#pragma once
#include "Render/LoboObjMesh.h"
#include "PointClouds/PointsCloud.h"

class LoboVolumetricMesh;
class LoboNeuralNetwork;
class WavMFCC;

class WaveTongueMode
{
public:
	WaveTongueMode();
	~WaveTongueMode();

	void movefowardCurframe();

	virtual void loadNormlizeMatrix(const char* filename);
	virtual void appendNewFrame(double* waveFrame);

	virtual void mapSensorDataSeed(LoboVolumetricMesh* volumetricmesh);

	virtual void insertFrame(double* sensorP);

	Vector3d getSensorPositionOri(int sensorId);
	Vector3d getSensorPosition(int sensorId, int frameid);
	Vector3d getCurFrame(int sensorId);
	Vector3d getNextFrame(int sensorId);


	int getNumSensor(){ return numSensors; };


	Vector3d getTranslate() const { return translate; }
	void setTranslate(Vector3d val) { translate = val; }
	double getScale() const { return scale; }
	void setScale(double val) { scale = val; }
	std::vector<int> getSensorseedindex() const { return sensorseedindex; }
	void setSensorseedindex(std::vector<int> val) { sensorseedindex = val; }
protected:

	std::vector<double> sensorData;

	Vector3d translate;
	double scale;


	LoboNeuralNetwork* neuralNetwork;
	WavMFCC* wavmfcc;

	VectorXd acf_mean;
	VectorXd acf_std;
	VectorXd arf_mean;
	VectorXd arf_std;

	//base sensor data
	int numSensors;
	int numFrame;
	int totalSensors;


	//map to volumetric node;
	std::vector<int> sensorseedindex;

	int curFrame;

};

