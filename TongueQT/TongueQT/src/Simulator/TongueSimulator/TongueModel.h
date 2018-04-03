#pragma once
#include "Render/LoboObjMesh.h"
#include "PointClouds/PointsCloud.h"


class LoboNodeBase;
class LoboVolumetricMesh;
class LoboNeuralNetwork;
class WavMFCC;
class TongueModel
{
public:
	TongueModel();
	~TongueModel();

	void loadNormlizeMatrix(const char* filename);

	void appendNewFrame(double* waveframe);

	/**
	 * @brief
	 *
	 * 第一列是时间信息 x1,y1,x2,y2,...,x9,y9
	 *
	 * @param[in,out] filename filepath
	 */
	void readSensorData(const char* filename);
	void mapSensorDataSeed(LoboVolumetricMesh* volumetricmesh);

	void saveSensorData(const char* filename);

	void searchAndBuildKeyFrame(int numkeyframe);

	/**
	 * @brief search tongue points cloud to bind the sensor date to node index
	 */
	void searchNodeFromSensor();
	
	void readTongueMeshBinary(const char* filepath, Vector3d translate);
	void exporTongueMeshObj(const char* filepath);
	void readJawMeshBinary(const char* filepath, Vector3d translate);

	void generateSensorDataToCurrentMesh();

	

	/* --------------------------------
	query
	-------------------------------- */

	void saveSensorFrames(const char* filename);

	PointsCloud* getTonguePoints(){ return &tonguepoints; };
	PointsCloud* getJawPoints(){ return &jawpoints; }

	int getNumSensor(){return numSensor;}
	int getNumData(){ return numDataSize; }

	void setSensorPositionOri(Vector3d position, int sensorid, int timeid);
	Vector3d getSensorPositionOri(int sensorId,int timeid);
	Vector3d getSensorPosition(int sensorId,int timeid);
	void InsertSensorKeyFrame(std::vector<Vector3d> targetPosition, int num_interpolateFrames);
	void insertSensorKeyFrameByCurNode(int num_interpolateFrames);
	void clearFrames();

	Eigen::Vector3d getTranslate() const { return translate; }
	void setTranslate(Eigen::Vector3d val) { translate = val; updateSensorNodePosition(0);}
	double getScale() const { return scale; }
	void setScale(double val) { scale = val; updateSensorNodePosition(0);}

	std::vector<int> getKeyFrame(){ return keyframe;}
	bool isKeyFrame(int animaframeid);

	std::vector<int> getSensorseedindex() const { return sensorseedindex; }

	Eigen::Vector3d getCurSensorNodePosition(int sensorid);

	void updateSensorNodePosition(int step_index);
	void moveSensorNodePosition(int sensorid,Vector3d displacement);


protected:

	/**
	* @brief generate a sequence of positions that describe how the mesh move to target
	*
	* @param[in,out] target a list of vector3d for each sensor
	* @param[in,out] numofdata Put argument desc here
	* @return Put return information here
	*/
	void generateSensordataByTarget(std::vector<Vector3d> target, int numofdata);
	void searchKeyFrame(std::vector<int>& keyframeindex, int keyframenumber);
	void searchKeyFrameAverge(std::vector<int>& keyframeindex, int keyframenumber);
	void interpolateSensorData(int numInsertPose);
	void translateCoordinates();
	void removeNoise();

	PointsCloud tonguepoints;
	PointsCloud jawpoints;

	//Sensor data

	int numSensor;
	int numDataSize;
	int numKeyframe;

	std::vector<double> timeStamp;
	std::vector<int> keyframe;

	std::vector<Vector3d> noderesetposition;
	std::vector<int> sensorMapNode;

	std::vector<Vector3d> curSensorNodePosition;


	//map to volumetric node;
	std::vector<int> sensorseedindex;


	MatrixXd sensorData;
	std::vector<double> sensorDataList;

	Vector3d translate;
	double scale;

	LoboNeuralNetwork* neuralNetwork;
	WavMFCC* wavmfcc;

	VectorXd acf_mean;
	VectorXd acf_std;
	VectorXd arf_mean;
	VectorXd arf_std;


};

