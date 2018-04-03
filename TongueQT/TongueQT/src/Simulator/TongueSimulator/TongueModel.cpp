#include "Functions/spline.h"
#include "HTKInterface/WavMFCC.h"
#include "TongueModel.h"
#include <fstream>
#include "Functions/VectorOperation.h"
#include "Functions/findElementInVector.h"
#include "LoboVolumetricMesh/LoboVolumetricMesh.h"


#include <iostream>
#include "LoboNeuralNetwork/LoboNeuralNetwork.h"

#include <QElapsedTimer>

TongueModel::TongueModel()
{
	numDataSize = 0;
	numSensor = 0;
	translate = Vector3d::Zero();
	scale = 1.0;
	neuralNetwork = new LoboNeuralNetwork();
	//neuralNetwork->loadDNNV2("./data/tongue/DNNmatrix/tongueDNN.txt", "relu");

	//load matrix
	//loadNormlizeMatrix("./data/tongue/DNNmatrix/tongueChineseScore.txt");
	wavmfcc = new WavMFCC();

	//int size = 512;
	//double* wave = (double*)malloc(sizeof(double)*size);

	//std::ifstream test("./data/tongue/DNNmatrix/wavetest.txt");
	//for (int i = 0; i < size; i++)
	//{
	//	test >> wave[i];
	//}

	//this->appendNewFrame(wave);

}

TongueModel::~TongueModel()
{
	delete neuralNetwork;
	delete wavmfcc;
}

void TongueModel::loadNormlizeMatrix(const char* filename)
{
	std::ifstream inputstream(filename);
	int x;
	int y;
	inputstream >> x >> y;
	acf_mean.resize(y,1);
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

void TongueModel::appendNewFrame(double* waveframe)
{
	QElapsedTimer timer;
	timer.start();
	int frsize = wavmfcc->getFrSize();
	for (int i = 0; i < frsize; i++)
	{
		waveframe[i] *= 32768;
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
		else if (k+totalframes >=1 && k+totalframes<=totalframes)
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

	output *= 2; //in my coordinate



}

void TongueModel::readSensorData(const char* filename)
{
	std::cout << "sensor data :" << filename << std::endl;
	int row, col;
	std::ifstream in(filename);
	in >> row >> col;
	numDataSize = row;
	numSensor = (col - 1) / 2;

	sensorData.resize(row, col);
	sensorData.setZero();

	for (int i = 0; i < row; i++)
	{
		for (int j = 0; j < col; j++)
		{
			in >> sensorData.data()[j*row + i];
		}
	}
	in.close();

	//test
	//for (int i = 1; i < row; i++)
	//{
	//	for (int j = 0; j < col; j++)
	//	{
	//		sensorData.data()[j*row + i] = sensorData.data()[j*row+0];
	//	}
	//}

	//for (int i = 1; i < row; i++)
	//{
	//	//sensorData.data()[8 * row + i] += i*0.1;
	//	//sensorData.data()[6 * row + i] -= i*0.1;
	//	sensorData.data()[4 * row + i] -= i*0.1;
	//	//sensorData.data()[2 * row + i] -= i*0.1;
	//}

	std::cout << "interpolateSensorData" << std::endl;
	interpolateSensorData(1);
	std::cout << "translateCoordinates" << std::endl;

	translateCoordinates();
	std::cout << "removeNoise" << std::endl;

	//removeNoise();
	std::cout << "searchNodeFromSensor" << std::endl;

	//searchNodeFromSensor();
	std::cout << "updateSensorNodePosition" << std::endl;

	updateSensorNodePosition(0);

	std::cout << numSensor << " sensors." << std::endl;
	std::cout << numDataSize << " numDataSize." << std::endl;

}

void TongueModel::mapSensorDataSeed(LoboVolumetricMesh* volumetricmesh)
{
	sensorseedindex.resize(numSensor);
	for (int i = 0; i < numSensor; i++)
	{
		Vector3d sensorposition = getSensorPosition(i, 0);
		if (i == 0)
		{
			sensorposition.y() -= 0.5;
		}

		sensorseedindex[i] = volumetricmesh->searchCloseNode(sensorposition);
	}
}

void TongueModel::saveSensorData(const char* filename)
{
	std::ofstream test(filename);
	test << sensorData.rows() << " " << sensorData.cols() << std::endl;
	test << sensorData << std::endl;
	test.close();
}

void TongueModel::searchAndBuildKeyFrame(int numkeyframes)
{
	this->numKeyframe = numkeyframes;
	searchKeyFrameAverge(keyframe, numKeyframe);
}

void TongueModel::searchNodeFromSensor()
{
	int insertSensor = 1;
	int extendNumSensor = numSensor + insertSensor;

	sensorMapNode.resize(extendNumSensor);
	this->noderesetposition.resize(extendNumSensor);

	std::vector<double> X(4), Y(4);
	std::vector<Vector3d> orisensorposition;
	for (int i = 0; i < 4; i++)
	{
		Vector3d resetposition = this->getSensorPositionOri(i, 0);

		X[3-i] = resetposition.data()[0];
		Y[3-i] = resetposition.data()[1];

		orisensorposition.push_back(resetposition);
	}

	tk::spline s;
	s.set_points(X, Y);

	Vector3d mid = (orisensorposition[1] + orisensorposition[2]) / 2;
	double y  =  s(mid.x());
	Vector3d temp(mid.x(),y,0);
	orisensorposition.push_back(temp);

	for (int i = 4; i < numSensor; i++)
	{
		Vector3d resetposition = this->getSensorPositionOri(i, 0);
		orisensorposition.push_back(resetposition);
	}

	for (int i = 0; i < extendNumSensor; i++)
	{
		Vector3d resetposition = orisensorposition[i];
		int nodeid = tonguepoints.searchColseNode(resetposition);
		sensorMapNode[i] = nodeid;
		noderesetposition[i] = tonguepoints.getPointOriPosition(nodeid);
	}

}

void TongueModel::searchKeyFrame(std::vector<int>& keyframeindex, int keyframenumber)
{
	keyframeindex.resize(keyframenumber);
	std::vector<int> keyframeFromSensor(keyframenumber);
	for (int i = 0; i < keyframenumber; i++)
	{
		keyframeindex[i] = -1;
	}

	VectorXd gradientnorm(numDataSize*4);
	VectorXd gradientgradient(numDataSize*4);

	//only consider first 4 sensor data
	for (int i = 0; i < 4; i++)
	{
		VectorXd x = sensorData.col(i * 2 + 1);
		VectorXd y = sensorData.col(i * 2 + 1+1);
		VectorXd positionnorm(sensorData.rows());
		for (int j = 0; j < sensorData.rows(); j++)
		{
			positionnorm[j] = sqrt(x[j]*x[j]+y[j]*y[j]);
		}

		VectorXd dx = computeVectorGradient(positionnorm);

		//VectorXd dy = computeVectorGradient(y);
		VectorXd ddx = computeVectorGradient(dx);
		//VectorXd ddy = computeVectorGradient(dy);


		for (int j = 0; j < numDataSize; j++)
		{
			double x = ddx.data()[j];
			gradientnorm.data()[i*numDataSize + j] = std::abs(x);
			gradientgradient.data()[i*numDataSize + j] = ddx[j];
		}
	}

	std::vector<int> sortedidx = sortVector(gradientnorm, 0);
	
	//for (int i = 0; i < sortedidx.size(); i++)
	//{
	//	std::cout << gradientnorm[sortedidx[i]] << std::endl;
	//}
	//std::cout << "---------------------" << std::endl;

	int offset = 0;

	for (int i = 0; i < sortedidx.size(); i++)
	{
		if (findElement(keyframeindex, sortedidx[i] % numDataSize))
		{
			continue;
		}

		keyframeindex[offset] = sortedidx[i] % numDataSize;
		keyframeFromSensor[offset] = sortedidx[i] / numDataSize;

		offset++;
		if (offset == keyframenumber)
		{
			break;
		}
	}
	

	//std::ofstream output("keyframe.txt");
	//output.precision(32);
	//for (int i = 0; i < keyframeindex.size(); i++)
	//{
	//	output << keyframeindex[i] << " " << keyframeFromSensor[i] << std::endl;
	//}
	//output.close();

	//output.open("sensordata.txt");
	//output << sensorData << std::endl;
	//output.close();

}

void TongueModel::searchKeyFrameAverge(std::vector<int>& keyframeindex, int keyframenumber)
{
	keyframeindex.resize(keyframenumber);
	numDataSize = 200;
	int step = numDataSize / keyframenumber-1;
	int offset = 0;
	for (int i = 0; i < keyframenumber; i++)
	{
		offset += step;
		keyframeindex[i] = offset;
		std::cout << keyframeindex[i] << std::endl;
	}
}

void TongueModel::readTongueMeshBinary(const char* filepath, Vector3d translate)
{
	translate.setZero();
	tonguepoints.readPointsBinary(filepath, translate);
}

void TongueModel::exporTongueMeshObj(const char* filepath)
{
	tonguepoints.saveObj(filepath);
}

void TongueModel::readJawMeshBinary(const char* filepath, Vector3d translate)
{
	translate.setZero();
	jawpoints.readPointsBinary(filepath, translate);
}

void TongueModel::generateSensorDataToCurrentMesh()
{
	std::vector<Vector3d> target(4);
	for (int i = 0; i < 4; i++)
	{
		int nodeid = this->sensorMapNode[i];
		Vector3d nodeposition = tonguepoints.getPointOriPosition(nodeid);
		target[i] = nodeposition;
	}
	generateSensordataByTarget(target, 50);
}

void TongueModel::generateSensordataByTarget(std::vector<Vector3d> target, int numofdata)
{
	//compute steps
	this->numSensor = target.size();
	std::vector<Vector2d> resetposition;
	for (int i = 0; i < numSensor; i++)
	{
		Vector2d resetposition_2d;
		Vector3d resetposition_3d = noderesetposition[i];
		resetposition_2d.data()[0] = resetposition_3d.data()[0];
		resetposition_2d.data()[1] = resetposition_3d.data()[1];
		resetposition.push_back(resetposition_2d);
	}
	std::vector<Vector2d> displacement_step;

	for (int i = 0; i < numSensor; i++)
	{
		Vector2d target_2d(target[i].x(),target[i].y());
		Vector2d step = (target_2d - resetposition[i]) / (numofdata-1);
		displacement_step.push_back(step);
	}

	
	this->numDataSize = numofdata;
	this->sensorData.resize(numofdata, numSensor * 2 + 1);
	sensorData.setZero();
	
	//set first position
	for (int i = 0; i < numSensor; i++)
	{
		int col = i * 2 + 1;
		sensorData.data()[col*numofdata+0]  = resetposition[i].data()[0]/2.0;
		sensorData.data()[(col + 1)*numofdata + 0] = resetposition[i].data()[1] / -2.0;;
	}

	for (int i = 1; i < numofdata; i++)
	{
		for (int j = 0; j < numSensor; j++)
		{
			int col = j * 2 + 1;
			sensorData.data()[col*numofdata + i] = (resetposition[j].data()[0]+displacement_step[j].data()[0]*i)/2.0;
			sensorData.data()[(col + 1)*numofdata + i] = (resetposition[j].data()[1] + displacement_step[j].data()[1]*i)/-2.0;
		}
	}

	std::cout << "sensor data generated." << std::endl;
	std::cout << numofdata << " numofdata, " << numSensor << " numSensor." << std::endl;
}



void TongueModel::saveSensorFrames(const char* filename)
{
	std::ofstream output(filename);
	int numframe = numSensor;
	output << numframe << std::endl;
	for (int i = 0; i < numframe; i++)
	{
		output << i << " " << getSensorPositionOri(i, 0).transpose() << std::endl;
	}
	output.close();
}

void TongueModel::setSensorPositionOri(Vector3d position, int sensorId, int timeid)
{
	int row = timeid;
	int col = sensorId * 2 + 1;
	sensorData.data()[col*sensorData.rows() + row] = position.data()[0] / 2.0;
	col++;
	sensorData.data()[col*sensorData.rows() + row] = position.data()[1] / -2.0;
}

Eigen::Vector3d TongueModel::getSensorPositionOri(int sensorId, int timeid)
{
	Vector3d position;
	int row = timeid;
	int col = sensorId * 2 + 1;
	position.data()[0] = 2.0*sensorData.data()[col*sensorData.rows() + row];
	col++;
	position.data()[1] = -2.0*sensorData.data()[col*sensorData.rows() + row];
	position.data()[2] = 0;
	return position;
}

Eigen::Vector3d TongueModel::getSensorPosition(int sensorId, int timeid)
{
	if (timeid >= numDataSize)
	{
		timeid = numDataSize - 1;
	}

	Vector3d position;
	int row = timeid;
	int col = sensorId * 2+1;
	position.data()[0] = 2.0*sensorData.data()[col*sensorData.rows() + row];
	col++;
	position.data()[1] = -2.0*sensorData.data()[col*sensorData.rows() + row];
	position.data()[2] = 0;
	
	position += translate;
	position *= scale;

	return position;
}

void TongueModel::InsertSensorKeyFrame(std::vector<Vector3d> targetPosition, int num_interpolateFrames)
{
	std::cout << "InsertSensorKeyFrame : " << std::endl;

	MatrixXd newSensorData;
	int rows = sensorData.rows() + num_interpolateFrames;
	int cols = sensorData.cols();
	newSensorData.resize(rows, cols);
	newSensorData.setZero();
	newSensorData.block(0, 0, sensorData.rows(), sensorData.cols()) = sensorData;

	int numEMAWeCare = targetPosition.size();

	for (int i = 0; i < numEMAWeCare; i++)
	{
		Vector3d step;
		step.setZero();
		Vector3d ori = this->getSensorPosition(i, sensorData.rows()-1);
		step = (targetPosition[i] - ori) / num_interpolateFrames;

		for (int j = 0; j < num_interpolateFrames; j++)
		{
			int row = j+sensorData.rows();
			int col = i * 2 + 1;

			Vector3d cur_position = ori + step*j;
			//transfor to ori coordinate
			cur_position /= scale;
			cur_position -= translate;
			cur_position.data()[0] /= 2.0;
			cur_position.data()[1] /= -2.0;

			newSensorData.data()[col*rows + row] = cur_position.data()[0];
			newSensorData.data()[(col+1)*rows + row] = cur_position.data()[1];
		}
	}

	sensorData = newSensorData;
	numDataSize = sensorData.rows();

}

void TongueModel::insertSensorKeyFrameByCurNode(int num_interpolateFrames)
{
	InsertSensorKeyFrame(curSensorNodePosition, num_interpolateFrames);
}

void TongueModel::clearFrames()
{
	MatrixXd newSensorData = sensorData.block(0, 0, 1, sensorData.cols());
	sensorData = newSensorData;
	numDataSize = sensorData.rows();
}

bool TongueModel::isKeyFrame(int animaframeid)
{
	return findElement(keyframe, animaframeid);
}

Eigen::Vector3d TongueModel::getCurSensorNodePosition(int sensorid)
{
	return curSensorNodePosition[sensorid];
}


void TongueModel::updateSensorNodePosition(int step_index)
{
	curSensorNodePosition.resize(numSensor);
	for (int i = 0; i < numSensor; i++)
	{
		curSensorNodePosition[i] = this->getSensorPosition(i,step_index);
	}
}

void TongueModel::moveSensorNodePosition(int sensorid, Vector3d displacement)
{
	displacement.z() = 0;
	curSensorNodePosition[sensorid] += displacement;
}

void TongueModel::interpolateSensorData(int numInsertPose)
{
	MatrixXd newSensorData;
	
	int row, col;
	row = (numDataSize-1)*(numInsertPose+1)+1;
	col = numSensor*2+1;

	newSensorData.resize(row, col);
	newSensorData.setZero();

	for (int i = 0; i < row; i++)
	{
		if (i % (numInsertPose + 1) == 0)
		{
			for (int j = 1; j < col; j++)
			{
				int oldrow = i / (numInsertPose + 1);
				int col = j;
				newSensorData.data()[col*newSensorData.rows() + i] = sensorData.data()[col*sensorData.rows() + oldrow];
			}
			continue;
		}
	}

	for (int i = 0; i < numDataSize-1; i++)
	{
		int oldrow = i;
		int nextoldrow = i + 1;

		for (int j = 0; j < numSensor; j++)
		{
			Vector2d perdistance;
			Vector2d startposition, stopposition;
			
			int col = j * 2 + 1 + 0;
			startposition.data()[0] = sensorData.data()[col*sensorData.rows()+oldrow];
			startposition.data()[1] = sensorData.data()[(col + 1)*sensorData.rows() + oldrow];

			stopposition.data()[0] = sensorData.data()[col*sensorData.rows() + nextoldrow];
			stopposition.data()[1] = sensorData.data()[(col + 1)*sensorData.rows() + nextoldrow];

			perdistance = (stopposition - startposition) / (1 + numInsertPose);

			for (int k = 1; k <= numInsertPose; k++)
			{
				Vector2d curposition = k*perdistance + startposition;
				int newrow = oldrow*(numInsertPose+1) + k;
				newSensorData.data()[col*newSensorData.rows() + newrow] = curposition.data()[0];
				newSensorData.data()[(col + 1)*newSensorData.rows() + newrow] = curposition.data()[1];
			}
		}
	}

	numDataSize = row;
	numSensor = (col - 1) / 2;

	sensorData = newSensorData;
}

void TongueModel::translateCoordinates()
{
	if (numSensor >= 5)
	{
		//will based on the num 5's coordinate
		for (int i = 0; i < numDataSize; i++)
		{
			Vector3d offset = this->getSensorPositionOri(4, i) - this->getSensorPositionOri(4, 0);
			for (int j = 0; j < 4; j++)
			{
				Vector3d newposition = this->getSensorPositionOri(j, i) - offset;
				setSensorPositionOri(newposition, j, i);
			}
		}
	}
}

void TongueModel::removeNoise()
{
	int k = numDataSize / 10;
	int rows = sensorData.rows();
	int cols = sensorData.cols();
	for (int i = 1; i < cols; i++)
	{
		lowPassFilter(&sensorData.data()[i*rows], rows, 0.3);
	}


}

