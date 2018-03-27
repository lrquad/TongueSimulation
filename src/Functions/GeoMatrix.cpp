#include "GeoMatrix.h"
#include <QQuaternion>
#include <QMatrix4x4>
#include <random>
#include <iostream>
#include <time.h>
#include <fstream>

void computeShearMatrix(double x, double y, double z, Matrix3d &matrix)
{
	matrix.setIdentity();
	matrix.data()[1] = x;
	matrix.data()[2] = z;
	matrix.data()[3] = y;

	//matrix.data()[5] = y;
	//matrix.data()[6] = z;
	//matrix.data()[7] = x;
}

void computeRotationMatrix(double x, double y, double z, Matrix3d &matrix)
{
	Quaternion<double> quaternion(1, x, y, z);
	matrix = quaternion.matrix();
}

void PCAMatrix(MatrixXd &inputMatrix, MatrixXd &outputMatrix, MatrixXd &outputPCATransform, int pcadimension, RowVectorXd &scalingFactors, RowVectorXd &minCoeffs, RowVectorXd &featureMeans)
{
	MatrixXd dataset = inputMatrix;

	RowVectorXd maxCoeffs = dataset.colwise().maxCoeff();
	minCoeffs = dataset.colwise().minCoeff();
	scalingFactors = maxCoeffs - minCoeffs;
	for (int i = 0; i < scalingFactors.size(); i++)
	{
		if (std::abs(scalingFactors[i]) < 1e-15)
		{
			scalingFactors[i] = 1;
		}
	}

	dataset = (dataset.rowwise() - minCoeffs).array().rowwise() / scalingFactors.array();

	featureMeans = dataset.colwise().mean();
	 dataset = dataset.rowwise() - featureMeans;

	// Compute the covariance matrix.
	Eigen::MatrixXd cov = dataset.adjoint() * dataset;
	cov = cov / (dataset.rows() - 1);

	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(cov);
	// Normalize eigenvalues to make them represent percentages.
	Eigen::VectorXd normalizedEigenValues = eig.eigenvalues() / eig.eigenvalues().sum();

	// Get the two major eigenvectors and omit the others.
	Eigen::MatrixXd evecs = eig.eigenvectors();
	outputPCATransform = evecs.rightCols(pcadimension);

	// Map the dataset in the new two dimensional space.
	outputMatrix = dataset * outputPCATransform;
}

void sampleRotationMatrix(double longitude, double latitude , double angle, Matrix3d &matrix)
{
	double x, y, z;
	x = std::cos(longitude);
	y = std::sin(longitude);
	z = std::sin(latitude);

	double xyscale = std::cos(latitude);
	x *= xyscale;
	y *= xyscale;
	/*QQuaternion rotation = QQuaternion::fromAxisAndAngle(QVector3D(x, y, z).normalized(), angle);
	rotation.normalize();
	QMatrix3x3 rmatrix = rotation.toRotationMatrix();


	for (int i = 0; i < 9; i++)
	{
	matrix.data()[i] = rmatrix.data()[i];
	}*/
	AngleAxis<double> aa(angle, Vector3d(x, y, z).normalized());
	Matrix3d rmatrix_ = aa.toRotationMatrix();
	matrix = rmatrix_;
}

void smapleRoataionVector(double longitude, double latitude, double angle, Vector3d &rotationvector)
{
	double x, y, z;

	x = std::cos(longitude);
	y = std::sin(longitude);
	z = std::sin(latitude);

	double xyscale = std::cos(latitude);
	x *= xyscale;
	y *= xyscale;
	/*QQuaternion rotation = QQuaternion::fromAxisAndAngle(QVector3D(x, y, z).normalized(), angle);
	rotation.normalize();
	QMatrix3x3 rmatrix = rotation.toRotationMatrix();


	for (int i = 0; i < 9; i++)
	{
	matrix.data()[i] = rmatrix.data()[i];
	}*/
	rotationvector = Vector3d(x, y, z).normalized()*angle;

}

void sampleRotationVector(double longitude, double latitude, double angle, Vector3d &rotationvector)
{
	double x, y, z;
	//x = std::cos(longitude);
	//y = std::sin(longitude);
	//z = std::sin(latitude);

	x = std::sin(latitude);
	y = std::cos(longitude);
	z = std::sin(longitude);

	double yzscale = std::cos(latitude);
	y *= yzscale;
	z *= yzscale;
	/*QQuaternion rotation = QQuaternion::fromAxisAndAngle(QVector3D(x, y, z).normalized(), angle);
	rotation.normalize();
	QMatrix3x3 rmatrix = rotation.toRotationMatrix();
	for (int i = 0; i < 9; i++)
	{
	matrix.data()[i] = rmatrix.data()[i];
	}*/
	rotationvector = Vector3d(x, y, z).normalized()*angle;
}

void sampleShearingScaleMatrix(double shearingRatio, Matrix3d& matrix,int i)
{
	matrix.setRandom();
	std::default_random_engine generator;
	generator.seed(time(NULL) + i);
	std::uniform_real_distribution<double> scaledouble(-shearingRatio, shearingRatio);
	matrix = matrix.transpose()*matrix;
	/*double scale = scaledouble(generator);
	matrix *= scale;*/
	//matrix.data()[0] = scaledouble(generator);
	//matrix.data()[1] = scaledouble(generator);
	//matrix.data()[2] = scaledouble(generator);
	//matrix.data()[4] = scaledouble(generator);
	//matrix.data()[5] = scaledouble(generator);
	//matrix.data()[8] = scaledouble(generator);

	//matrix.data()[3] = matrix.data()[1];
	//matrix.data()[6] = matrix.data()[2];
	//matrix.data()[7] = matrix.data()[5];
}

double uniformSampleRotation(int seed, double maxlatitude, double minlatittude, double maxlongitude, double minlongitude, double maxangle, double minangle, Matrix3d &rotationmatrix, Vector3d &rotationVector)
{
	std::default_random_engine generator;
	generator.seed(seed);
	std::uniform_real_distribution<double> latitude_distribution(minlatittude, maxlatitude);
	std::uniform_real_distribution<double> longitude_distribution(minlongitude,maxlongitude);
	std::uniform_real_distribution<double> angle_distribution(minangle,maxangle);

	double Latitude = latitude_distribution(generator);
	double Longtitude = longitude_distribution(generator);
	double Angle = angle_distribution(generator);

	sampleRotationMatrix(Latitude, Longtitude, Angle, rotationmatrix);

	double x, y, z;
	x = std::cos(Latitude);
	y = std::sin(Latitude);
	z = std::sin(Longtitude);

	double xyscale = std::cos(Longtitude);
	x *= xyscale;
	y *= xyscale;

	rotationVector.data()[0] = x;
	rotationVector.data()[1] = y;
	rotationVector.data()[2] = z;

	return Angle;
}

void generatePolarDecompositionData(double maxlongitude, double minlongitude, int num_longitude, double maxLatitude, double minLatitude, int num_latitude, double maxangle, double minangle, int num_angle, double maxshearing, double minshearing, int num_shearing, std::vector<double> &origin_data, std::vector<double> &target_data, std::vector<double> &axis_list)
{
	std::vector<double> sample_latitude(num_latitude);
	std::vector<double> sample_longitude(num_longitude);
	std::vector<double> sample_angle(num_angle);
	std::vector<double> sample_shearing(num_shearing);

	origin_data.clear();
	target_data.clear();

	double interval_;
	
	interval_ = (maxLatitude - minLatitude) / ((double)num_latitude+1);
	for (int i = 1; i < num_latitude+1; i++)
	{
		sample_latitude[i - 1] = interval_*i + minLatitude;
	}

	interval_ = (maxlongitude - minlongitude) / ((double)num_longitude+1);
	for (int i = 1; i < num_longitude+1; i++)
	{
		sample_longitude[i - 1] = interval_*i + minlongitude;
	}

	interval_ = (maxangle - minangle) / ((double)num_angle+1);
	for (int i = 1; i < num_angle+1; i++)
	{
		sample_angle[i - 1] = interval_*i + minangle;
	}

	interval_ = (maxshearing - minshearing) / ((double)num_shearing+1);
	for (int i = 1; i < num_shearing+1; i++)
	{
		sample_shearing[i - 1] = interval_*i + minshearing;
	}

	Matrix3d rotationMatrix;
	Matrix3d shearScaleMatrix;

	int index = 0;
	for (int i = 0; i < num_latitude; i++)
	{
		for (int j = 0; j < num_longitude; j++)
		{
			for (int k = 0; k < num_angle; k++)
			{
				for (int l = 0; l < num_shearing; l++)
				{
					sampleRotationMatrix(sample_longitude[j], sample_latitude[i], sample_angle[k], rotationMatrix);
					sampleShearingScaleMatrix(sample_shearing[l], shearScaleMatrix, l);

					Matrix3d inputMatrix = rotationMatrix*shearScaleMatrix;
					srand(time(NULL) + index);
					index++;
					//inputMatrix.setRandom();
					inputMatrix.normalize();


					AngleAxisd aa;
					aa = rotationMatrix;
					Vector3d axis = aa.axis();
					
					/*Affine3d af;
					af = inputMatrix;

					shearScaleMatrix = af.rotation().transpose()*inputMatrix;

					inputMatrix = rotationMatrix*shearScaleMatrix;
					inputMatrix.normalize();
					af = inputMatrix;
					shearScaleMatrix = af.rotation().transpose()*inputMatrix;*/

					if (inputMatrix.determinant() < 1e-3)
					{
						continue;
					}

					if (std::abs(aa.angle()) < 1e-1)
					{
						continue;
					}

					for (int a = 0; a < 9; a++)
					{
						origin_data.push_back(inputMatrix.data()[a]);
						target_data.push_back(rotationMatrix.data()[a]);
					}

					for (int a = 0; a < 3; a++)
					{
						axis_list.push_back(axis.data()[a]);
					}
					axis_list.push_back(aa.angle());
					axis_list.push_back(inputMatrix.determinant());
				}
			}
		}
	}


}



void generateRotationVectorSample(double maxlongitude, double minlongitude, int num_longitude, double maxLatitude, double minLatitude, int num_latitude, double maxangle, double minangle, int num_angle,   std::vector<Vector3d> &rotationVectorList)
{
	std::vector<double> sample_latitude(num_latitude);
	std::vector<double> sample_longitude(num_longitude);
	std::vector<double> sample_angle(num_angle);

	double interval_;

	interval_ = (maxLatitude - minLatitude) / ((double)num_latitude - 1);
	for (int i = 0; i < num_latitude ; i++)
	{
		sample_latitude[i] = interval_*i + minLatitude;
	}

	interval_ = (maxlongitude - minlongitude) / ((double)num_longitude - 1);
	for (int i = 0; i < num_longitude; i++)
	{
		sample_longitude[i] = interval_*i + minlongitude;
	}

	
	if (num_angle == 1)
	{
		interval_ = (maxangle - minangle);
	}
	else
	{
		interval_ = (maxangle - minangle) / ((double)num_angle - 1);
	}
	for (int i = 0; i < num_angle; i++)
	{
		sample_angle[i] = interval_*i + minangle;
	}

	Matrix3d rotationMatrix;
	Vector3d rotationVector;
	rotationVectorList.clear();
	int index = 0;
	for (int i = 0; i < num_latitude; i++)
	{
		for (int j = 0; j < num_longitude; j++)
		{
			for (int k = 0; k < num_angle; k++)
			{
				
				smapleRoataionVector(sample_longitude[j], sample_latitude[i], sample_angle[k], rotationVector); 
				rotationVectorList.push_back(rotationVector);
			}

			if (std::abs(sample_latitude[i] - M_PI / 2.0) < 0.001)
			{
				break;
			}
		}
	}
}

void generateRotationVectorSample(double maxlongitude, double minlongitude, int num_longitude, double maxLatitude, double minLatitude, int num_latitude, double maxangle, double minangle, int num_angle, std::vector<Vector3d> &rotationVectorList, std::vector<bool> &reset)
{
	std::vector<double> sample_latitude(num_latitude);
	std::vector<double> sample_longitude(num_longitude);
	std::vector<double> sample_angle(num_angle);

	double interval_;

	if (num_latitude == 1)
	{
		interval_ = (maxLatitude - minLatitude);
	}
	else
	{
		interval_ = (maxLatitude - minLatitude) / ((double)num_latitude - 1);
	}

	for (int i = 0; i < num_latitude; i++)
	{
		sample_latitude[i] = interval_*i + minLatitude;
	}

	interval_ = (maxlongitude - minlongitude) / ((double)num_longitude - 1);
	for (int i = 0; i < num_longitude; i++)
	{
		sample_longitude[i] = interval_*i + minlongitude;
	}


	if (num_angle == 1)
	{
		interval_ = (maxangle - minangle);
	}
	else
	{
		interval_ = (maxangle - minangle) / ((double)num_angle - 1);
	}
	for (int i = 0; i < num_angle; i++)
	{
		sample_angle[i] = interval_*i + minangle;
	}

	Matrix3d rotationMatrix;
	Vector3d rotationVector;
	rotationVectorList.clear();
	int index = 0;
	for (int i = 0; i < num_latitude; i++)
	{
		for (int j = 0; j < num_longitude; j++)
		{
			for (int k = 0; k < num_angle; k++)
			{

				sampleRotationVector(sample_longitude[j], sample_latitude[i], sample_angle[k], rotationVector);

				rotationVectorList.push_back(rotationVector);

				if (k == 0)
				{
					reset.push_back(true);

				}else
				reset.push_back(false);
			}

			if (std::abs(sample_latitude[i] - M_PI / 2.0) < 0.001)
			{
				break;
			}
		}
	}
}

void generateRotationVectorSample(double maxlongitude, double minlongitude, int num_longitude, double maxLatitude, double minLatitude, int num_latitude, double maxangle, double minangle, int num_angle, std::vector<Vector3d> &rotationVectorList, std::vector<bool> &reset, std::vector<Vector3d> &potentialdirection)
{
	std::vector<double> sample_latitude(num_latitude);
	std::vector<double> sample_longitude(num_longitude);
	std::vector<double> sample_angle(num_angle);

	double interval_;

	if (num_latitude == 1)
	{
		interval_ = (maxLatitude - minLatitude);
	}
	else
	{
		interval_ = (maxLatitude - minLatitude) / ((double)num_latitude - 1);
	}

	for (int i = 0; i < num_latitude; i++)
	{
		sample_latitude[i] = interval_*i + minLatitude;
	}


	if (num_longitude == 1)
	{
		interval_ = (maxlongitude - minlongitude);
	}
	else
	{
		interval_ = (maxlongitude - minlongitude) / ((double)num_longitude - 1);
	}

	for (int i = 0; i < num_longitude; i++)
	{
		sample_longitude[i] = interval_*i + minlongitude;
	}


	if (num_angle == 1)
	{
		interval_ = (maxangle - minangle);
	}
	else
	{
		interval_ = (maxangle - minangle) / ((double)num_angle - 1);
	}
	for (int i = 0; i < num_angle; i++)
	{
		sample_angle[i] = interval_*i + minangle;
	}

	Vector3d rotationVector;
	rotationVectorList.clear();
	int index = 0;
	bool toppointsset = false;
	for (int i = 0; i < num_longitude; i++)
	{
		for (int j = 0; j <num_latitude; j++)
		{
			if (std::abs(std::abs(sample_latitude[j]) - M_PI / 2.0) < 0.001)
			{
				if (toppointsset == true)
				{
					continue;
				}
				else
				{
					toppointsset = true;
				}
			}

			for (int k = 0; k < num_angle; k++)
			{
				sampleRotationVector(sample_longitude[i], sample_latitude[j], sample_angle[k], rotationVector);

				rotationVectorList.push_back(rotationVector);
				potentialdirection.push_back(rotationVector.normalized());
				reset.push_back(true);
				/*if (k == 0)
				{
					reset.push_back(true);
				}
				else
					reset.push_back(false);*/
			}
		}
	}

}

void generatePolarDecompositionDataRandomly(int numData, std::vector<double> &origin_data, std::vector<double> &target_data, std::vector<double> &axis_list)
{
	origin_data.clear();
	target_data.clear();
	axis_list.clear();

	Matrix3d rotationMatrix;
	Matrix3d shearScaleMatrix;
	Matrix3d inputMatrix;

	for (int i = 0; i < numData; i++)
	{
		srand(time(NULL) + i);
		inputMatrix.setRandom();
		inputMatrix.normalize();

		Affine3d af;
		af = inputMatrix;
		rotationMatrix = af.rotation();

		AngleAxisd aa;
		aa = rotationMatrix;
		Vector3d axis = aa.axis();

		shearScaleMatrix = rotationMatrix.transpose()*inputMatrix;


		if (inputMatrix.determinant() < 1e-3)
		{
			continue;
		}

		if (std::abs(aa.angle()) < 1e-1)
		{
			continue;
		}

		for (int a = 0; a < 9; a++)
		{
			origin_data.push_back(inputMatrix.data()[a]);
			target_data.push_back(rotationMatrix.data()[a]);
		}

		for (int a = 0; a < 3; a++)
		{
			axis_list.push_back(axis.data()[a]);
		}
		axis_list.push_back(aa.angle());
		axis_list.push_back(inputMatrix.determinant());

	}
}

void computeRotationBetween2Vec(Vector3d a, Vector3d b, Matrix3d& R, double eplison /*= 1e-12*/)
{
	double norma = a.norm();
	double normb = b.norm();

	if (norma < eplison)
	{
		norma  = eplison;
	}

	if (normb < eplison)
	{
		normb = eplison;
	}

	a /= norma;
	b /= normb;

	Vector3d ax = a.cross(b);
	double normax = ax.norm();
	if (normax < eplison)
	{
		normax = eplison;
	}

	ax /= normax;
//angle = acos(min(dot(an, bn), 1));

	double dotab = a.dot(b);
	if (dotab > 1)
	{
		dotab = 1;
	}

	double angle = std::acos(dotab);

	AngleAxis<double> aa(angle, ax);

	R = aa.toRotationMatrix();

	if (angle < 1e-14)
	{
		R.setIdentity();
	}
}

