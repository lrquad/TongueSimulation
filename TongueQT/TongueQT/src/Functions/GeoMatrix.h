#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <math.h>
#include <vector>
#ifndef M_PI
#    define M_PI 3.14159265358979323846
#endif
using namespace Eigen;

void computeShearMatrix(double x, double y, double z, Matrix3d &matrix);
void computeRotationMatrix(double x, double y, double z, Matrix3d &matrix);

void PCAMatrix(MatrixXd &inputMatrix, MatrixXd &outputMatrix, MatrixXd &outputPCATransform, int pcadimension, RowVectorXd &scalingFactors, RowVectorXd &minCoeffs, RowVectorXd &featureMeans);

/**
 * @brief Put brief description here
 *
 * Put full description here
 * @param[in,out] longitude 360
 * @param[in,out] Latitude +-90
 * @param[in,out] angle Put argument desc here
 * @param[in,out] matrix Put argument desc here
 * @return Put return information here
 */
void sampleRotationMatrix(double longitude, double latituede, double angle, Matrix3d &matrix);
void smapleRoataionVector(double longitude, double latituede, double angle, Vector3d &rotationvector);

//-1 0 0, 0 1 0 , 0 0 1
void sampleRotationVector(double longitude, double latituede, double angle, Vector3d &rotationvector);

 void sampleShearingScaleMatrix(double shearingRatio, Matrix3d& matrix,int i);

 double uniformSampleRotation(int seed,double maxlatitude,double minlatittude,double maxlongitude,double minlongitude,double maxangle,double minangle,Matrix3d &rotationmatrix,Vector3d &rotationVector);



//************************************
// Method:    generatePolarDecompositionData
// FullName:  generatePolarDecompositionData
// Access:    public 
// Returns:   void
// Qualifier:
// Parameter: double maxlongitude
// Parameter: double minlongitude
// Parameter: int num_longitude
// Parameter: double maxLatitude
// Parameter: double minLatitude
// Parameter: int num_latitude
// Parameter: double maxangle
// Parameter: double minangle
// Parameter: int num_angle
// Parameter: double maxshearing
// Parameter: double minshearing
// Parameter: int num_shearing
// Parameter: std::vector<double> & origin_data
// Parameter: std::vector<double> & target_data
// Parameter: std::vector<double> & axis
//************************************
void generatePolarDecompositionData(double maxlongitude, double minlongitude, int num_longitude, double maxLatitude, double minLatitude, int num_latitude, double maxangle, double minangle, int num_angle, double maxshearing, double minshearing, int num_shearing, std::vector<double> &origin_data, std::vector<double> &target_data, std::vector<double> &axis);

void generateRotationVectorSample(double maxlongitude, double minlongitude, int num_longitude, double maxLatitude, double minLatitude, int num_latitude, double maxangle, double minangle, int num_angle, std::vector<Vector3d> &rotationVectorList);

void generateRotationVectorSample(double maxlongitude, double minlongitude, int num_longitude, double maxLatitude, double minLatitude, int num_latitude, double maxangle, double minangle, int num_angle, std::vector<Vector3d> &rotationVectorList, std::vector<bool> &reset);

void generateRotationVectorSample(double maxlongitude, double minlongitude, int num_longitude, double maxLatitude, double minLatitude, int num_latitude, double maxangle, double minangle, int num_angle, std::vector<Vector3d> &rotationVectorList, std::vector<bool> &reset,std::vector<Vector3d> &potentialdirection);

void generatePolarDecompositionDataRandomly(int numData, std::vector<double> &origin_data, std::vector<double> &target_data, std::vector<double> &axis);


void computeRotationBetween2Vec(Vector3d a,Vector3d b,Matrix3d& R,double eplison = 1e-12);

template <class MatT>
Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime>
pseudoinverse(const MatT &mat, typename MatT::Scalar tolerance = typename MatT::Scalar{ 1e-4 }) // choose appropriately
{
	typedef typename MatT::Scalar Scalar;
	auto svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
	const auto &singularValues = svd.singularValues();
	Eigen::Matrix<Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> singularValuesInv(mat.cols(), mat.rows());
	singularValuesInv.setZero();
	for (unsigned int i = 0; i < singularValues.size(); ++i) {
		if (singularValues(i) > tolerance)
		{
			singularValuesInv(i, i) = Scalar{ 1 } / singularValues(i);
		}
		else
		{
			singularValuesInv(i, i) = Scalar{ 0 };
		}
	}
	return svd.matrixV() * singularValuesInv * svd.matrixU().adjoint();
}