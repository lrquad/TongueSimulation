#pragma once
#define PI_2 1.57079632679
#include <Eigen/Dense>
#include <vector>
using namespace Eigen;

Matrix3d EulerAngelRoataion(double x,double y,double z);

Matrix3d xconventionRotation(double x);
Matrix3d yconventionRotation(double y);
Matrix3d zconventionRotation(double z);


void xconventionTransform(Matrix4d &output, double x);
void xconventionRotation_dx(Matrix4d &output, double x);
void xconventionRotation_dxdx(Matrix4d &output, double x);
void xconventionRotation_dxdxdx(Matrix4d &output, double x);

void yconventionTransform(Matrix4d &output, double y);
void yconventionRotation_dy(Matrix4d & output, double y);
void yconventionRotation_dydy(Matrix4d &output, double y);
void yconventionRotation_dydydy(Matrix4d &output, double y);

void zconventionTransform(Matrix4d &output, double z);
void zconventionRotation_dz(Matrix4d & output, double z);
void zconventionRotation_dzdz(Matrix4d &output, double z);
void zconventionRotation_dzdzdz(Matrix4d &output, double z);



Matrix3d xyzconventionRotation(double x, double y, double z);
std::vector<Matrix3d> xyzRotationDeriv(double x, double y, double z);


//************************************
// Method:    rotationFirstDervie_dx
// FullName:  rotationFirstDervie_dx
// Access:    public 
// Returns:   void
// Qualifier:
// Parameter: Matrix4d & output need to inital outside this function will only set the rotation part
// Parameter: double x
// Parameter: double y
// Parameter: double z
//************************************
void rotationFirstDerive_dx(Matrix4d& output,double x,double y,double z);
void rotationFirstDerive_dy(Matrix4d& output, double x, double y, double z);
void rotationFirstDerive_dz(Matrix4d& output, double x,double y,double z);


void rotationSecondDerive_dxdx(Matrix4d& output, double x, double y, double z);
void rotationSecondDerive_dxdy(Matrix4d& output, double x, double y, double z);
void rotationSecondDerive_dxdz(Matrix4d& output, double x, double y, double z);

void rotationSecondDerive_dydx(Matrix4d& output, double x, double y, double z);
void rotationSecondDerive_dydy(Matrix4d& output, double x, double y, double z);
void rotationSecondDerive_dydz(Matrix4d& output, double x, double y, double z);

void rotationSecondDerive_dzdx(Matrix4d& output, double x, double y, double z);
void rotationSecondDerive_dzdy(Matrix4d& output, double x, double y, double z);
void rotationSecondDerive_dzdz(Matrix4d& output, double x, double y, double z);


//thirdDerive



Matrix3d xRotationDeriv(double x);