#include "EulerAngelRotationMatrix.h"
#include <math.h>

Matrix3d EulerAngelRoataion(double x, double y, double z)
{
	Matrix3d m;
	m = xconventionRotation(x)
		* yconventionRotation(y)
		* zconventionRotation(z);



	return m;
}

Matrix3d xconventionRotation(double x)
{
	//return AngleAxisd(x, Vector3d::UnitX()).toRotationMatrix();

	Matrix3d m;

	double cosx = cos(x);
	double sinx = sin(x);

	m.setZero();
	m.data()[0] = 1;
	m.data()[4] = cosx;
	m.data()[5] = sinx;
	m.data()[7] = -sinx;
	m.data()[8] = cosx;

	return m;

	
}

Matrix3d yconventionRotation(double y)
{
	//return AngleAxisd(y, Vector3d::UnitY()).toRotationMatrix();
	Matrix3d m;

	double cosy = cos(y);
	double siny = sin(y);

	m.setZero();
	m.data()[0] = cosy;
	m.data()[2] = -siny;
	m.data()[4] = 1;
	m.data()[6] = siny;
	m.data()[8] = cosy;
	return m;

}

Matrix3d zconventionRotation(double z)
{
	//return AngleAxisd(z, Vector3d::UnitZ()).toRotationMatrix();
	Matrix3d m;
	m.setZero();

	double cosq = cos(z);
	double sinq = sin(z);

	m.data()[0] = cosq;
	m.data()[1] = sinq;
	m.data()[3] = -sinq;
	m.data()[4] = cosq;
	m.data()[8] = 1;

	return m;
}

void xconventionTransform(Matrix4d &output, double x)
{
	output.setZero();

	double cosx = cos(x);
	double sinx = sin(x);

	output.data()[0] = 1;
	output.data()[5] = cosx;
	output.data()[6] = sinx;
	output.data()[9] = -sinx;
	output.data()[10] = cosx;
	output.data()[15] = 1;
}

void xconventionRotation_dx(Matrix4d &output, double x)
{
	output.setZero();

	double cosx = cos(x);
	double sinx = sin(x);

	output.data()[0] = 0;
	output.data()[5] = -sinx;
	output.data()[6] = cosx;
	output.data()[9] = -cosx;
	output.data()[10] = -sinx;
}

void xconventionRotation_dxdx(Matrix4d &output, double x)
{
	output.setZero();

	double cosx = cos(x);
	double sinx = sin(x);

	output.data()[0] = 0;
	output.data()[5] = -cosx;
	output.data()[6] = -sinx;
	output.data()[9] = sinx;
	output.data()[10] = -cosx;
}

void xconventionRotation_dxdxdx(Matrix4d &output, double x)
{
	output.setZero();

	double cosx = cos(x);
	double sinx = sin(x);

	output.data()[0] = 0;
	output.data()[5] = sinx;
	output.data()[6] = -cosx;
	output.data()[9] = cosx;
	output.data()[10] = sinx;
}

void yconventionTransform(Matrix4d &output, double y)
{
	output.setZero();
	double cosy = cos(y);
	double siny = sin(y);

	output.data()[0] = cosy;
	output.data()[2] = -siny;
	output.data()[5] = 1;
	output.data()[8] = siny;
	output.data()[10] = cosy;
	output.data()[15] = 1;
}

void yconventionRotation_dy(Matrix4d & output, double y)
{
	output.setZero();


	double cosy = cos(y);
	double siny = sin(y);

	output.data()[0] = -siny;
	output.data()[2] = -cosy;
	output.data()[5] = 0;
	output.data()[8] = cosy;
	output.data()[10] = -siny;
}

void yconventionRotation_dydy(Matrix4d &output, double y)
{
	output.setZero();

	double cosy = cos(y);
	double siny = sin(y);

	output.data()[0] = -cosy;
	output.data()[2] = siny;
	output.data()[5] = 0;
	output.data()[8] = -siny;
	output.data()[10] = -cosy;
}


void yconventionRotation_dydydy(Matrix4d &output, double y)
{
	output.setZero();

	double cosy = cos(y);
	double siny = sin(y);

	output.data()[0] = siny;
	output.data()[2] = cosy;
	output.data()[5] = 0;
	output.data()[8] = -cosy;
	output.data()[10] = siny;
}

void zconventionTransform(Matrix4d &output, double z)
{
	output.setZero();

	double cosz = cos(z);
	double sinz = sin(z);

	output.data()[0] = cosz;
	output.data()[1] = sinz;
	output.data()[4] = -sinz;
	output.data()[5] = cosz;
	output.data()[10] = 1;
	output.data()[15] = 1;
}

void zconventionRotation_dz(Matrix4d & output, double z)
{
	output.setZero();
	double cosz = cos(z);
	double sinz = sin(z);

	output.data()[0] = -sinz;
	output.data()[1] = cosz;
	output.data()[4] = -cosz;
	output.data()[5] = -sinz;
	output.data()[10] = 0;
}

void zconventionRotation_dzdz(Matrix4d &output, double z)
{
	output.setZero();
	double cosz = cos(z);
	double sinz = sin(z);

	output.data()[0] = -cosz;
	output.data()[1] = -sinz;
	output.data()[4] = sinz;
	output.data()[5] = -cosz;
	output.data()[10] = 0;
}

void zconventionRotation_dzdzdz(Matrix4d &output, double z)
{
	output.setZero();
	double cosz = cos(z);
	double sinz = sin(z);

	output.data()[0] = sinz;
	output.data()[1] = -cosz;
	output.data()[4] = cosz;
	output.data()[5] = sinz;
	output.data()[10] = 0;
}

Matrix3d xyzconventionRotation(double x, double y, double z)
{
	Matrix3d m;

	double sinx = sin(x);
	double cosx = cos(x);

	double siny = sin(y);
	double cosy = cos(y);

	double sinz = sin(z);
	double cosz = cos(z);

	m.data()[0] = cosy*cosz;
	m.data()[1] = cosz*sinx*siny + cosx*sinz;
	m.data()[2] = -cosx*cosz*siny + sinx*sinz;

	m.data()[3] = -cosy*sinz;
	m.data()[4] = cosx*cosz - sinx*siny*sinz;
	m.data()[5] = cosz*sinx + cosx*siny*sinz;

	m.data()[6] = siny;
	m.data()[7] = -cosy*sinx;
	m.data()[8] = cosx*cosy;

	return m;
}

std::vector<Matrix3d> xyzRotationDeriv(double x, double y, double z)
{
	std::vector<Matrix3d> m(3);
	double sinx = sin(x);
	double cosx = cos(x);

	double siny = sin(y);
	double cosy = cos(y);

	double sinz = sin(z);
	double cosz = cos(z);
	

	//repect to x
	m[0].setZero();
	m[0].data()[1] = cosz*cosx*siny - sinx*sinz;
	m[0].data()[2] = sinx*cosz*siny + cosx*sinz;
	m[0].data()[4] = -sinx*cosz - cosx*siny*sinz;
	m[0].data()[5] = cosz*cosx - sinx*siny*sinz;
	m[0].data()[7] = -cosy*cosx;
	m[0].data()[8] = -sinx*cosy;


	//respect y
	m[1].setZero();
	m[1].data()[0] = -siny*cosz;
	m[1].data()[1] = cosz*sinx*cosy;
	m[1].data()[2] = -cosx*cosz*cosy;
	m[1].data()[3] = siny*sinz;
	m[1].data()[4] = -sinx*cosy*sinz;
	m[1].data()[5] = cosx*cosy*sinz;
	m[1].data()[6] = cosy;
	m[1].data()[7] = siny*sinx;
	m[1].data()[8] = -cosx*siny;

	//respect z
	m[2].setZero();
	m[2].data()[0] = -cosy*sinz;
	m[2].data()[1] = -sinz*sinx*siny + cosx*cosz;
	m[2].data()[2] = cosx*sinz*siny + sinx*cosz;
	m[2].data()[3] = -cosy*cosz;
	m[2].data()[4] = -cosx*sinz - sinx*siny*cosz;
	m[2].data()[5] = -sinz*cosx + cosx*siny*cosz;


	return m;
}

void rotationFirstDerive_dx(Matrix4d& output, double x, double y, double z)
{
	output.topLeftCorner<3, 3>().setZero();

	double sinx = sin(x);
	double cosx = cos(x);

	double siny = sin(y);
	double cosy = cos(y);

	double sinz = sin(z);
	double cosz = cos(z);

	//output.data()[0] = 0;
	output.data()[1] = cosz*cosx*siny - sinx*sinz;
	output.data()[2] = sinx*cosz*siny + cosx*sinz;
	//output.data()[4] = 0;
	output.data()[5] = -sinx*cosz - cosx*siny*sinz;
	output.data()[6] = cosz*cosx - sinx*siny*sinz;
	//output.data()[8] = 0;
	output.data()[9] = -cosy*cosx;
	output.data()[10] = -sinx*cosy;
}

void rotationFirstDerive_dy(Matrix4d& output, double x, double y, double z)
{
	output.topLeftCorner<3, 3>().setZero();

	double sinx = sin(x);
	double cosx = cos(x);

	double siny = sin(y);
	double cosy = cos(y);

	double sinz = sin(z);
	double cosz = cos(z);

	output.data()[0] = -siny*cosz;
	output.data()[1] = cosz*sinx*cosy;
	output.data()[2] = -cosx*cosz*cosy;
	output.data()[4] = siny*sinz;
	output.data()[5] = -sinx*cosy*sinz;
	output.data()[6] = cosx*cosy*sinz;
	output.data()[8] = cosy;
	output.data()[9] = siny*sinx;
	output.data()[10] = -cosx*siny;
}

void rotationFirstDerive_dz(Matrix4d& output, double x, double y, double z)
{
	output.topLeftCorner<3, 3>().setZero();

	double sinx = sin(x);
	double cosx = cos(x);

	double siny = sin(y);
	double cosy = cos(y);

	double sinz = sin(z);
	double cosz = cos(z);

	output.data()[0] = -cosy*sinz;
	output.data()[1] = -sinz*sinx*siny + cosx*cosz;
	output.data()[2] = cosx*sinz*siny + sinx*cosz;
	output.data()[4] = -cosy*cosz;
	output.data()[5] = -cosx*sinz - sinx*siny*cosz;
	output.data()[6] = -sinz*sinx + cosx*siny*cosz;
}

void rotationSecondDerive_dxdx(Matrix4d& output, double x, double y, double z)
{
	output.topLeftCorner<3, 3>().setZero();

	double sinx = sin(x);
	double cosx = cos(x);

	double siny = sin(y);
	double cosy = cos(y);

	double sinz = sin(z);
	double cosz = cos(z);

	output.data()[1] = -cosz*sinx*siny - cosx*sinz;
	output.data()[2] = cosx*cosz*siny - sinx*sinz;
	output.data()[5] = -cosx*cosz + sinx*siny*sinz;
	output.data()[6] = -cosz*sinx - cosx*siny*sinz;
	output.data()[9] = cosy*sinx;
	output.data()[10] = -cosx*cosy;
}

void rotationSecondDerive_dxdy(Matrix4d& output, double x, double y, double z)
{
	output.topLeftCorner<3, 3>().setZero();

	double sinx = sin(x);
	double cosx = cos(x);

	double siny = sin(y);
	double cosy = cos(y);

	double sinz = sin(z);
	double cosz = cos(z);

	output.data()[1] = cosz*cosx*cosy;
	output.data()[2] = sinx*cosz*cosy;
	output.data()[5] = - cosx*cosy*sinz;
	output.data()[6] =  - sinx*cosy*sinz;
	output.data()[9] = siny*cosx;
	output.data()[10] = sinx*siny;
}

void rotationSecondDerive_dxdz(Matrix4d& output, double x, double y, double z)
{
	output.topLeftCorner<3, 3>().setZero();

	double sinx = sin(x);
	double cosx = cos(x);

	double siny = sin(y);
	double cosy = cos(y);

	double sinz = sin(z);
	double cosz = cos(z);

	output.data()[1] = -sinz*cosx*siny - sinx*cosz;
	output.data()[2] = -sinx*sinz*siny + cosx*cosz;
	output.data()[5] = sinx*sinz - cosx*siny*cosz;
	output.data()[6] = -sinz*cosx - sinx*siny*cosz;

	/*output.data()[9] = 0;
	output.data()[10] = 0;*/
}

void rotationSecondDerive_dydx(Matrix4d& output, double x, double y, double z)
{
	output.topLeftCorner<3, 3>().setZero();

	double sinx = sin(x);
	double cosx = cos(x);

	double siny = sin(y);
	double cosy = cos(y);

	double sinz = sin(z);
	double cosz = cos(z);

	//output.data()[0] = 0;
	output.data()[1] = cosz*cosx*cosy;
	output.data()[2] = sinx*cosz*cosy;
	//output.data()[4] = 0;
	output.data()[5] = -cosx*cosy*sinz;
	output.data()[6] = -sinx*cosy*sinz;
	//output.data()[8] = 0;
	output.data()[9] = siny*cosx;
	output.data()[10] = sinx*siny;
}

void rotationSecondDerive_dydy(Matrix4d& output, double x, double y, double z)
{
	output.topLeftCorner<3, 3>().setZero();

	double sinx = sin(x);
	double cosx = cos(x);

	double siny = sin(y);
	double cosy = cos(y);

	double sinz = sin(z);
	double cosz = cos(z);

	output.data()[0] = -cosy*cosz;
	output.data()[1] = -cosz*sinx*siny;
	output.data()[2] = cosx*cosz*siny;
	output.data()[4] = cosy*sinz;
	output.data()[5] = sinx*siny*sinz;
	output.data()[6] = -cosx*siny*sinz;
	output.data()[8] = -siny;
	output.data()[9] = cosy*sinx;
	output.data()[10] = -cosx*cosy;
}

void rotationSecondDerive_dydz(Matrix4d& output, double x, double y, double z)
{
	output.topLeftCorner<3, 3>().setZero();

	double sinx = sin(x);
	double cosx = cos(x);

	double siny = sin(y);
	double cosy = cos(y);

	double sinz = sin(z);
	double cosz = cos(z);

	output.data()[0] = siny*sinz;
	output.data()[1] = -sinz*sinx*cosy;
	output.data()[2] = cosx*sinz*cosy;
	output.data()[4] = siny*cosz;
	output.data()[5] = -sinx*cosy*cosz;
	output.data()[6] = cosx*cosy*cosz;
	//output.data()[8] = 0;
	//output.data()[9] = 0;
	//output.data()[10] = 0;
}

void rotationSecondDerive_dzdx(Matrix4d& output, double x, double y, double z)
{
	output.topLeftCorner<3, 3>().setZero();

	double sinx = sin(x);
	double cosx = cos(x);

	double siny = sin(y);
	double cosy = cos(y);

	double sinz = sin(z);
	double cosz = cos(z);

	//output.data()[0] = 0;
	output.data()[1] = -sinz*cosx*siny -sinx*cosz;
	output.data()[2] = -sinx*sinz*siny + cosx*cosz;
	//output.data()[4] = 0;
	output.data()[5] = sinx*sinz - cosx*siny*cosz;
	output.data()[6] = -sinz*cosx - sinx*siny*cosz;

}

void rotationSecondDerive_dzdy(Matrix4d& output, double x, double y, double z)
{
	output.topLeftCorner<3,3>().setZero();

	double sinx = sin(x);
	double cosx = cos(x);

	double siny = sin(y);
	double cosy = cos(y);

	double sinz = sin(z);
	double cosz = cos(z);

	output.data()[0] = siny*sinz;
	output.data()[1] = -sinz*sinx*cosy;
	output.data()[2] = cosx*sinz*cosy;
	output.data()[4] = siny*cosz;
	output.data()[5] = - sinx*cosy*cosz;
	output.data()[6] =  cosx*cosy*cosz;
	
}


void rotationSecondDerive_dzdz(Matrix4d& output, double x, double y, double z)
{
	output.topLeftCorner<3, 3>().setZero();

	double sinx = sin(x);
	double cosx = cos(x);

	double siny = sin(y);
	double cosy = cos(y);

	double sinz = sin(z);
	double cosz = cos(z);

	output.data()[0] = -cosy*cosz;
	output.data()[1] = -cosz*sinx*siny - cosx*sinz;
	output.data()[2] = cosx*cosz*siny - sinx*sinz;
	output.data()[4] = cosy*sinz;
	output.data()[5] = -cosx*cosz + sinx*siny*sinz;
	output.data()[6] = -cosz*sinx - cosx*siny*sinz;

}

Matrix3d xRotationDeriv(double x)
{
	Matrix3d m;
	return m;
}
