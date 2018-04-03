#include "LoboFunctions.h"
#include "eig3.h"

void lobo::centerTinyShape(std::vector<shape_t> &shapes, Vector3d& center)
{
	//center.setZero();
	//int numberofposition = 0;
	//for (int i = 0; i < shapes.size(); i++)
	//{
	//	numberofposition += shapes[i].mesh.positions.size() / 3;
	//	for (int j = 0; j < shapes[i].mesh.positions.size()/3; j++){
	//		center.data()[0] += shapes[i].mesh.positions[j * 3 + 0];
	//		center.data()[1] += shapes[i].mesh.positions[j * 3 + 1];
	//		center.data()[2] += shapes[i].mesh.positions[j * 3 + 2];
	//	}
	//}
	//center /= (double)numberofposition;

	//for (int i = 0; i < shapes.size(); i++)
	//{
	//	for (int j = 0; j < shapes[i].mesh.positions.size() / 3; j++){
	//		shapes[i].mesh.positions[j * 3 + 0] -= center.data()[0];
	//		shapes[i].mesh.positions[j * 3 + 1] -= center.data()[1];
	//		shapes[i].mesh.positions[j * 3 + 2] -= center.data()[2];
	//	}
	//}
	//center *= -1;
}

void lobo::translateTinyShape(std::vector<shape_t> &shapes, Vector3d &translate)
{
	//for (int i = 0; i < shapes.size(); i++)
	//{
	//	for (int j = 0; j < shapes[i].mesh.positions.size() / 3; j++){
	//		shapes[i].mesh.positions[j * 3 + 0] += translate.data()[0];
	//		shapes[i].mesh.positions[j * 3 + 1] += translate.data()[1];
	//		shapes[i].mesh.positions[j * 3 + 2] += translate.data()[2];
	//	}
	//}
}

bool lobo::computeFunnelRadius(double y, double &x, double funnel_radius)
{
	if (y > 1.2)
	{
		return false;
	}
	if (y < -1.2)
	{
		return false;
	}
	
	if (y >= 0)
	{
		x = 0.5*pow(y, 4) + funnel_radius;
	}
	else if (y>-1.2)
	{
		x = funnel_radius;
	}
	else
	{
		x = 0.5*pow(std::abs(y + 1.2) + 0.4, 4) + funnel_radius;
	}

	return true;
}

bool lobo::isInRect(Rect &rect, Vector3d &point)
{
	if (point.x() < rect.x2&&point.x() > rect.x1&&point.y() > rect.y1&&point.y() < rect.y2&&point.z() > rect.z1 && point.z() < rect.z2)
	{
		return true;
	}
	return false;
}

void lobo::neighborOffsetMap(Vector3d &position, Vector3d &neighbor, int &index)
{
	int x, y, z;
	Vector3d offset = neighbor - position;
	
	x = mapDoubleTo101(offset.x());
	y = mapDoubleTo101(offset.y());
	z = mapDoubleTo101(offset.z());
	index = mapOffset[x * 9 + y * 3 + z];
	assert(index != -1);
}

bool lobo::isFiniteNumber(double x)
{
	return !(x <= DBL_MAX && x >= -DBL_MAX);
}

int lobo::mapDoubleTo101(double x)
{
	int x_;
	if (x>0)
	{
		x_ = 2;
	}
	if (x < 0)
	{
		x_ = 0;
	}
	if (std::abs(x) < 1e-10)
	{
		x_ = 1;
	}
	return x_;
}










void lobo::computeAng(Vector3d &center, Vector3d &z, Vector3d &y, double &result, bool negy)
{
	Vector3d z_c = z - center;
	Vector3d y_c = y - center;
	result = std::acos(z_c.dot(y_c) / (z_c.norm()*y_c.norm()));
}

void lobo::subSparseMatrix(SparseMatrix<double> &source, SparseMatrix<double> &result, std::vector<int> &map)
{
	std::vector<EIGEN_TRI> reusltcoef;
	if (map.size() == source.rows())
	{
		for (int j = 0; j < source.outerSize(); ++j)
			for (SparseMatrix<double>::InnerIterator it(source, j); it; ++it)
			{
				int i_p = map[it.row()];
				int j_p = map[it.col()];
				if (!(i_p == -1 || j_p == -1))
				{
					reusltcoef.push_back(EIGEN_TRI(i_p, j_p, it.value()));
				}
			}
		result.setFromTriplets(reusltcoef.begin(), reusltcoef.end());
	}
	else
	if (map.size() == result.rows())
	{
		std::cout << "sparse matrix does not support this map" << std::endl;
		return;
	}
}

void lobo::subMatrix(MatrixXd& source, MatrixXd&result, std::vector<int> &map)
{
	// map source:result
	if (map.size() == source.rows())
	{
		for (int i = 0; i < source.rows(); i++)
		{
			for (int j = 0; j < source.cols(); j++)
			{
				int i_p = map[i];
				int j_p = map[j];
				if (!(i_p == -1 || j_p == -1))
				{
					result.data()[j_p*result.rows() + i_p] = source.data()[j*source.rows() + i];
				}
			}
		}

	}
	else // map result:source
	if (map.size() == result.rows())
	{
		for (int i = 0; i < result.rows(); i++)
		{
			for (int j = 0; j < result.cols(); j++)
			{
				int i_p = map[i];
				int j_p = map[j];
				result.data()[j*result.rows() + i] = source.data()[j_p*source.rows() + i_p];
			}
		}
	}
}

void lobo::subVector(VectorXd& source, VectorXd&result, std::vector<int> &map)
{
	if (map.size() == source.size())
	{
		for (int i = 0; i < source.size(); i++)
		{
				int i_p = map[i];
				if (!(i_p == -1))
				{
					result.data()[i_p] = source.data()[i];
				}
		}

	}
	else // map result:source
	if (map.size() == result.size())
	{
		for (int i = 0; i < result.size(); i++)
		{
				int i_p = map[i];
				result.data()[i] = source.data()[i_p];
		}
	}
}

void lobo::matrixBlockOperate(MatrixXd &source, MatrixXd &block, int i, int j, int row, int col)
{
	int sourcer, sourcec;
	for (int r = 0; r < row; r++)
	{
		for (int c = 0; c < col; c++)
		{
			sourcer = i + r;
			sourcec = j + c;
			source.data()[sourcec*source.rows() + sourcec] += block.data()[c*block.rows() + r];
		}
	}
}

void lobo::computeIntersectionForce(int tri1, Tri_Mesh* mode1, int tri2, Tri_Mesh* mode2, bool selfcollision, int inverse_hack_force, bool if_hack)
{
	int tri1_ndoe0 = mode1->indices.data()[tri1 * 3 + 0];
	int tri1_ndoe1 = mode1->indices.data()[tri1 * 3 + 1];
	int tri1_ndoe2 = mode1->indices.data()[tri1 * 3 + 2];

	int tri2_ndoe0 = mode2->indices.data()[tri2 * 3 + 0];
	int tri2_ndoe1 = mode2->indices.data()[tri2 * 3 + 1];
	int tri2_ndoe2 = mode2->indices.data()[tri2 * 3 + 2];

	Vector3d v_tri1_node0(mode1->positions->data()[tri1_ndoe0 * 3 + 0] + mode1->displacement.data()[tri1_ndoe0 * 3 + 0],
		mode1->positions->data()[tri1_ndoe0 * 3 + 1] + mode1->displacement.data()[tri1_ndoe0 * 3 + 1],
		mode1->positions->data()[tri1_ndoe0 * 3 + 2] + mode1->displacement.data()[tri1_ndoe0 * 3 + 2]);
	Vector3d v_tri1_node1(mode1->positions->data()[tri1_ndoe1 * 3 + 0] + mode1->displacement.data()[tri1_ndoe1 * 3 + 0],
		mode1->positions->data()[tri1_ndoe1 * 3 + 1] + mode1->displacement.data()[tri1_ndoe1 * 3 + 1],
		mode1->positions->data()[tri1_ndoe1 * 3 + 2] + mode1->displacement.data()[tri1_ndoe1 * 3 + 2]);
	Vector3d v_tri1_node2(mode1->positions->data()[tri1_ndoe2 * 3 + 0] + mode1->displacement.data()[tri1_ndoe2 * 3 + 0],
		mode1->positions->data()[tri1_ndoe2 * 3 + 1] + mode1->displacement.data()[tri1_ndoe2 * 3 + 1],
		mode1->positions->data()[tri1_ndoe2 * 3 + 2] + mode1->displacement.data()[tri1_ndoe2 * 3 + 2]);


	Vector3d v_tri2_node0(mode2->positions->data()[tri2_ndoe0 * 3 + 0] + mode2->displacement.data()[tri2_ndoe0 * 3 + 0],
		mode2->positions->data()[tri2_ndoe0 * 3 + 1] + mode2->displacement.data()[tri2_ndoe0 * 3 + 1],
		mode2->positions->data()[tri2_ndoe0 * 3 + 2] + mode2->displacement.data()[tri2_ndoe0 * 3 + 2]);
	Vector3d v_tri2_node1(mode2->positions->data()[tri2_ndoe1 * 3 + 0] + mode2->displacement.data()[tri2_ndoe1 * 3 + 0],
		mode2->positions->data()[tri2_ndoe1 * 3 + 1] + mode2->displacement.data()[tri2_ndoe1 * 3 + 1],
		mode2->positions->data()[tri2_ndoe1 * 3 + 2] + mode2->displacement.data()[tri2_ndoe1 * 3 + 2]);
	Vector3d v_tri2_node2(mode2->positions->data()[tri2_ndoe2 * 3 + 0] + mode2->displacement.data()[tri2_ndoe2 * 3 + 0],
		mode2->positions->data()[tri2_ndoe2 * 3 + 1] + mode2->displacement.data()[tri2_ndoe2 * 3 + 1],
		mode2->positions->data()[tri2_ndoe2 * 3 + 2] + mode2->displacement.data()[tri2_ndoe2 * 3 + 2]);

	Vector3d normal1,normal2;
	lobo::computeTriangleNorm(v_tri1_node0, v_tri1_node1, v_tri1_node2, normal1);
	lobo::computeTriangleNorm(v_tri2_node0, v_tri2_node1, v_tri2_node2, normal2);

	addForceToModel(v_tri2_node0, v_tri2_node1, v_tri2_node2, normal2, v_tri1_node0, tri1_ndoe0, mode1, mode2, selfcollision, inverse_hack_force, if_hack);
	addForceToModel(v_tri2_node0, v_tri2_node1, v_tri2_node2, normal2, v_tri1_node1, tri1_ndoe1, mode1, mode2, selfcollision, inverse_hack_force, if_hack);
	addForceToModel(v_tri2_node0, v_tri2_node1, v_tri2_node2, normal2, v_tri1_node2, tri1_ndoe2, mode1, mode2, selfcollision, inverse_hack_force, if_hack);

	addForceToModel(v_tri1_node0, v_tri1_node1, v_tri1_node2, normal1, v_tri2_node0, tri2_ndoe0, mode2, mode1, selfcollision, inverse_hack_force, if_hack);
	addForceToModel(v_tri1_node0, v_tri1_node1, v_tri1_node2, normal1, v_tri2_node1, tri2_ndoe1, mode2, mode1, selfcollision, inverse_hack_force, if_hack);
	addForceToModel(v_tri1_node0, v_tri1_node1, v_tri1_node2, normal1, v_tri2_node2, tri2_ndoe2, mode2, mode1, selfcollision, inverse_hack_force, if_hack);
}

void lobo::addForceToModel(Vector3d &v1, Vector3d &v2, Vector3d &v3, Vector3d &normal, Vector3d &p, int pindex, Tri_Mesh* mode1, Tri_Mesh* mode2, bool selfcollision, int inverse_hack_force,bool if_hack)
{
	double distance =
	lobo::computeDistancePointToTriangle(v1, v2, v3, normal, p);
	if (distance < 0)
	{
		Vector3d vel; 
		if (mode1->velocity.size()>0)
		{
			vel.data()[0] = mode1->velocity.data()[pindex * 3 + 0];
			vel.data()[1] = mode1->velocity.data()[pindex * 3 + 1];
			vel.data()[2] = mode1->velocity.data()[pindex * 3 + 2];
		}
		else
		{
			vel.setZero();
		}

		//hack code add damp to collision force
		if (if_hack)
		{
			if (inverse_hack_force == 1)
			{
				vel -= Vector3d(0, 0, 0.0008) * 1 / 300.0;
			}
			else if (inverse_hack_force == 0)
				vel -= Vector3d(0, 0, -0.0008) * 1 / 300.0;
			else if (inverse_hack_force == 2)
			{

			}
		}

		TriMeshExForce TMEF;
		TMEF.trinode_indice = pindex;
		Vector3d friction = vel - vel.dot(normal)*normal;
		TMEF.force.setZero();

		TMEF.force = -distance*normal*mode1->impactratio - mode1->dampimpact*vel.dot(normal)*normal
			- mode1->frictionratio*std::abs(distance)*friction;
		if (selfcollision)
		{
			TMEF.force += -distance*normal*mode1->selfimpactratio;
		}
		//for impact method handel in simulator
		TMEF.distance = distance*normal;
		TMEF.force_position = p;
		TMEF.relativeVelocity = vel;
		TMEF.normdirection = normal;
		mode1->tri_mesh_force.push_back(TMEF);
	}
}

void lobo::computeTriangleNorm(Vector3d &v1, Vector3d &v2, Vector3d &v3, Vector3d &normal)
{
	normal = (v2 - v1).cross(v3 - v1);
	normal.normalize();
}

double lobo::computeTriangleArea(Vector3d n0, Vector3d n1, Vector3d n2)
{
	n1 = n1 - n0;
	n2 = n2 - n0;

	n0 = n1.cross(n2);

	double area = 0.5 * n0.norm();
	return area;
}

double lobo::computeDistancePointToTriangle(Vector3d &v1, Vector3d &v2, Vector3d &v3, Vector3d &normal, Vector3d &p)
{
	Vector3d pv1 = (p - v1);
	return (p-v1).norm()*pv1.dot(normal)/(pv1.norm());
}

void lobo::eigen_sym(Matrix3d & a, Vector3d & eig_val, Vector3d(&eig_vec)[3])
{
	double A[3][3] = { { a.data()[0], a.data()[3], a.data()[6] },
	{ a.data()[1], a.data()[4], a.data()[7] },
	{ a.data()[2], a.data()[5], a.data()[8] } };

	double V[3][3];
	double d[3];
	eigen_decomposition(A, V, d);

	eig_val = Vector3d(d[2], d[1], d[0]);

	eig_vec[0] = Vector3d(V[0][2], V[1][2], V[2][2]);
	eig_vec[1] = Vector3d(V[0][1], V[1][1], V[2][1]);
	eig_vec[2] = Vector3d(V[0][0], V[1][0], V[2][0]);
}

void lobo::massGrammSchmidt(VectorXd &mode, MatrixXd &target)
{


}

void lobo::findOrthonormalVector(Vector3d &v1, Vector3d &v2)
{
	int smallestIndex = 0;
	for (int dim = 1; dim < 3; dim++)
	{
		if (fabs(v1[dim]) < fabs(v1[smallestIndex]))
		{
			smallestIndex = dim;
		}
	}

	Vector3d axis(0.0, 0.0, 0.0);
	axis[smallestIndex] = 1.0;
	v2 = v1.cross(axis).normalized();
}

void lobo::computeElementStrain(Matrix3d &F, Matrix3d &E)
{
	E = 0.5*(F.transpose()*F - Matrix3d().Identity());
}

void lobo::computeElementPiolaStress(Matrix3d &E, Matrix3d &F, Matrix3d &P, double &mu_, double &lambda_)
{
	P = F*(2.0 * mu_*E + lambda_*E.trace()*Matrix3d().Identity());
}

void lobo::mapFromScreenToWorld(Matrix4d MVP, int width, int height, double depth, Vector3d &veoct, Vector2d &screenpos, bool ortho)
{
	Vector4d vector;
	vector.data()[0] = screenpos.data()[0] * 2.0 / width - 1;
	vector.data()[1] = (height - screenpos.data()[1])*2.0 / height - 1;
	vector.data()[2] = depth;
	vector.data()[3] = 1;

	vector = MVP.inverse()*vector;
	veoct.data()[0] = vector.data()[0]/vector.data()[3];
	veoct.data()[1] = vector.data()[1] / vector.data()[3];
	veoct.data()[2] = vector.data()[2] / vector.data()[3];
}

void lobo::mapToScreenCoordinate(Matrix4d MVP, int width, int height,double &depth, Vector3d &veoct, Vector2d &screenpos,bool ortho)
{
	Vector4d vector;
	vector.data()[0] = veoct.data()[0];
	vector.data()[1] = veoct.data()[1];
	vector.data()[2] = veoct.data()[2];
	vector.data()[3] = 1;

	vector = MVP*vector;

	vector.data()[0] /= vector.data()[3];
	vector.data()[1] /= vector.data()[3];
	
	depth = vector.data()[2]/vector.data()[3];

	screenpos.data()[0] = (vector.data()[0] + 1)*width / 2.0;
	screenpos.data()[1] = height - (vector.data()[1] + 1)*height / 2.0;
}

void lobo::skewMatrix(Vector3d &w, Matrix3d &result)
{
	result.setZero();
	result.data()[1] = w.data()[2];
	result.data()[2] = -w.data()[1];
	result.data()[5] = w.data()[0];
	Matrix3d temp = result.transpose();
	result -= temp;
}

void lobo::skewVector(Vector3d &result, Matrix3d &w)
{
	Matrix3d A = (w-w.transpose())/2;
	result.setZero();
	result.data()[0] = A.data()[5];
	result.data()[1] = -A.data()[2];
	result.data()[2] = A.data()[1];
}

double* lobo::getRowmajorFromColMajor3X3(Matrix3d &input)
{
	double *M = new double[9];
	M[0] = input.data()[0];
	M[1] = input.data()[3];
	M[2] = input.data()[6];
	M[3] = input.data()[1];
	M[4] = input.data()[4];
	M[5] = input.data()[7];
	M[6] = input.data()[2];
	M[7] = input.data()[5];
	M[8] = input.data()[8];
	
	return M;
}

void lobo::rowmajorTocolmajor3X3(double *rowmajor, Matrix3d &result)
{
	result.data()[0] = rowmajor[0];
	result.data()[1] = rowmajor[0 + 3];
	result.data()[2] = rowmajor[0 + 6];
	result.data()[3] = rowmajor[1];
	result.data()[4] = rowmajor[1 + 3];
	result.data()[5] = rowmajor[1 + 6];
	result.data()[6] = rowmajor[2 + 0];
	result.data()[7] = rowmajor[2 + 3];
	result.data()[8] = rowmajor[2 + 6];
}

void lobo::computeSVD(Matrix3d &F, Matrix3d &U, Matrix3d &V, Matrix3d &singularF, double singularValue_eps, int modifiedSVD)
{
	singularF.setZero();
	Matrix3d normalEq = F.transpose()*F;
	Vector3d eigenValues;
	Vector3d eigenVectors[3];
	lobo::eigen_sym(normalEq, eigenValues, eigenVectors);
	V << eigenVectors[0][0], eigenVectors[1][0], eigenVectors[2][0],
		eigenVectors[0][1], eigenVectors[1][1], eigenVectors[2][1],
		eigenVectors[0][2], eigenVectors[1][2], eigenVectors[2][2];

	if (V.determinant() < 0.0)
	{
		V.data()[0] *= -1.0;
		V.data()[1] *= -1.0;
		V.data()[2] *= -1.0;
	}

	singularF.data()[0 * 3 + 0] = (eigenValues[0] > 0.0) ? sqrt(eigenValues[0]) : 0.0;
	singularF.data()[1 * 3 + 1] = (eigenValues[1] > 0.0) ? sqrt(eigenValues[1]) : 0.0;
	singularF.data()[2 * 3 + 2] = (eigenValues[2] > 0.0) ? sqrt(eigenValues[2]) : 0.0;

	Vector3d singualrFInvers;
	singualrFInvers[0] = (singularF.data()[0 * 3 + 0]>singularValue_eps_) ? (1.0 / singularF.data()[0 * 3 + 0]) : 0.0;
	singualrFInvers[1] = (singularF.data()[1 * 3 + 1]>singularValue_eps_) ? (1.0 / singularF.data()[1 * 3 + 1]) : 0.0;
	singualrFInvers[2] = (singularF.data()[2 * 3 + 2]>singularValue_eps_) ? (1.0 / singularF.data()[2 * 3 + 2]) : 0.0;
	U = F*V*singualrFInvers.asDiagonal();
	if ((singularF.data()[0 * 3 + 0] < singularValue_eps_)
		&& (singularF.data()[1 * 3 + 1] < singularValue_eps_)
		&& (singularF.data()[2 * 3 + 2] < singularValue_eps_))
	{
		U.setIdentity();
	}
	else
	{
		int done = 0;
		for (int dim = 0; dim < 3; dim++)
		{
			int dimA = dim;
			int dimB = (dim + 1) % 3;
			int dimC = (dim + 2) % 3;
			if ((singularF.data()[dimB * 3 + dimB] < singularValue_eps_)
				&& (singularF.data()[dimC * 3 + dimC] < singularValue_eps_))
			{
				Vector3d temVec1(U.data()[dimA * 3 + 0], U.data()[dimA * 3 + 1], U.data()[dimA * 3 + 2]);
				Vector3d temVec2;
				lobo::findOrthonormalVector(temVec1, temVec2);
				Vector3d temVec3 = temVec1.cross(temVec2).normalized();
				U.data()[dimB * 3 + 0] = temVec2[0];
				U.data()[dimB * 3 + 1] = temVec2[1];
				U.data()[dimB * 3 + 2] = temVec2[2];
				U.data()[dimC * 3 + 0] = temVec3[0];
				U.data()[dimC * 3 + 1] = temVec3[1];
				U.data()[dimC * 3 + 2] = temVec3[2];
				if (U.determinant() < 0.0)
				{
					U.data()[dimB * 3 + 0] *= -1.0;
					U.data()[dimB * 3 + 1] *= -1.0;
					U.data()[dimB * 3 + 2] *= -1.0;
				}
				done = 1;
				break;
			}
		}

		if (!done)
		{
			for (int dim = 0; dim<3; dim++)
			{
				int dimA = dim;
				int dimB = (dim + 1) % 3;
				int dimC = (dim + 2) % 3;

				if (singularF.data()[dimA * 3 + dimA] < singularValue_eps_)
				{
					// columns dimB and dimC are both good, but column dimA corresponds to a tiny singular value
					Vector3d tmpVec1(U.data()[dimB * 3 + 0], U.data()[dimB * 3 + 1], U.data()[dimB * 3 + 2]); // column dimB
					Vector3d tmpVec2(U.data()[dimC * 3 + 0], U.data()[dimC * 3 + 1], U.data()[dimC * 3 + 2]); // column dimC
					Vector3d tmpVec3 = tmpVec1.cross(tmpVec2).normalized();
					U.data()[dimA * 3 + 0] = tmpVec3[0];
					U.data()[dimA * 3 + 1] = tmpVec3[1];
					U.data()[dimA * 3 + 2] = tmpVec3[2];
					if (U.determinant() < 0.0)
					{
						U.data()[dimA * 3 + 0] *= -1.0;
						U.data()[dimA * 3 + 1] *= -1.0;
						U.data()[dimA * 3 + 2] *= -1.0;
					}
					done = 1;
					break; // out of for
				}
			}
		}

		//Original code need to set a flag modeifiedSVD, our simulator is only for
		//invertible, so I removed the flag. 
		if ((!done)&&(modifiedSVD == 1))
		{
			if (U.determinant() < 0.0)
			{
				int smallestSingularValueIndex = 0;
				for (int dim = 1; dim < 3; dim++)
				{
					if (singularF.data()[dim * 3 + dim]
						< singularF.data()[smallestSingularValueIndex * 3 + smallestSingularValueIndex])
					{
						smallestSingularValueIndex = dim;
					}
				}

				singularF.data()[smallestSingularValueIndex * 3 + smallestSingularValueIndex] *= -1.0;
				U.data()[smallestSingularValueIndex * 3 + 0] *= -1.0;
				U.data()[smallestSingularValueIndex * 3 + 1] *= -1.0;
				U.data()[smallestSingularValueIndex * 3 + 2] *= -1.0;
			}
			//end of if
		}
		//end of else
	}
}