#pragma once
//use MKL 
#include "Functions/LoboDefine.h"
//#define EIGEN_USE_MKL_ALL
#include <iostream>
#include <vector>
#include <queue>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/PardisoSupport>
#include "ObjLoader/tiny_obj_loader.h"
#include <time.h>
#include <btBulletDynamicsCommon.h>
#include <coldet.h>
#define OZCOLLIDE_PCH
#include <ozcollide\ozcollide.h>


#define doublemax DBL_MAX
#define doublemin -DBL_MAX

#define sqrt3 1.73205080757
#define sqrt2 1.41421356237
#define singularValue_eps_ 1e-8
#define inversionThreshold 0.5
#define cubaturetrainingsize 400
#define cubatureTOL 0.01
#define outputprecision 64

#define neighbormap(i) i>12 ? i + 1 : i
#define TENSOR3_INDEX(i,j,k) (9*(i)+3*(j)+(k))
#define MIN(x,y) x<y?x:y
#define MAX(x,y) x>y?x:y
#define COLORTABLE_SIZE ((sizeof(lobo::colortable) / sizeof(float)) / 3);

#define NZ_INDEX 12
#define PZ_INDEX 13

#define NX_INDEX 4
#define PX_INDEX 21

#define NY_INDEX 10
#define PY_INDEX 15

typedef Eigen::Triplet<double> EIGEN_TRI;

using namespace tinyobj;
using namespace Eigen;


class QuadraticMWElement;
class QuadraticElement;
class QuadraticFrame;
class FrameBase;

namespace lobo{

	/// <summary>
	/// Data struct to keep one external force element.
	/// </summary>
	struct TriMeshExForce{

		/// <summary>
		/// The force apply to this node in triangle mesh.
		/// </summary>
		int trinode_indice;
		Vector3d force;
		Vector3d distance;
		Vector3d force_position;
		Vector3d relativeVelocity;
		Vector3d normdirection;
	};

	struct TriMeshExConstraint{
		int trinode_indice;
		Vector3d distance;
	};

	struct Rect{
		double x1, x2;
		double y1, y2;
		double z1, z2;

		void setZero()
		{
			x1 = 0;
			x2 = 0;
			y1 = 0;
			y2 = 0;
			z1 = 0;
			z2 = 0;
		}

		inline Rect operator=(Rect a) {
			x1 = a.x1;
			x2 = a.x2;
			y1 = a.y1;
			y2 = a.y2;
			z1 = a.z1;
			z2 = a.z2;
			return a;
		}
	};

	struct Rect2D{
		double x1, x2;
		double y1, y2;
		void setZero()
		{
			x1 = 0; x2 = 0;
			y1 = 0; y2 = 0;
		}

		bool isInRec(double &x, double &y)
		{
			if ((x < x2&&x > x1)&&(y<y2&&y>y1))
			{
				return true;
			}
			else
			{
				return false;
			}
		}
	};

	struct MouseDescribe{
		Rect2D rect;
		Matrix4d MVP;
		int width;
		int height;
		bool ortho;

		void setRect(double x1,double x2,double y1,double y2)
		{
			rect.x1 = MIN(x1, x2);
			rect.x2 = MAX(x1, x2);
			rect.y1 = MIN(y1, y2);
			rect.y2 = MAX(y1, y2);
		}

	};

	struct Tri_Mesh{

		VectorXd* positions;
		VectorXf positions_current;
		VectorXd displacement;
		VectorXd velocity;
		VectorXi indices;
		Vector3d translate;
		Vector3d modeltranslate;
		Vector3d center;
		int collide_tri;
		std::vector<int> coollide_tri_list;
		std::vector<TriMeshExForce> tri_mesh_force;
		std::vector<TriMeshExConstraint> tri_mesh_Constraint;

		double selfimpactratio;

		double impactratio;
		double frictionratio;
		double dampimpact;

		//collision shape
		
		/// <summary>
		/// Invalid
		/// </summary>
		ozcollide::AABBTreePoly *BaseTree;
		/// <summary>
		/// Invalid
		/// </summary>
		ozcollide::AABBTreePoly::BoxColResult boxresult;

		/// <summary>
		/// To detect detail info of collision between two triangle mesh
		/// </summary>
		CollisionModel3D* coldet_model;
		CollisionModel3D* singletriangle_model;

		btDbvt *bt_dynamic_aabb;
		std::vector<btDbvtNode*> leaf;

		btTriangleIndexVertexArray *bt_tri_vertexarray;
		btTriangleMesh *btMesh;
		/// <summary>
		/// Use bullet collision shape to detect if two triangle mesh collid.
		/// </summary>
		btGImpactMeshShape* bt_collision_shape;
		btCollisionObject* bt_collision_object;

		Tri_Mesh()
		{
			collide_tri = -1;
			BaseTree = NULL;
			bt_tri_vertexarray = NULL;
			bt_collision_shape = NULL;
			btMesh = NULL;
			singletriangle_model = NULL;
			coldet_model = NULL;

			velocity.resize(0);
			impactratio = 1;
			frictionratio = 0.001;
			dampimpact = 0.003;
			selfimpactratio = 0.00001;
		}

		~Tri_Mesh()
		{

		}
	};

	template<typename T>
	void deleteStdvectorPointer(std::vector<T> &pointer_list)
	{
		for (std::vector<T>::iterator it = pointer_list.begin(); it != pointer_list.end(); ++it)
		{
			delete (*it);
		}
		pointer_list.clear();
	}

	
	/// <summary>
	/// Find the x% min elements in pointer_list.
	/// </summary>
	/// <param name="pointer_list">The pointer_list.</param>
	/// <param name="percent">The percent.</param>
	/// <returns></returns>
	template<typename T>
	T minPercent(std::vector<T> pointer_list,double percent)
	{		
		if (percent > 1) { percent = 1; std::cout << "percent set to 1" << std::endl; }
		std::sort(pointer_list.begin(), pointer_list.end());
		return pointer_list[pointer_list.size()*percent];
	}

	template<typename T>
	bool findElement(std::vector<T> pointer_list, T ele)
	{
		return std::find(pointer_list.begin(), pointer_list.end(), ele) != pointer_list.end();
	}

	enum loadmode{
		dontsave = 0,
		enforcesave = 1
	};

	template<class Matrix>
	void write_binary(const char* filename, const Matrix& matrix){
		std::ofstream out(filename, std::ios::out | std::ios::binary | std::ios::trunc);
		out.precision(outputprecision);
		typename Matrix::Index rows = matrix.rows(), cols = matrix.cols();
		out.write((char*)(&rows), sizeof(typename Matrix::Index));
		out.write((char*)(&cols), sizeof(typename Matrix::Index));
		out.write((char*)matrix.data(), rows*cols*sizeof(typename Matrix::Scalar));
		out.close();
	}

	template<class Matrix>
	void write_binary(std::ofstream & out, const Matrix& matrix){
		typename Matrix::Index rows = matrix.rows(), cols = matrix.cols();
		out.write((char*)(&rows), sizeof(typename Matrix::Index));
		out.write((char*)(&cols), sizeof(typename Matrix::Index));
		out.write((char*)matrix.data(), rows*cols*sizeof(typename Matrix::Scalar));
	}

	template<class Vector>
	void write_binary_vec(std::ofstream & out, const Vector& vector){
		typename Vector::Index size = vector.size();
		out.write((char*)(&size), sizeof(typename Vector::Index));
		out.write((char*)vector.data(), size*sizeof(typename Vector::Scalar));
	}

	template<class Matrix>
	void read_binary(const char* filename, Matrix& matrix){
		std::ifstream in(filename, std::ios::in | std::ios::binary);
		if (!in.good())
		{
			std::cout << "file not open" << std::endl;
			return;
		}
		typename Matrix::Index rows = 0, cols = 0;
		in.read((char*)(&rows), sizeof(typename Matrix::Index));
		in.read((char*)(&cols), sizeof(typename Matrix::Index));
		matrix.resize(rows, cols);
		in.read((char *)matrix.data(), rows*cols*sizeof(typename Matrix::Scalar));
		in.close();
	}

	template<class Matrix>
	void read_binary(std::ifstream &in, Matrix& matrix){
		if (!in.good())
		{
			std::cout << "file not open" << std::endl;
			return;
		}
		typename Matrix::Index rows = 0, cols = 0;
		in.read((char*)(&rows), sizeof(typename Matrix::Index));
		in.read((char*)(&cols), sizeof(typename Matrix::Index));
		matrix.resize(rows, cols);
		in.read((char *)matrix.data(), rows*cols*sizeof(typename Matrix::Scalar));
	}

	bool isInRect(Rect &rect, Vector3d &point);

	//shapes output
	void centerTinyShape(std::vector<shape_t> &shapes,Vector3d& center);
	
	void translateTinyShape(std::vector<shape_t> &shapes,Vector3d &translate);

	//a node have 26 neighbors.
	// the order is 
	// skip x=0 y=0 z=0;
	void neighborOffsetMap(Vector3d &position,Vector3d &neighbor,int &index);
	
	int mapDoubleTo101(double x);

	//simulator
	void computeDisplacementField(std::vector<QuadraticElement> &sampleNodes_list_,std::vector<FrameBase*> &frame_list_,VectorXd &regioninfo);
	void computeBFSDistance(std::vector<QuadraticElement> &sampleNodes_list_, std::vector<FrameBase*> &frame_list_, VectorXd &regioninfo);
	

	void computeWeightGradient(std::vector<QuadraticElement> &sampleNodes_list_,double edgeratio = 1);

	int searchClosedSamplePoint(std::vector<QuadraticElement> &sampleNodes_list_, Vector3d &position);
	int searchClosedSamplePoint(std::vector<QuadraticMWElement> &sampleNodes_list_, Vector3d &position);


	void subSparseMatrix(SparseMatrix<double> &source, SparseMatrix<double> &result,std::vector<int> &map);

	void subMatrix(MatrixXd& source,MatrixXd&result,std::vector<int> &map);

	void subVector(VectorXd& source,VectorXd&result,std::vector<int> &map);

	void eigen_sym(Matrix3d & M, Vector3d & eig_val, Vector3d(&eig_vec)[3]);

	void findOrthonormalVector(Vector3d &v1, Vector3d &v2);

	void computeElementStrain(Matrix3d &F, Matrix3d &E);

	void computeElementPiolaStress(Matrix3d &E, Matrix3d &F, Matrix3d &P, double &mu_, double &lambda_);

	void matrixBlockOperate(MatrixXd &source,MatrixXd &block,int i,int j,int row,int col);

	void computeIntersectionForce(int tri1, Tri_Mesh* mode1, int tri2, Tri_Mesh* mode2,bool selfcollision = false,int inverse_hack_force = 0,bool if_hack = false);
	
	void computeTriangleNorm(Vector3d &v1,Vector3d &v2, Vector3d &v3,Vector3d &normal);

	double computeTriangleArea(Vector3d v1, Vector3d v2, Vector3d v3);

	double computeDistancePointToTriangle(Vector3d &v1, Vector3d &v2, Vector3d &v3, Vector3d &normal,Vector3d &p);

	void massGrammSchmidt(VectorXd &mode, MatrixXd &target);

	void mapFromScreenToWorld(Matrix4d MVP, int width, int height,double depth, Vector3d &veoct, Vector2d &screenpos_input, bool ortho = false);
	void mapToScreenCoordinate(Matrix4d MVP,int width,int height,double &depth, Vector3d &veoct, Vector2d &screenpos,bool ortho = false);

	void computeSVD(Matrix3d &F, Matrix3d &U, Matrix3d &V, Matrix3d &singularF, double singularValue_eps, int modifiedSVD);

	void skewMatrix(Vector3d &w, Matrix3d &result);

	void skewVector(Vector3d &result, Matrix3d &w);
	//v1 v2 v3 triangle normal from mode2, this function will add force to p from mode1
	inline void addForceToModel(Vector3d &v1, Vector3d &v2, Vector3d &v3, Vector3d &normal, Vector3d &p,int pindex, Tri_Mesh* mode1, Tri_Mesh* mode2,bool selfcollision = false,int inverse_hack_force = false,bool if_hack = false);

	void rowmajorTocolmajor3X3(double *rowmajor, Matrix3d &result);

	double* getRowmajorFromColMajor3X3(Matrix3d &input);

	void setRoataionMatrix(Matrix3d &rotation,double angle, double x,double y,double z);

	bool isFiniteNumber(double x);
	
	void computeAng(Vector3d &center, Vector3d &z, Vector3d &y,double &result,bool negy);

	bool computeFunnelRadius(double y, double &x, double funnel_radius=0.2);

	static inline bool isSpace(const char c) { return (c == ' ') || (c == '\t'); }

	static int mapOffset[27] =
	{
		0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, -1, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25
	};

	static double edgelength[27] =
	{
		sqrt3, sqrt2, sqrt3,
		sqrt2, 1, sqrt2,
		sqrt3, sqrt2, sqrt3,
		sqrt2, 1, sqrt2,
		1, 0, 1,
		sqrt2, 1, sqrt2,
		sqrt3, sqrt2, sqrt3,
		sqrt2, 1, sqrt2,
		sqrt3, sqrt2, sqrt3
	};

	static float colortable[27] =
	{
		1, 0.5, 0,
		0.5, 1, 0,
		0, 0.5, 1,
		1, 1, 0.5,
		0.5, 1, 1,
		1, 0.5, 1,
		1, 0.2,0.2,
		0.2,1,0.2,
		0.2,0.2,1
	};

};