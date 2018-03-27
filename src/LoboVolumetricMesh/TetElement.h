#pragma once
#include "LoboElement.h"

class TetElement : public LoboElement
{
public:
	TetElement();
	~TetElement();

	virtual LoboElement::elementType getType() const;

	Matrix4d shape_function;
	Matrix4d shape_function_inv;

	Matrix3d Dm;
	Matrix3d Dm_inverse;

	int face_indices[12];
	double area[4];
	Vector3d internal_forces_[4];
	Vector3d nodes_norm_[4];
	Vector3d faces_norm_[4];

	Vector3d Phi_g[4];
	MatrixXd Phi_derivate;

	Matrix3d A[4][4];
	double B[4][4];
	Vector3d C[4][4][4];
	double D[4][4][4][4];

	double volume_;
	int region_;


	virtual void computeElementDeformationshapeMatrix(Matrix3d &Ds, double* dis);
	
};

TetElement* downCastTetelement(LoboElement* element);

