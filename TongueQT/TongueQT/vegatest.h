#pragma once
#include <Eigen/Dense>
using namespace Eigen;

class VolumetricMesh;
class StVKElementABCD;
class StVKInternalForces;
class StVKStiffnessMatrix;
class TetMesh;
class IsotropicMaterial;
class IsotropicHyperelasticFEM;
class IsotropicHyperelasticFEMForceModel;

class Vegatest
{
public:
	Vegatest(const char* volumtricmeshfile,double young,double possion,double density);
	~Vegatest();

	void test(double * displacement = NULL);

	VolumetricMesh* volumetricMesh;
	StVKElementABCD * precomputedIntegrals;
	StVKInternalForces* stVKInternalForces;
	StVKStiffnessMatrix* stVKStiffnessMatrix;
	TetMesh * tetMesh;
	IsotropicMaterial * isotropicMaterial;
	IsotropicHyperelasticFEM * isotropicHyperelasticFEM;
	IsotropicHyperelasticFEMForceModel * isotropicHyperelasticFEMForceModel;

};

