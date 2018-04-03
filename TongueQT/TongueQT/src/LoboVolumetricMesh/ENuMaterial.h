#pragma once
#include "LoboVolumetricMesh/LoboVolumetricMesh.h"
class LoboVolumetricMesh::ENuMaterial :public LoboVolumetricMesh::Material
{
public:
	ENuMaterial(const std::string name, double density, double E, double nu);
	ENuMaterial(const ENuMaterial & eNuMaterial);
	virtual LoboVolumetricMesh::Material * clone() const;
	~ENuMaterial();

	virtual LoboVolumetricMesh::Material::materialType getType();

	inline double getE() const; // Young's modulus
	inline double getNu() const; // Poisson's ratio
	inline double getLambda() const; // Lame's lambda coefficient
	inline double getMu() const; // Lame's mu coefficient
	inline void setE(double E);
	inline void setNu(double nu);

protected:
	double E_, nu_;
};

inline LoboVolumetricMesh::ENuMaterial::ENuMaterial(std::string name, double density, double E, double nu) : LoboVolumetricMesh::Material(name, density), E_(E), nu_(nu) {}
inline LoboVolumetricMesh::ENuMaterial::ENuMaterial(const ENuMaterial & eNuMaterial) : LoboVolumetricMesh::Material(eNuMaterial.getName(), eNuMaterial.getDensity()), E_(eNuMaterial.getE()), nu_(eNuMaterial.getNu()) {}
inline double LoboVolumetricMesh::ENuMaterial::getE() const { return E_; }
inline double LoboVolumetricMesh::ENuMaterial::getNu() const { return nu_; }
inline double LoboVolumetricMesh::ENuMaterial::getLambda() const { return (nu_ * E_) / ((1 + nu_) * (1 - 2 * nu_)); }
inline double LoboVolumetricMesh::ENuMaterial::getMu() const { return E_ / (2 * (1 + nu_)); }
inline void LoboVolumetricMesh::ENuMaterial::setE(double E) { E_ = E; }
inline void LoboVolumetricMesh::ENuMaterial::setNu(double nu) { nu_ = nu; }

// obtain pointer to ENuMaterial (necessary inside classes that assume ENu material)
LoboVolumetricMesh::ENuMaterial * downcastENuMaterial(LoboVolumetricMesh::Material * material); // performs a check via getType and returns NULL if material is not ENU