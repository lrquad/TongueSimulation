#pragma once
#include <vector>
#include <set>
#include <Eigen/Dense>
#include <Eigen/Sparse>
using namespace Eigen;

class LoboElement;
class LoboNodeBase;
/// <summary>
/// This class based the idea from vega's volumetric mesh. But I add the class of element and node for convenient.
/// in default all elements will have a same material and one region
/// </summary>
class LoboVolumetricMesh
{
	class Set;
	class Material;
	class Region;

public:

	LoboVolumetricMesh();
	LoboVolumetricMesh(const LoboVolumetricMesh* volumetricmesh);
	~LoboVolumetricMesh();

	typedef enum { TET, CUBE } VolumetricmeshType;

	/* =============================
	load mesh
	   =============================*/

	virtual void readNodeList(const char* nodefilename, Vector3d translate, double scale = 1.0);
	virtual void transformMesh(Matrix3d Transform,Vector3d translate);


	virtual void readElementsIndex(const char* elefilename);
	virtual void readElementMesh(const char* filenamebase, double* translate = NULL, double scale = 1.0);

	virtual void saveElementMesh(std::ofstream &outstream);
	virtual void saveNodeList(std::ofstream &outstream);
	virtual void saveElementsIndex(std::ofstream &outstream);

	virtual void readElementMesh(std::ifstream &instream);
	virtual void readNodeList(std::ifstream &instream);
	virtual void readElementsIndex(std::ifstream &instream);

	//read from other mesh
	virtual void readNodeList(LoboVolumetricMesh* input, std::vector<int> nodeindex);
	virtual void readElements(LoboVolumetricMesh* input, std::vector<int> eleindex,std::vector<int> node_map);

	/// <summary>
	/// Init element list from elements;
	/// </summary>
	virtual void readElementList() = 0;

	virtual void assignDefaultMaterial();
	virtual void assignDefualtSet();

	//for experiment
	virtual void creatMaterialForBar(double maxYoung,double minYoung);
	virtual void createMaterialForLollipop(double maxYoung, double minYoung);

	virtual void scaleVolumetricMesh(double scale);

	virtual void computeNodeRotationRing(int nodeid, Matrix3d& rotation);


	/* =============================
	set mesh displacement
	=============================*/
	virtual void setOriPosition(double *q);
	virtual void setDisplacement(double *q);

	virtual void setDisplacementBuffer(double* q);
	double* getDisplacementRef();

	virtual void resetDisplacement();
	virtual void getDisplacement(double *q);

	virtual void exportNodeDisplacementCSV(const char* filename);
	virtual void exportNodeRegionDisplacementCSV(const char* filename);

	//target
	virtual void exportNodeDisplacementStream(std::vector<double> &target);

	//origin
	virtual void exportNodeRegionDisplacementStream(std::vector<double> &origin_);


	virtual VectorXd getNodeRegionDisplacement(int nodeid);

	/* =============================
	control
	=============================*/

	virtual void searchNeighborElement() = 0;

	virtual void computeLaplacianMatrix(SparseMatrix<double>* L);

	virtual void computeRigidModes(MatrixXd &rigidmodes);
	
	virtual int searchCloseNode(const Vector3d &position) const;
	virtual int searchCloseSurfaceNode(const Vector3d &position) const;
	
	virtual int getContainedElement(const Vector3d &position);
	
	virtual bool containsVertex(int element, const Vector3d &pos)  const = 0; // true if given element contain given position, false otherwise

	virtual void computeMassMatrix(std::vector<LoboElement*>* elementlist, SparseMatrix<double>* mass) const= 0;
	virtual void computeMassMatrix(SparseMatrix<double>* mass) = 0;

	virtual void computeBarycentricWeights(int elementid, const Vector3d &pos, Vector4d &weight) const= 0;

	// this will compute surface info save to internal buffer
	virtual void computeSurface() = 0;
	virtual void releaseSurface() = 0;
	
	virtual void saveSurfaceToObjAscii(const char* filename) const = 0;
	virtual void saveSubMeshAsscii(const char* filenamebase,std::vector<int> element_list,std::vector<bool> subnodemark,std::vector<int> submap){};

	virtual void scaleMesh(double scale);

	//************************************
	// Method:    uniformMesh
	// FullName:  LoboVolumetricMesh::uniformMesh
	// Access:    virtual public 
	// Returns:   void
	// Qualifier:
	//************************************
	virtual void uniformMesh(double scaleEx = 1, bool useExScale = false);

	//return the value we need scale the mesh to become uniform
	virtual double computeUniformScale();

	virtual double computeUniformScaleExceptConstraint(int numconstraints,int* constrainedDofs);


	/* =============================
	query
	=============================*/

	bool isSurfaceNode(int nodeid);

	virtual void computeNodeVolume(double *nodeweight) = 0;
	virtual double computeCurrentMeshVolume() = 0;
	virtual void computeDeformationGradient(int elementid, Matrix3d &output) = 0;

	inline const char* getName(){ return mesh_name.c_str();}
	inline void setName(const char* meshname){ mesh_name = meshname;}

	inline bool getSurfaceReady(){ return surfaceready; }
	inline void setSurfaceReady(bool ready){ surfaceready = ready;}

	inline virtual VolumetricmeshType getMeshType() const = 0;

	Vector3d getNodePosition(int i) const; // return node i's current position
	Vector3d getNodeDisplacement(int i) const;
	Vector3d getNodeRestPosition(int i) const; 
	LoboNodeBase* getNodeRef(int i) const;

	inline LoboElement* getElement(int i){ return element_list[i];};
	int getElementNode(int ele, int i){ return elements[ele*numer_element_vertcies + i]; }

	inline int getVertexIndex(int element, int vertex) const {
		return elements[element*numer_element_vertcies + vertex];
	}

	inline int getNumVertices(){ return numVertices;}
	inline int getNumElements(){ return this->numElements;}
	inline int getNumElementVertices(){ return numer_element_vertcies; }

	virtual int getNumElementNeighbors(int ele);
	virtual int getElmentNeighborid(int ele, int neighbori);

	Material* getElementMaterial(int ele) const;
	Material* getNodeMaterial(int node) const;
	Material* getMaterialById(int mid);

	class Set
	{
	public:

		Set(const std::string & name);
		Set(const Set & set);
		Set(const std::string & name, const std::set<int> & elements);

		inline std::string getName() const;
		inline int getNumElements() const;
		inline void getElements(std::set<int> & elements) const;
		inline bool isMember(int element) const;

		inline void insert(int element);
		inline void clear();

	protected:
		std::string name;
		std::set<int> elements;
	};

	class Material
	{
	public:
		Material(const std::string name, double density);
		Material(const Material & material);
		virtual ~Material() {};
		virtual Material * clone() const = 0;

		inline std::string getName() const; // material name
		inline double getDensity() const; // density
		inline void setName(const std::string name);
		inline void setDensity(double density);

		// ENU = any isotropic material parameterized by E (Young's modulus), nu (Poisson's ratio)
		// ORTHOTROPIC = orthotropic anisotropic material
		// MOONEYRIVLIN = Mooney-Rivlin material
		typedef enum { INVALID, ENU} materialType;
		virtual materialType getType() = 0;
	protected:
		std::string name;
		double density;
	};

	class ENuMaterial;

	// a volumetric mesh region, i.e., a set of elements sharing the same material
	class Region
	{
	public:
		Region(int materialIndex, int setIndex);
		inline int getMaterialIndex() const;
		inline int getSetIndex() const;

	protected:
		int setIndex, materialIndex;
	};

	std::vector<int> getSurface_node() const { return surface_node_; }
	std::vector<int> getSurface_ele() const { return surface_ele; }

	friend LoboVolumetricMesh;
	bool getUniformMeshAfterLoad() const { return uniformMeshAfterLoad; }
	void setUniformMeshAfterLoad(bool val) { uniformMeshAfterLoad = val; }
	Eigen::Vector3d getUniformT() const { return uniformT; }
	void setUniformT(Eigen::Vector3d val) { uniformT = val; }

	double getUniformScale() const { return uniformScale; }
	void setUniformScale(double val) { uniformScale = val; }
	bool getUnifromUseScaleEx() const { return unifromUseScaleEx; }
	void setUnifromUseScaleEx(bool val) { unifromUseScaleEx = val; }
	Matrix3d getTransformBaseUniform() const { return transformBaseUniform; }
	void setTransformBaseUniform(Matrix3d val) { transformBaseUniform = val; }
	Vector3d getTranslateBaseUniform() const { return translateBaseUniform; }
	void setTranslateBaseUniform(Vector3d val) { translateBaseUniform = val; }

	std::vector<int> surface_node_;

protected:

	VectorXd volumetricmeshDisplacment;

	std::string mesh_name;

	int numVertices;

	int numer_element_vertcies;
	int numElements; 
	
	std::vector<int> elements;
	std::vector<int> elementsNeighbor;

	int num_Materials;
	int num_Sets;
	int num_Regions;

	std::vector<Set*> sets;
	std::vector<Region*> regions;
	std::vector<Material*> materials;

	std::vector<LoboElement*> element_list;
	std::vector<LoboNodeBase*> node_list;

	std::vector<int> element_material;
	std::vector<int> node_material;
	
	bool surfaceready;
	std::vector<int> surface_ele;
	
	std::vector<int> surface_face_3i_; // 3*facenumber;
	std::vector<bool> issurface_node_;

	bool uniformMeshAfterLoad;
	bool unifromUseScaleEx;
	Vector3d uniformT;
	double uniformScale;

	Matrix3d transformBaseUniform;
	Vector3d translateBaseUniform;

};

inline LoboVolumetricMesh::Set::Set(const std::string & name_) { name = name_; }
inline LoboVolumetricMesh::Set::Set(const Set & set) { elements = set.elements; name = set.getName(); }
inline LoboVolumetricMesh::Set::Set(const std::string & name_, const std::set<int> & elements_) : name(name_), elements(elements_) {}
inline std::string LoboVolumetricMesh::Set::getName() const { return name; }
inline int LoboVolumetricMesh::Set::getNumElements() const { return (int)(this->elements.size()); }
inline void LoboVolumetricMesh::Set::getElements(std::set<int> & elements) const { elements = this->elements; }
inline bool LoboVolumetricMesh::Set::isMember(int element) const { return (elements.find(element) != elements.end()); }
inline void LoboVolumetricMesh::Set::insert(int element) { elements.insert(element); }
inline void LoboVolumetricMesh::Set::clear() { elements.clear(); }

inline LoboVolumetricMesh::Material::Material(const std::string name_, double density_) : density(density_) { name = name_; }
inline LoboVolumetricMesh::Material::Material(const Material & material) : density(material.getDensity()) { name = material.getName(); }
inline std::string LoboVolumetricMesh::Material::getName() const { return name; }  // material name
inline double LoboVolumetricMesh::Material::getDensity() const { return density; } // density
inline void LoboVolumetricMesh::Material::setName(const std::string name_) { name = name_; }
inline void LoboVolumetricMesh::Material::setDensity(double density_) { density = density_; }

inline LoboVolumetricMesh::Region::Region(int materialIndex_, int setIndex_) : setIndex(setIndex_), materialIndex(materialIndex_) {}
inline int LoboVolumetricMesh::Region::getMaterialIndex() const { return materialIndex; }
inline int LoboVolumetricMesh::Region::getSetIndex() const { return setIndex; }