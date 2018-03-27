#include "LoboTriMesh.h"
#include <iostream>
#include <fstream>
#include "TinyObjMeshControl.h"

LoboTriMesh::LoboTriMesh()
{
	initConstructor();
}


LoboTriMesh::LoboTriMesh(const char* objfilename)
{
	initConstructor();
	this->objFilename = objfilename;
	readObjModel(objfilename);
	numShapes = meshShapes.size();
	initShapeInfo();
	initEigenVector();
}

LoboTriMesh::LoboTriMesh(LoboObjMesh* objmesh)
{
	initConstructor();
}

LoboTriMesh::LoboTriMesh(const char* objfilename, bool centered)
{
	initConstructor();
	this->objFilename = objfilename;
	readObjModel(objfilename, centered);
	numShapes = meshShapes.size();
	initShapeInfo();
	initEigenVector();
}

LoboTriMesh::~LoboTriMesh()
{

}

void LoboTriMesh::finishEditMesh()
{
	copyTinyAttribute(currentVertexAttribute, originalVertexAttribute);
	
	//recompute center shape (may be scaled)
	centerTinyAttribute(originalVertexAttribute, centeredAttribute, originalToCenter);

	//set translate to 0
	setTranslateFromOri(Vector3d::Zero());
	setScaleFromCenter(1.0);
	updateEigenVector();
}

void LoboTriMesh::translateMesh(double x, double y, double z)
{
	Vector3d translate(x, y, z);
	translateMesh(translate);
}

void LoboTriMesh::translateMesh(Vector3d translate)
{
	setTranslateFromOri(translate + getOriginalToCenter());
	translateTinyAttribute(originalVertexAttribute, currentVertexAttribute, getTranslateFromOri());
	updateEigenVector();
}


void LoboTriMesh::transformMesh(Vector3d translate, Matrix3d rotation)
{
	setTranslateFromOri(translate + getOriginalToCenter());
	rotateTinyAttribute(originalVertexAttribute, currentVertexAttribute, rotation);
	translateTinyAttribute(currentVertexAttribute, currentVertexAttribute, getTranslateFromOri());
	updateEigenVector();
}

void LoboTriMesh::moveMesh(Vector3d speed)
{
	setTranslateFromOri(getTranslateFromOri()+speed);
	translateTinyAttribute(originalVertexAttribute, currentVertexAttribute, getTranslateFromOri());
	updateEigenVector();
}

void LoboTriMesh::updateTranslate()
{
	translateTinyAttribute(originalVertexAttribute, currentVertexAttribute, getTranslateFromOri());
	updateEigenVector();
}

void LoboTriMesh::scaleMeshAroundMeshcenter(double scale)
{
	setScaleFromCenter(scale);
	scaleTinyAttribute(currentVertexAttribute, currentVertexAttribute, getScaleFromCenter());
	updateEigenVector();
}

void LoboTriMesh::shapeMatching(LoboTriMesh* targettrimesh)
{
	VectorXd currentposition = this->getCurrentPositionDouble();
	VectorXd targetPosition = targettrimesh->getCurrentPositionDouble();

	std::ofstream test("test.txt");
	test << targetPosition << std::endl;
	test.close();

	double left = currentposition.transpose()*currentposition;
	double right = currentposition.transpose()*targetPosition;
	double scale = right / left;
	std::cout << left << " " << right << std::endl;
	std::cout << scale << std::endl;
	this->scaleMeshAroundMeshcenter(scale);

}

void LoboTriMesh::scaleOriginalAtCenter(double scale)
{
	setScaleFromCenter(scale);
	scaleTinyAttribute(centeredAttribute, originalVertexAttribute, getScaleFromCenter());
}

void LoboTriMesh::scaleAtCurrent(double scale)
{
	scaleTinyAttribute(currentVertexAttribute, currentVertexAttribute, scale);
}

void LoboTriMesh::scaleAtCurrentUpdate(double scale)
{
	scaleTinyAttribute(currentVertexAttribute, currentVertexAttribute, scale);
	updateEigenVector();
}

void LoboTriMesh::transformAtcurrentUpdate(Matrix3d transform, Vector3d translate)
{
	transformTinyAttribute(currentVertexAttribute, currentVertexAttribute, transform, translate);
	updateEigenVector();
}

int LoboTriMesh::getNumVertex()
{
	return numVertex;
}

tinyobj::attrib_t* LoboTriMesh::getAttributeRef()
{
	return &currentVertexAttribute;
}

std::vector<tinyobj::shape_t>* LoboTriMesh::getShapesRef()
{
	return &meshShapes;
}

std::vector<tinyobj::material_t>* LoboTriMesh::getMaterialRef()
{
	return &materials;
}

void LoboTriMesh::updateTriDisplacement(double * displacement)
{
	memcpy(globalDisPlacement.data(), displacement, sizeof(double)*globalDisPlacement.size());
	
	for (int i = 0; i < currentVertexAttribute.vertices.size(); i++)
	{
		currentVertexAttribute.vertices[i] = globalOriPosition.data()[i] + globalDisPlacement.data()[i];
	}

	setMeshUpdated(true);

}

Vector3d LoboTriMesh::getOriPosition(int vertexid)
{
	Vector3d position;
	position.data()[0] = globalOriPosition.data()[vertexid * 3 + 0];
	position.data()[1] = globalOriPosition.data()[vertexid * 3 + 1];
	position.data()[2] = globalOriPosition.data()[vertexid * 3 + 2];
	return position;
}

Eigen::Vector3d LoboTriMesh::getCurPosition(int vertexid)
{
	Vector3d position;
	position.data()[0] = globalOriPosition.data()[vertexid * 3 + 0] + globalDisPlacement.data()[vertexid * 3 + 0];

	position.data()[1] = globalOriPosition.data()[vertexid * 3 + 1] + globalDisPlacement.data()[vertexid * 3 + 1];

	position.data()[2] = globalOriPosition.data()[vertexid * 3 + 2] + globalDisPlacement.data()[vertexid * 3 + 2];

	return position;
}

VectorXf LoboTriMesh::getCurrentPositionFloat()
{
	VectorXf currentposition(numVertex*3);
	for (int i = 0; i < currentposition.size(); i++)
	{
		currentposition.data()[i] = globalOriPosition.data()[i] + globalDisPlacement.data()[i];
	}
	return currentposition;
}

Eigen::VectorXd LoboTriMesh::getCurrentPositionDouble()
{
	return globalOriPosition + globalDisPlacement;
}

VectorXi LoboTriMesh::getIndices()
{
	return globalIndices;
}

void LoboTriMesh::exportMTL(std::ofstream &outputStream)
{
	for (int i = 0; i < meshShapes.size(); i++)
	{
		tinyobj::material_t* material = &materials[meshShapes[i].mesh.material_ids[0]];

		outputStream << "newmtl " << material->name.c_str() << std::endl;
		outputStream << "illum " << material->illum << std::endl;
		
		outputStream << "Kd ";
		outputStream << material->diffuse[0] << " ";
		outputStream << material->diffuse[1] << " ";
		outputStream << material->diffuse[2] << " ";
		outputStream << std::endl;

		outputStream << "Ka ";
		outputStream << material->ambient[0] << " ";
		outputStream << material->ambient[1] << " ";
		outputStream << material->ambient[2] << " ";
		outputStream << std::endl;

		outputStream << "Ks ";
		outputStream << material->specular[0] << " ";
		outputStream << material->specular[1] << " ";
		outputStream << material->specular[2] << " ";
		outputStream << std::endl;

		outputStream << "Ns " << material->shininess << std::endl;
		outputStream << "map_Kd " << material->diffuse_texname << std::endl;
		outputStream << "map_Ks " << material->specular_texname << std::endl;
		outputStream << "map_Ns " << material->normal_texname << std::endl;
		outputStream << "bump " << material->bump_texname << std::endl;
	}
}

void LoboTriMesh::exportObj(std::ofstream &outputstream, int &vertexOffset, int &texCoordOffset)
{
	
	for (int i = 0; i < currentVertexAttribute.vertices.size() / 3; i++)
	{
		Vector3d node_position;
		node_position.data()[0] = currentVertexAttribute.vertices[i * 3 + 0];
		node_position.data()[1] = currentVertexAttribute.vertices[i * 3 + 1];
		node_position.data()[2] = currentVertexAttribute.vertices[i * 3 + 2];

		outputstream << "v " << node_position.x() << " "
			<< node_position.y() << " "
			<< node_position.z() << std::endl;
	}

	for (int i = 0; i < currentVertexAttribute.texcoords.size() / 2; i++)
	{
		outputstream << "vt " << currentVertexAttribute.texcoords[i * 2 + 0] << " " << currentVertexAttribute.texcoords[i * 2 + 1] << std::endl;
	}

	for (int i = 0; i < meshShapes.size(); i++)
	{
		outputstream << "g " << meshShapes[i].name.c_str() << tri_id << std::endl;
		outputstream << "usemtl " << materials[meshShapes[i].mesh.material_ids[0]].name.c_str() << std::endl;

		for (int f = 0; f < meshShapes[i].mesh.indices.size() / 3; f++)
		{
			outputstream << "f " << meshShapes[i].mesh.indices[f * 3 + 0].vertex_index + vertexOffset;
			outputstream << "/" << meshShapes[i].mesh.indices[f * 3 + 0].texcoord_index + texCoordOffset;

			outputstream << " " << meshShapes[i].mesh.indices[f * 3 + 1].vertex_index + vertexOffset;
			outputstream << "/" << meshShapes[i].mesh.indices[f * 3 + 1].texcoord_index + texCoordOffset;

			outputstream << " " << meshShapes[i].mesh.indices[f * 3 + 2].vertex_index + vertexOffset;
			outputstream << "/" << meshShapes[i].mesh.indices[f * 3 + 2].texcoord_index + texCoordOffset;
			outputstream << std::endl;
		}
		
	}

	vertexOffset += currentVertexAttribute.vertices.size() / 3;
	texCoordOffset += currentVertexAttribute.texcoords.size() / 2;
}

void LoboTriMesh::exportObj(const char* filename)
{
	std::ofstream output(filename);
	if (!output.is_open())
	{
		std::cout << "file not open" << std::endl;
	}

	output << "mtllib " << mtlFilename << std::endl;

	int offset = 0;
	int vtoffset = 0;
	 
	for (int i = 0; i < currentVertexAttribute.vertices.size()/3; i++)
	{
		Vector3d node_position;
		node_position.data()[0] = currentVertexAttribute.vertices[i * 3 + 0];
		node_position.data()[1] = currentVertexAttribute.vertices[i * 3 + 1];
		node_position.data()[2] = currentVertexAttribute.vertices[i * 3 + 2];

		output << "v " << node_position.x() << " "
			<< node_position.y() << " "
			<< node_position.z() << std::endl;
	}

	for (int i = 0; i < currentVertexAttribute.texcoords.size() / 2; i++)
	{
		output << "vt " << currentVertexAttribute.texcoords[i * 2 + 0] << " " << currentVertexAttribute.texcoords[i * 2 + 1] << std::endl;
	}

	for (int i = 0; i < meshShapes.size(); i++)
	{
		output << "g " << meshShapes[i].name.c_str() << std::endl;
		output << "usemtl " << materials[meshShapes[i].mesh.material_ids[0]].name.c_str() << std::endl;

		for (int f = 0; f < meshShapes[i].mesh.indices.size() / 3; f++)
		{
			output << "f " << meshShapes[i].mesh.indices[f * 3 + 0].vertex_index+1;
			output << "/" << meshShapes[i].mesh.indices[f * 3 + 0].texcoord_index + 1;

			output << " " << meshShapes[i].mesh.indices[f * 3 + 1].vertex_index + 1;
			output << "/" << meshShapes[i].mesh.indices[f * 3 + 1].texcoord_index + 1;

			output << " " << meshShapes[i].mesh.indices[f * 3 + 2].vertex_index + 1;
			output << "/" << meshShapes[i].mesh.indices[f * 3 + 2].texcoord_index + 1;
			output << std::endl;
		}
	}

	output.close();

}

std::vector<std::vector<int>>* LoboTriMesh::getMapFromTinyObjShape()
{
	return &mapFromTinyObjShape;
}

void LoboTriMesh::setVertexColorData3D(double * weights)
{
	//std::cout <<weights[0] <<" "<< weights[1] <<" " <<weights[2] << std::endl;
	for (int i = 0; i < numVertex*3; i++)
	{
		globalVertexColor.data()[i] = weights[i];
	}

	this->setMeshUpdated(true);
	//int offset = 0;
	//for (int i = 0; i < currentShapes.size(); i++)
	//{
	//	int shape_colorsize = currentShapes[i].mesh.vertexColors.size();
	//	for (int j = 0; j < shape_colorsize; j++)
	//	{
	//		currentShapes[i].mesh.vertexColors[j] = globalVertexColor.data()[offset + j];
	//	}
	//	offset += shape_colorsize;
	//}
}

void LoboTriMesh::setVertexTexCoor(double *coordinate)
{
	for (int i = 0; i < meshShapes.size(); i++)
	{

		for (int f = 0; f < meshShapes[i].mesh.indices.size() / 3; f++)
		{
			for (int j = 0; j < 3; j++)
			{
				int vertexid = meshShapes[i].mesh.indices[f * 3 + j].vertex_index;
				int textureid = meshShapes[i].mesh.indices[f * 3 + j].texcoord_index;
				currentVertexAttribute.texcoords.data()[textureid * 2 + 0] = coordinate[vertexid * 2 + 0];
				currentVertexAttribute.texcoords.data()[textureid * 2 + 1] = coordinate[vertexid * 2 + 1];
			}
		}
	}

}

float* LoboTriMesh::getVertexColorData()
{
	return globalVertexColor.data();
}

void LoboTriMesh::updateEigenVector()
{
	for (int i = 0; i < currentVertexAttribute.vertices.size(); i++)
	{
		globalOriPosition.data()[i] = currentVertexAttribute.vertices[i];
	}
	globalDisPlacement.setZero();
	setMeshUpdated(true);
}

void LoboTriMesh::initEigenVector()
{
	globalDisPlacement.resize(numVertex * 3);
	globalOriPosition.resize(numVertex * 3);
	globalIndices.resize(numFaces*3);
	
	globalVertexColor.resize(numVertex * 3);
	globalVertexColor.setZero();

	int positionoffset = 0;
	int indicesoffset = 0;

	for (int i = 0; i < currentVertexAttribute.vertices.size(); i++)
	{
		globalOriPosition.data()[i] = currentVertexAttribute.vertices[i];
	}

	currentVertexAttribute.normals.resize(currentVertexAttribute.vertices.size());
	std::fill(currentVertexAttribute.normals.begin(), currentVertexAttribute.normals.end(), 0);

	for (int i = 0; i < meshShapes.size(); i++)
	{
		for (int j = 0; j < meshShapes[i].mesh.indices.size(); j++)
		{
			globalIndices.data()[indicesoffset + j] = meshShapes[i].mesh.indices[j].vertex_index;
		}
		indicesoffset += meshShapes[i].mesh.indices.size();
	}

	globalDisPlacement.setZero();

}

void LoboTriMesh::initShapeInfo()
{
	numVertex = 0;
	numFaces = 0;
	numVertex = currentVertexAttribute.vertices.size() / 3;
	for (int i = 0; i < meshShapes.size(); i++)
	{
		numFaces += meshShapes[i].mesh.indices.size() / 3;
	}

}

void LoboTriMesh::initConstructor()
{
	getTranslateFromOri().setZero();
	scaleFromCenter = 1.0;
	setMeshUpdated(false);
	//no collision bind
	collision_id_ = -1;
	tri_id = -1;
	vertexData.resize(0);
}

bool LoboTriMesh::readObjModel(const char* filename, bool centered /*= true*/)
{
	std::string err;
	bool loadinfo = tinyobj::LoadObj(&originalVertexAttribute, &meshShapes, &materials, &err, filename, "3dmodel/", true);
	if (loadinfo == 0)
	std::cout << loadinfo << " " << err << std::endl;
	//don't need this anymore with new tiny obj version
	//readObjVertexOnly(filename);

	copyTinyAttribute(originalVertexAttribute, currentVertexAttribute);

	//read mtlib info for export obj
	std::string token;
	std::ifstream inStream(filename);
	while (true)
	{
		inStream >> token;
		if (inStream.eof())
		{
			break;
		}
		if (token == "mtllib")
		{
			inStream >> mtlFilename;
			break;
		}
	} inStream.close();

	if (centered)
	{
		centerTinyAttribute(originalVertexAttribute, currentVertexAttribute, originalToCenter);
		copyTinyAttribute(currentVertexAttribute, centeredAttribute);
		setTranslateFromOri(originalToCenter);
		std::cout <<"tri mesh ori center: "<< -originalToCenter.transpose() << std::endl;
	}
	else
	{
		originalToCenter.setZero();
		copyTinyAttribute(currentVertexAttribute, centeredAttribute);
		setTranslateFromOri(originalToCenter);
		std::cout << "tri mesh ori center: " << -originalToCenter.transpose() << std::endl;
	}

	if (loadinfo)
	{

	}
	else
	{
		std::cout << "read tri mesh error." << std::endl;
	}

	return loadinfo;
}

void LoboTriMesh::readObjVertexOnly(const char* filename)
{
	//mesh_vertex.clear();
	//std::ifstream inputstream(filename);
	//std::string token;
	//while (true)
	//{
	//	inputstream >> token;
	//	if (inputstream.eof())
	//	{
	//		break;
	//	}
	//	if (token == "v")
	//	{
	//		float x, y, z;
	//		inputstream >> x >> y >> z;
	//		mesh_vertex.push_back(Vector3f(x, y, z));
	//	}
	//}
	//numMeshVertes = mesh_vertex.size();
	//mapFromTinyObjShape.resize(oriShapes.size());

	//for (int i = 0; i < oriShapes.size(); i++)
	//{
	//	int shapePositionsize = oriShapes[i].mesh.positions.size() / 3;
	//	mapFromTinyObjShape[i].resize(shapePositionsize);
	//	for (int j = 0; j < shapePositionsize; j++)
	//	{
	//		Vector3f shape_position(oriShapes[i].mesh.positions[j * 3 + 0], oriShapes[i].mesh.positions[j * 3 + 1], oriShapes[i].mesh.positions[j * 3 + 2]);
	//		

	//		//search close node
	//		double mindistance = DBL_MAX;
	//		int bestnodeid = -1;
	//		for (int k = 0; k < mesh_vertex.size(); k++)
	//		{
	//			double distance = (mesh_vertex[k] - shape_position).norm();
	//			if (distance < mindistance)
	//			{
	//				mindistance = distance;
	//				bestnodeid = k;
	//			}
	//		}
	//		mapFromTinyObjShape[i][j] = bestnodeid;
	//	}
	//}
}
