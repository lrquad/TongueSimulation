#include "LoboObjMesh.h"
#include <iostream>
#include <cassert>
#include <fstream>
#include "LoboTriMesh/TinyObjMeshControl.h"

LoboObjMesh::LoboObjMesh() :
texture_flag_(true),
collision_flag_(true),
aabbupdate_flag_(false),
collision_render_(true),
selfcollision_flag_(false),
output_flat_(true),
funnel_radius(0.2),
moving_obj_(false)
{
	std::cout << "model init" << std::endl;
	initializeOpenGLFunctions();
	// Generate 2 VBOs
	// Initializes cube geometry and transfers it to VBOs

	if (texture_flag_)
	initTexture();
	init();
}

LoboObjMesh::LoboObjMesh(const char* filename)
	: 
	texture_flag_(true),
	collision_flag_(true),
	aabbupdate_flag_(false),
	collision_render_(true),
	selfcollision_flag_(false),
	output_flat_(true),
	funnel_radius(0.2),
	moving_obj_(false)
{
	std::cout << "model init" << std::endl;
	initializeOpenGLFunctions();
	if (!readObjModel(filename))
	{
		std::cout << "model read failed" << std::endl;
	}
	obj_filename = filename;
	//initModel();
	initShapes();

	if (texture_flag_)
	initTexture();
	
	init();

}

LoboObjMesh::~LoboObjMesh()
{
	for (int i = 0; i < shape_render.size(); i++)
	{
		if (texture_flag_&&shape_render[i].bool_texture && (shape_render[i].texture!=NULL))
		{
			shape_render[i].texture->destroy();
			shape_render[i].textureBuf->destroy();
		}
		shape_render[i].arrayBuf->destroy();
		shape_render[i].indexBuf->destroy();
	}

	tris.clear();
	verts.clear();

	if (tri_mesh.BaseTree!=NULL)
	tri_mesh.BaseTree->destroy();

	delete tri_mesh.bt_collision_shape;
	delete tri_mesh.bt_tri_vertexarray;
	delete tri_mesh.btMesh;
	delete tri_mesh.bt_collision_object;
	delete tri_mesh.bt_dynamic_aabb;
}

void LoboObjMesh::init()
{
	setAlpha(1.0);
	translate.setZero();
	modeltranslate.setZero();
}

void LoboObjMesh::initShapes()
{
	/*std::cout << "init shapes" << std::endl;
	shape_render.resize(shapes.size());
	for (int i = 0; i < shape_render.size(); i++)
	{
		shape_render[i].arrayBuf = new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
		shape_render[i].indexBuf = new QOpenGLBuffer(QOpenGLBuffer::IndexBuffer);
		shape_render[i].arrayBuf->create();
		shape_render[i].indexBuf->create();
		if (shapes[i].mesh.texcoords.size() != 0)
		{
			shape_render[i].textureBuf = new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
			shape_render[i].textureBuf->create();
		}
	}

	for (size_t i = 0; i < shape_render.size(); i++) {
		shape_render[i].arrayBuf->bind();
		shape_render[i].arrayBuf->allocate(shapes[i].mesh.positions.data(), shapes[i].mesh.positions.size()*sizeof(float));
		if (shapes[i].mesh.texcoords.size() != 0)
		{
			shape_render[i].textureBuf->bind();
			shape_render[i].textureBuf->allocate(shapes[i].mesh.texcoords.data(), shapes[i].mesh.texcoords.size()*sizeof(float));
		}
		shape_render[i].indexBuf->bind();
		shape_render[i].indexBuf->allocate(shapes[i].mesh.indices.data(), shapes[i].mesh.indices.size()*sizeof(unsigned int));
		shape_render[i].material_id = shapes[i].mesh.material_ids[0];
	}*/
}

void LoboObjMesh::saveDisplacement(int totalcols, int col, const char* filename, bool initMatrix)
{
	if (col < totalcols)
	{
		//resize matrix at col == 0 
		if (col == 0)
		{
			displacement_list.resize(displacement.rows(), totalcols);
			displacement_list.setZero();
		}

		displacement_list.col(col) = this->displacement;
	}
	else if (col == totalcols)
	{
		std::cout << "save displacement " << displacement_list.rows() << " " << displacement_list.cols() << std::endl;
		std::ofstream out(filename);
		out.precision(32);
		out << displacement_list << std::endl;
		out.close();
	}
	else
	{
		return;
	}

}

Eigen::Vector3d LoboObjMesh::getOriPosition(int positionid)
{
	Vector3d position;
	position.data()[0] = oriposition.data()[positionid * 3 + 0];
	position.data()[1] = oriposition.data()[positionid * 3 + 1];
	position.data()[2] = oriposition.data()[positionid * 3 + 2];
	return position;
}

void LoboObjMesh::saveShape(std::ofstream &outStream, int &nodeoffset, int &texoffset, bool matchfunnel)
{
	//outStream << "mtllib " << mtl_filename << std::endl;
	//int offset = 0;
	//int vtoffset = 0;
	//for (int i = 0; i < shapes.size(); i++)
	//{
	//	for (int j = 0; j < shapes[i].mesh.positions.size()/3; j++)
	//	{
	//		Vector3d node_position;
	//		node_position.data()[0] = shapes[i].mesh.positions[j * 3 + 0];
	//		node_position.data()[1] = shapes[i].mesh.positions[j * 3 + 1];
	//		node_position.data()[2] = shapes[i].mesh.positions[j * 3 + 2];

	//		if (matchfunnel)
	//		{
	//			double y = node_position.y();
	//			double radius;
	//			if (lobo::computeFunnelRadius(y, radius, funnel_radius))
	//			{
	//				Vector3d origin_radius(node_position.x(), 0, node_position.z());

	//				if (origin_radius.norm() > radius)
	//				{
	//					origin_radius = origin_radius.normalized()*radius;
	//					node_position.x() = origin_radius.x();
	//					node_position.z() = origin_radius.z();
	//				}
	//			}
	//		}

	//		outStream << "v " << node_position.x() << " "
	//			<< node_position.y() << " "
	//			<< node_position.z() << std::endl;
	//	}

	//	for (int vt = 0; vt < shapes[i].mesh.texcoords.size() / 2; vt++)
	//	{
	//		outStream << "vt " << shapes[i].mesh.texcoords[vt * 2 + 0] << " "
	//			<< shapes[i].mesh.texcoords[vt * 2 + 1] << std::endl;
	//	}

	//	outStream <<"g " << shapes[i].name.c_str() << std::endl;
	//	outStream << "usemtl " << materials[shape_render[i].material_id].name.c_str() << std::endl;
	//	for (int f = 0; f < shapes[i].mesh.indices.size() / 3; f++)
	//	{
	//		outStream << "f " << shapes[i].mesh.indices[f * 3 + 0] + nodeoffset + offset;
	//		if (shapes[i].mesh.texcoords.size())
	//		{
	//			outStream <<"/" << shapes[i].mesh.indices[f * 3 + 0] + texoffset + vtoffset;
	//		}

	//		outStream << " " << shapes[i].mesh.indices[f * 3 + 1] + nodeoffset + offset;

	//		if (shapes[i].mesh.texcoords.size())
	//		{
	//			outStream << "/" << shapes[i].mesh.indices[f * 3 + 1] + texoffset + vtoffset;
	//		}
	//		
	//		outStream << " " << shapes[i].mesh.indices[f * 3 + 2] + nodeoffset + offset ;
	//		if (shapes[i].mesh.texcoords.size())
	//		{
	//			outStream << "/" << shapes[i].mesh.indices[f * 3 + 2] + texoffset + vtoffset;
	//		}

	//		outStream << std::endl;
	//	}

	//	offset += shapes[i].mesh.positions.size() / 3;
	//	vtoffset += shapes[i].mesh.texcoords.size() / 2;
	//}
	//nodeoffset += offset;
	//texoffset += vtoffset;
}

void LoboObjMesh::updateBuffer()
{
	/*tri_mesh.translate = tri_mesh.center + modeltranslate;
	tri_mesh.modeltranslate = modeltranslate;

	for (size_t i = 0; i < shape_render.size(); i++) {
		shape_render[i].arrayBuf->bind();
		shape_render[i].arrayBuf->write(0,shapes[i].mesh.positions.data(), shapes[i].mesh.positions.size()*sizeof(float));
		shape_render[i].indexBuf->bind();
		shape_render[i].indexBuf->write(0,shapes[i].mesh.indices.data(), shapes[i].mesh.indices.size()*sizeof(unsigned int));
		shape_render[i].material_id = shapes[i].mesh.material_ids[0];
	}*/
	//updateCollisionShape();
}

void LoboObjMesh::setDiffuseColor(double R, double G, double B)
{
	for (int i = 0; i < materials.size(); i++)
	{
		materials[i].diffuse[0] = R;
		materials[i].diffuse[1] = G;
		materials[i].diffuse[2] = B;
	}
}

void LoboObjMesh::initTexture()
{
	std::cout << "init Texture" << std::endl;
	for (size_t i = 0; i < shape_render.size(); i++) {
		if (materials[shape_render[i].material_id].diffuse_texname != "")
		{
			QString str = "texture/";
			str.append(materials[shape_render[i].material_id].diffuse_texname.c_str());
			shape_render[i].texture = new QOpenGLTexture(QImage(str).mirrored());
			shape_render[i].texture->setMaximumAnisotropy(8);

			shape_render[i].texture->setMinificationFilter(QOpenGLTexture::LinearMipMapLinear);
			shape_render[i].texture->setMagnificationFilter(QOpenGLTexture::Linear);
			shape_render[i].texture->setWrapMode(QOpenGLTexture::Repeat);
			shape_render[i].bool_texture = true;
		}
		else
		{
			shape_render[i].bool_texture = false;
		}
	}
}

const char* LoboObjMesh::getMeshName()
{
	return obj_filename.c_str();
}

bool LoboObjMesh::readObjModel(const char* filename,bool centered)
{
	//obj_filename = filename;
	//std::cout << "Loading " << filename << "...";
	//std::string err;
	//bool ret = tinyobj::LoadObj(ori_shapes, materials, err, filename, "3dmodel/", true);
	//copyTinyShape(ori_shapes, shapes);

	//std::cout << " finished. "<< std::endl;
	////read mtlib info
	//std::string token;
	//std::ifstream inStream(filename);
	//while (true)
	//{
	//	inStream >> token;
	//	if (inStream.eof())
	//	{
	//		break;
	//	}
	//	if (token == "mtllib")
	//	{
	//		inStream >> mtl_filename;
	//		break;
	//	}
	//} inStream.close();

	//
	//if (centered)
	//{
	//	//lobo::centerTinyShape(shapes, tri_mesh.center);
	//	centerTinyShape(ori_shapes, shapes, tri_mesh.center);
	//	copyTinyShape(shapes, centered_shapes);
	//	translate = tri_mesh.center;
	//}

	//if (ret)
	//{
	//	initDisplacement();
	//	initCollisionShape();
	//}



	//printInfo(shapes, materials, true);
	
	return 1;
}

void LoboObjMesh::initCollisionShape()
{
	std::cout << "init collision shape ... ";
	tri_mesh.positions_current = Map<VectorXd>(tri_mesh.positions->data(), tri_mesh.positions->size()).cast<float>() + Map<VectorXd>(tri_mesh.displacement.data(), tri_mesh.displacement.size()).cast<float>();

	tri_mesh.bt_tri_vertexarray = new btTriangleIndexVertexArray(tri_mesh.indices.size() / 3, tri_mesh.indices.data(), 3 * sizeof(int), tri_mesh.positions->size() / 3, (float*)tri_mesh.positions_current.data(), 3 * sizeof(float));

	tri_mesh.bt_collision_shape = new btGImpactMeshShape(tri_mesh.bt_tri_vertexarray);
	tri_mesh.bt_collision_shape->updateBound();
	tri_mesh.bt_dynamic_aabb = new btDbvt();
	tri_mesh.coldet_model = newCollisionModel3D(false);

	for (int i = 0; i < tri_mesh.indices.size() / 3; i++)
	{
		float trianglepoints[3][3];
		int v1, v2, v3;
		v1 = tri_mesh.indices.data()[i * 3 + 0];
		v2 = tri_mesh.indices.data()[i * 3 + 1];
		v3 = tri_mesh.indices.data()[i * 3 + 2];
		trianglepoints[0][0] = tri_mesh.positions->data()[v1 * 3] + tri_mesh.displacement.data()[v1 * 3];
		trianglepoints[0][1] = tri_mesh.positions->data()[v1 * 3 + 1] + tri_mesh.displacement.data()[v1 * 3 + 1];
		trianglepoints[0][2] = tri_mesh.positions->data()[v1 * 3 + 2] + tri_mesh.displacement.data()[v1 * 3 + 2];
		trianglepoints[1][0] = tri_mesh.positions->data()[v2 * 3] + tri_mesh.displacement.data()[v2 * 3];
		trianglepoints[1][1] = tri_mesh.positions->data()[v2 * 3 + 1] + tri_mesh.displacement.data()[v2 * 3 + 1];
		trianglepoints[1][2] = tri_mesh.positions->data()[v2 * 3 + 2] + tri_mesh.displacement.data()[v2 * 3 + 2];
		trianglepoints[2][0] = tri_mesh.positions->data()[v3 * 3] + tri_mesh.displacement.data()[v3 * 3];
		trianglepoints[2][1] = tri_mesh.positions->data()[v3 * 3 + 1] + tri_mesh.displacement.data()[v3 * 3 + 1];
		trianglepoints[2][2] = tri_mesh.positions->data()[v3 * 3 + 2] + tri_mesh.displacement.data()[v3 * 3 + 2];
		tri_mesh.coldet_model->addTriangle(&trianglepoints[0][0], &trianglepoints[1][0], &trianglepoints[2][0]);
	}
	tri_mesh.coldet_model->finalize();
	tri_mesh.bt_collision_object = new btCollisionObject();
	tri_mesh.bt_collision_object->setCollisionShape(tri_mesh.bt_collision_shape);
}

void LoboObjMesh::updateCollisionShape()
{
	if (!collision_flag_)
	return;

	delete tri_mesh.bt_tri_vertexarray;
	delete tri_mesh.bt_collision_shape;

	tri_mesh.positions_current = Map<VectorXd>(tri_mesh.positions->data(), tri_mesh.positions->size()).cast<float>() + Map<VectorXd>(tri_mesh.displacement.data(), tri_mesh.displacement.size()).cast<float>();

	tri_mesh.bt_tri_vertexarray = new btTriangleIndexVertexArray(tri_mesh.indices.size() / 3, tri_mesh.indices.data(), 3 * sizeof(int), tri_mesh.positions->size() / 3, (float*)tri_mesh.positions_current.data(), 3 * sizeof(float));

	tri_mesh.bt_collision_shape = new btGImpactMeshShape(tri_mesh.bt_tri_vertexarray);
	tri_mesh.bt_collision_shape->updateBound();
	tri_mesh.bt_collision_object->setCollisionShape(tri_mesh.bt_collision_shape);
}

void LoboObjMesh::updateAABBtree()
{
	if (!aabbupdate_flag_)
	{
		return;
	}
	delete tri_mesh.coldet_model;

	tri_mesh.coldet_model = newCollisionModel3D(false);
	
	for (int i = 0; i < tri_mesh.indices.size() / 3; i++)
	{
		float trianglepoints[3][3];
		int v1, v2, v3;
		v1 = tri_mesh.indices.data()[i * 3 + 0];
		v2 = tri_mesh.indices.data()[i * 3 + 1];
		v3 = tri_mesh.indices.data()[i * 3 + 2];
		trianglepoints[0][0] = tri_mesh.positions->data()[v1 * 3] + tri_mesh.displacement.data()[v1 * 3];
		trianglepoints[0][1] = tri_mesh.positions->data()[v1 * 3 + 1] + tri_mesh.displacement.data()[v1 * 3 + 1];
		trianglepoints[0][2] = tri_mesh.positions->data()[v1 * 3 + 2] + tri_mesh.displacement.data()[v1 * 3 + 2];
		trianglepoints[1][0] = tri_mesh.positions->data()[v2 * 3] + tri_mesh.displacement.data()[v2 * 3];
		trianglepoints[1][1] = tri_mesh.positions->data()[v2 * 3 + 1] + tri_mesh.displacement.data()[v2 * 3 + 1];
		trianglepoints[1][2] = tri_mesh.positions->data()[v2 * 3 + 2] + tri_mesh.displacement.data()[v2 * 3 + 2];
		trianglepoints[2][0] = tri_mesh.positions->data()[v3 * 3] + tri_mesh.displacement.data()[v3 * 3];
		trianglepoints[2][1] = tri_mesh.positions->data()[v3 * 3 + 1] + tri_mesh.displacement.data()[v3 * 3 + 1];
		trianglepoints[2][2] = tri_mesh.positions->data()[v3 * 3 + 2] + tri_mesh.displacement.data()[v3 * 3 + 2];
		tri_mesh.coldet_model->addTriangle(&trianglepoints[0][0], &trianglepoints[1][0], &trianglepoints[2][0]);
	}
	tri_mesh.coldet_model->finalize();
	aabbupdate_flag_ = false;
}

void LoboObjMesh::selfCollisionDetection()
{
	//check if we need self collision
	if (!this->deform_flag_)
	{
		return;
	}

	//check if we want self collision
	if (!this->selfcollision_flag_)
	{
		return;
	}

	//If collision shape not update before we have to update that.
	//This means if we want self collision we have update this every time step.
	if (!aabbupdate_flag_)
	{
		if (tri_mesh.coldet_model!=NULL)
		delete tri_mesh.coldet_model;

		tri_mesh.coldet_model = newCollisionModel3D(false);

		for (int i = 0; i < tri_mesh.indices.size() / 3; i++)
		{
			float trianglepoints[3][3];
			int v1, v2, v3;
			v1 = tri_mesh.indices.data()[i * 3 + 0];
			v2 = tri_mesh.indices.data()[i * 3 + 1];
			v3 = tri_mesh.indices.data()[i * 3 + 2];
			trianglepoints[0][0] = tri_mesh.positions->data()[v1 * 3] + tri_mesh.displacement.data()[v1 * 3];
			trianglepoints[0][1] = tri_mesh.positions->data()[v1 * 3 + 1] + tri_mesh.displacement.data()[v1 * 3 + 1];
			trianglepoints[0][2] = tri_mesh.positions->data()[v1 * 3 + 2] + tri_mesh.displacement.data()[v1 * 3 + 2];
			trianglepoints[1][0] = tri_mesh.positions->data()[v2 * 3] + tri_mesh.displacement.data()[v2 * 3];
			trianglepoints[1][1] = tri_mesh.positions->data()[v2 * 3 + 1] + tri_mesh.displacement.data()[v2 * 3 + 1];
			trianglepoints[1][2] = tri_mesh.positions->data()[v2 * 3 + 2] + tri_mesh.displacement.data()[v2 * 3 + 2];
			trianglepoints[2][0] = tri_mesh.positions->data()[v3 * 3] + tri_mesh.displacement.data()[v3 * 3];
			trianglepoints[2][1] = tri_mesh.positions->data()[v3 * 3 + 1] + tri_mesh.displacement.data()[v3 * 3 + 1];
			trianglepoints[2][2] = tri_mesh.positions->data()[v3 * 3 + 2] + tri_mesh.displacement.data()[v3 * 3 + 2];
			tri_mesh.coldet_model->addTriangle(&trianglepoints[0][0], &trianglepoints[1][0], &trianglepoints[2][0]);
		}
		tri_mesh.coldet_model->finalize();
	}

	//loop all triangle to detection.
	for (int i = 0; i < tri_mesh.indices.size() / 3; i++)
	{
		if (tri_mesh.singletriangle_model != NULL)
		delete tri_mesh.singletriangle_model;

		tri_mesh.singletriangle_model = newCollisionModel3D(true);
		float trianglepoints[3][3];
		int v1, v2, v3;
		v1 = tri_mesh.indices.data()[i * 3 + 0];
		v2 = tri_mesh.indices.data()[i * 3 + 1];
		v3 = tri_mesh.indices.data()[i * 3 + 2];
		trianglepoints[0][0] = tri_mesh.positions->data()[v1 * 3] + tri_mesh.displacement.data()[v1 * 3];
		trianglepoints[0][1] = tri_mesh.positions->data()[v1 * 3 + 1] + tri_mesh.displacement.data()[v1 * 3 + 1];
		trianglepoints[0][2] = tri_mesh.positions->data()[v1 * 3 + 2] + tri_mesh.displacement.data()[v1 * 3 + 2];
		trianglepoints[1][0] = tri_mesh.positions->data()[v2 * 3] + tri_mesh.displacement.data()[v2 * 3];
		trianglepoints[1][1] = tri_mesh.positions->data()[v2 * 3 + 1] + tri_mesh.displacement.data()[v2 * 3 + 1];
		trianglepoints[1][2] = tri_mesh.positions->data()[v2 * 3 + 2] + tri_mesh.displacement.data()[v2 * 3 + 2];
		trianglepoints[2][0] = tri_mesh.positions->data()[v3 * 3] + tri_mesh.displacement.data()[v3 * 3];
		trianglepoints[2][1] = tri_mesh.positions->data()[v3 * 3 + 1] + tri_mesh.displacement.data()[v3 * 3 + 1];
		trianglepoints[2][2] = tri_mesh.positions->data()[v3 * 3 + 2] + tri_mesh.displacement.data()[v3 * 3 + 2];
		tri_mesh.singletriangle_model->addTriangle(&trianglepoints[0][0], &trianglepoints[1][0], &trianglepoints[2][0]);
		tri_mesh.singletriangle_model->finalize();

		if (tri_mesh.singletriangle_model->collision(tri_mesh.coldet_model))
		{
			std::vector<int> coollide_tri_list1;
			std::vector<int> coollide_tri_list2;
			tri_mesh.singletriangle_model->getCollidingTriangleList(
				coollide_tri_list1,
				coollide_tri_list2
				);

			int notneighbor = 0;
			//check if these collied triangle is neighbor or itself.
			for (int triangleid = 0; triangleid < coollide_tri_list2.size(); triangleid++)
			{
				int triid = coollide_tri_list2[triangleid];
				bool isneighbor = false;

				for (int j = 0; j < 3; j++)
				{
					int nodeid = tri_mesh.indices.data()[triid * 3 + j];
					float x = tri_mesh.positions->data()[nodeid * 3] + tri_mesh.displacement.data()[nodeid * 3];
					float y = tri_mesh.positions->data()[nodeid * 3 + 1] + tri_mesh.displacement.data()[nodeid * 3 + 1];
					float z = tri_mesh.positions->data()[nodeid * 3 + 2] + tri_mesh.displacement.data()[nodeid * 3 + 2];

					for (int xi = 0; xi < 3; xi++)
					{
						if (x == trianglepoints[xi][0]&&
							y == trianglepoints[xi][1]&&
							z == trianglepoints[xi][2]
							)
						{
							isneighbor = true;
						}
					}
				}
				if (!isneighbor)
				{
					lobo::computeIntersectionForce(coollide_tri_list1[triangleid], &tri_mesh, coollide_tri_list2[triangleid], &tri_mesh,true);
					notneighbor++;
				}
			}
		}
	}
	
}

void LoboObjMesh::translateObjModel(double x, double y, double z)
{
	/*modeltranslate = Vector3d(x, y, z);

	translate = Vector3d(x, y, z) + tri_mesh.center;
	translateTinyShape(ori_shapes, shapes, translate);

	int numberofposition = 0;
	for (int i = 0; i < shapes.size(); i++)
	{
		for (int j = 0; j < shapes[i].mesh.positions.size(); j++)
		{
			oriposition.data()[numberofposition + j] = shapes[i].mesh.positions[j];
		}
		numberofposition += shapes[i].mesh.positions.size();
	}

	this->updateBuffer();*/
}

void LoboObjMesh::scaleObjModel(double s)
{
	/*this->scale = s;

	scaleTineShape(shapes, shapes, scale);

	int numberofposition = 0;
	for (int i = 0; i < shapes.size(); i++)
	{
		for (int j = 0; j < shapes[i].mesh.positions.size(); j++)
		{
			oriposition.data()[numberofposition + j] = shapes[i].mesh.positions[j];
		}
		numberofposition += shapes[i].mesh.positions.size();
	}

	this->updateBuffer();*/
}

void LoboObjMesh::scaleObjAtCenter(double scale)
{
	/*this->scale = scale;

	scaleTineShape(centered_shapes, ori_shapes, scale);

	int numberofposition = 0;
	for (int i = 0; i < shapes.size(); i++)
	{
		for (int j = 0; j < shapes[i].mesh.positions.size(); j++)
		{
			oriposition.data()[numberofposition + j] = shapes[i].mesh.positions[j];
		}
		numberofposition += shapes[i].mesh.positions.size();
	}

	this->updateBuffer();*/
}

void LoboObjMesh::initDisplacement()
{
	/*int numberofposition = 0;
	int numberofindices = 0;
	for (int i = 0; i < shapes.size(); i++)
	{
		numberofposition += shapes[i].mesh.positions.size();
		numberofindices += shapes[i].mesh.indices.size();
	}
	oriposition.resize(numberofposition);
	indices.resize(numberofindices);
	displacement.resize(numberofposition);
	displacement.setZero();
	numberofposition = 0;
	numberofindices = 0;
	for (int i = 0; i < shapes.size(); i++)
	{
		for (int j = 0; j < shapes[i].mesh.indices.size(); j++)
		{
			indices.data()[numberofindices + j] = shapes[i].mesh.indices[j] + numberofposition/3;
		}
		numberofindices += shapes[i].mesh.indices.size();

		for (int j = 0; j < shapes[i].mesh.positions.size(); j++)
		{
			oriposition.data()[numberofposition + j] = shapes[i].mesh.positions[j];
		}
		numberofposition += shapes[i].mesh.positions.size();
		
	}
	tri_mesh.displacement = displacement;
	tri_mesh.positions = &oriposition;
	tri_mesh.indices = indices;*/
}

void LoboObjMesh::objUniformMotion(Vector3d step_moving)
{
	this->tri_mesh.displacement;

	for (int i = 0; i < tri_mesh.displacement.size() / 3; i++)
	{
		tri_mesh.displacement.data()[i * 3 + 0] += step_moving.data()[0];
		tri_mesh.displacement.data()[i * 3 + 1] += step_moving.data()[1];
		tri_mesh.displacement.data()[i * 3 + 2] += step_moving.data()[2];
	}
	updateDisplacement(tri_mesh.displacement.data());
}


void LoboObjMesh::updateDisplacement(const double * dis)
{
	//memcpy(displacement.data(), dis, displacement.size()*sizeof(double));

	//int numberofposition = 0;
	//
	//for (int i = 0; i < shapes.size(); i++)
	//{
	//	for (int j = 0; j < shapes[i].mesh.positions.size(); j++)
	//	{
	//		shapes[i].mesh.positions[j] = oriposition.data()[numberofposition + j] + displacement.data()[numberofposition+j];
	//	}
	//	numberofposition += shapes[i].mesh.positions.size();
	//}

	//updateBuffer();
}

void LoboObjMesh::printInfo(const std::vector<tinyobj::shape_t>& shapes, const std::vector<tinyobj::material_t>& materials, bool triangulate)
{
	/*std::cout << "# of shapes    : " << shapes.size() << std::endl;
	std::cout << "# of materials : " << materials.size() << std::endl;

	for (size_t i = 0; i < shapes.size(); i++) {
		printf("shape[%ld].name = %s\n", i, shapes[i].name.c_str());
		printf("Size of shape[%ld].indices: %ld\n", i, shapes[i].mesh.indices.size());

		if (triangulate)
		{
			printf("Size of shape[%ld].material_ids: %ld\n", i, shapes[i].mesh.material_ids.size());
			assert((shapes[i].mesh.indices.size() % 3) == 0);
		}

		printf("shape[%ld].num_faces: %ld\n", i, shapes[i].mesh.num_vertices.size());

		printf("shape[%ld].vertices: %ld\n", i, shapes[i].mesh.positions.size());
		assert((shapes[i].mesh.positions.size() % 3) == 0);
	}

	

	for (size_t i = 0; i < materials.size(); i++) {
		printf("material[%ld].name = %s\n", i, materials[i].name.c_str());
		std::map<std::string, std::string>::const_iterator it(materials[i].unknown_parameter.begin());
		std::map<std::string, std::string>::const_iterator itEnd(materials[i].unknown_parameter.end());

		for (; it != itEnd; it++) {
			printf("  material.%s = %s\n", it->first.c_str(), it->second.c_str());
		}
		printf("\n");
	}*/
}

void LoboObjMesh::drawModel(QOpenGLShaderProgram *program, double alpha /*=1.0*/)
{
	AmbientProduct_loc = program->uniformLocation("AmbientProduct");
	DiffuseProduct_loc = program->uniformLocation("DiffuseProduct");
	SpecularProduct_loc = program->uniformLocation("SpecularProduct");
	Shininess_loc = program->uniformLocation("Shininess");

	for (int i = 0; i < shape_render.size(); i++)
	{
		drawShape(program, shape_render[i], this->getAlpha());
	}

	if (collision_render_)
	if (this->tri_mesh.coollide_tri_list.size()>0)
	{
		int BoolPoint_loc = program->uniformLocation("point");
		program->setUniformValue(BoolPoint_loc, 1);
		glPointSize(6);

		

		QVector4D color = QVector4D(1, 0, 0, 1);
		program->setUniformValue(DiffuseProduct_loc, color);

		glBegin(GL_POINTS);
	
		for (int i = 0; i < this->tri_mesh.coollide_tri_list.size(); i++)
		{
			int triid = tri_mesh.coollide_tri_list[i];
			int v1, v2, v3;
			v1 = this->tri_mesh.indices.data()[triid * 3 + 0];
			v2 = this->tri_mesh.indices.data()[triid * 3 + 1];
			v3 = this->tri_mesh.indices.data()[triid * 3 + 2];
			glVertex3f(tri_mesh.positions->data()[v1 * 3] + tri_mesh.displacement.data()[v1 * 3],
				tri_mesh.positions->data()[v1 * 3 + 1] + tri_mesh.displacement.data()[v1 * 3 + 1],
				tri_mesh.positions->data()[v1 * 3 + 2] + tri_mesh.displacement.data()[v1 * 3 + 2]);

			glVertex3f(tri_mesh.positions->data()[v2 * 3] + tri_mesh.displacement.data()[v2 * 3],
				tri_mesh.positions->data()[v2 * 3 + 1] + tri_mesh.displacement.data()[v2 * 3 + 1],
				tri_mesh.positions->data()[v2 * 3 + 2] + tri_mesh.displacement.data()[v2 * 3 + 2]);

			glVertex3f(tri_mesh.positions->data()[v3 * 3] + tri_mesh.displacement.data()[v3 * 3],
				tri_mesh.positions->data()[v3 * 3 + 1] + tri_mesh.displacement.data()[v3 * 3 + 1],
				tri_mesh.positions->data()[v3 * 3 + 2] + tri_mesh.displacement.data()[v3 * 3 + 2]);
		}
		glEnd();
		program->setUniformValue(BoolPoint_loc, 0);
	}
}

void LoboObjMesh::drawShape(QOpenGLShaderProgram *program, RenderBufferObj &renderobj, double alpha/*=1.0*/)
{
	if (texture_flag_&&renderobj.bool_texture)
	{
		renderobj.texture->bind();
		renderobj.textureBuf->bind();

		int textueLocation = program->attributeLocation("a_texcoord");
		program->enableAttributeArray(textueLocation);
		program->setAttributeBuffer(textueLocation, GL_FLOAT, 0, 2, 0);
	}
	program->setUniformValue(AmbientProduct_loc, QVector4D(materials[renderobj.material_id].ambient[0],
		materials[renderobj.material_id].ambient[1],
		materials[renderobj.material_id].ambient[2], alpha));

	program->setUniformValue(DiffuseProduct_loc, QVector4D(materials[renderobj.material_id].diffuse[0],
		materials[renderobj.material_id].diffuse[1],
		materials[renderobj.material_id].diffuse[2], alpha));

	program->setUniformValue(SpecularProduct_loc, QVector4D(materials[renderobj.material_id].specular[0] ,
		materials[renderobj.material_id].specular[1] ,
		materials[renderobj.material_id].specular[2], alpha));
	program->setUniformValue(Shininess_loc, GLfloat(64));

	quintptr offset = 0;
	renderobj.arrayBuf->bind();
	int vertexLocation = program->attributeLocation("a_position");
	program->enableAttributeArray(vertexLocation);
	program->setAttributeBuffer(vertexLocation, GL_FLOAT, offset, 3, 0);

	renderobj.indexBuf->bind();
	glDrawElements(GL_TRIANGLES, renderobj.indexBuf->size() / sizeof(GL_UNSIGNED_INT), GL_UNSIGNED_INT, 0);

	renderobj.arrayBuf->release();
	renderobj.indexBuf->release();

	if (texture_flag_&&renderobj.bool_texture)
	{
		renderobj.texture->release();
		renderobj.textureBuf->release();
	}
}