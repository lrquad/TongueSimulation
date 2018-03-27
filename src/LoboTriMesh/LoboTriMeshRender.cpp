#include "LoboTriMeshRender.h"
#include <iostream>
#include "LoboTriMesh/LoboTriMesh.h"
#include <vector>
#include "Simulator/collision/TriMeshCollisionDetector.h"
#include "Render/SphereRender.h"
#include "TinyObjMeshControl.h"
#include <QElapsedTimer>


LoboTriMeshRender::LoboTriMeshRender(LoboTriMesh* trimesh)
{
	this->trimesh = trimesh;
	shapes = trimesh->getShapesRef();
	materials = trimesh->getMaterialRef();
	tri_attributes = trimesh->getAttributeRef();
	setSpecial_mesh(false);
	setAcceptwireframe(false);
	initializeOpenGLFunctions();
	initRender();
	initShapeBuffer();
	initTexture();
}

LoboTriMeshRender::LoboTriMeshRender(LoboTriMesh* trimesh, bool isspecial)
{
	this->special_mesh = isspecial;
	this->trimesh = trimesh;
	shapes = trimesh->getShapesRef();
	materials = trimesh->getMaterialRef();
	tri_attributes = trimesh->getAttributeRef();
	setAcceptwireframe(false);
	initializeOpenGLFunctions();
	initRender();
	initShapeBuffer();
	initTexture();
}

LoboTriMeshRender::~LoboTriMeshRender()
{
	for (int i = 0; i < shapeBuffer.size(); i++)
	{
		if (shapeBuffer[i].bool_texture && (shapeBuffer[i].texture != NULL))
		{
			shapeBuffer[i].texture->destroy();
		}
		shapeBuffer[i].arrayBuf->destroy();
		delete shapeBuffer[i].arrayBuf;
	}
}

void LoboTriMeshRender::renderModel(QOpenGLShaderProgram *program)
{
	if (!getVisible())
	{
		return;
	}
	if (getIsTransparent())
	{
		glEnable(GL_CULL_FACE);
		glCullFace(GL_BACK);
		glEnable(GL_BLEND); //Enable blending.
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	}

	AmbientProduct_loc = program->uniformLocation("AmbientProduct");
	DiffuseProduct_loc = program->uniformLocation("DiffuseProduct");
	SpecularProduct_loc = program->uniformLocation("SpecularProduct");
	Shininess_loc = program->uniformLocation("Shininess");

	int BoolPoint_loc = program->uniformLocation("smoothshading");
	int boolusecolor_location = program->uniformLocation("useinputcolor");
	int acceptShadow_loc = program->uniformLocation("acceptShadow");

	if (getSmoothshading())
	{
		program->setUniformValue(BoolPoint_loc, 1);
	}
	else
	{
		program->setUniformValue(BoolPoint_loc, 0);
	}
	
	//will be controled by shader
	/*if (getRenderVertexColor())
	{
		program->setUniformValue(boolusecolor_location, 1);
	}
	else
	{
		program->setUniformValue(boolusecolor_location, 0);
	}*/

	if (getAcceptShadow())
	{
		program->setUniformValue(acceptShadow_loc, 1);
	}
	else
	{
		program->setUniformValue(acceptShadow_loc, 0);
	}

	int numShapes = trimesh->getNumShapes();
	for (int i = 0; i < numShapes; i++)
	{
		drawShape(program, shapeBuffer[i], this->getAlphaChannel());
	}

	program->setUniformValue(acceptShadow_loc, 0);
	program->setUniformValue(boolusecolor_location, 0);
	program->setUniformValue(BoolPoint_loc, 0);


	if (getRenderCollision())
	{
		renderCollisionInfo(program);
	}

	if (getIsTransparent())
	{
		//glDisable(GL_DEPTH_TEST);
		glDisable(GL_BLEND);
		glDisable(GL_CULL_FACE);
		glDisable(GL_BLEND);
	}

	
}

void LoboTriMeshRender::renderCollisionInfo(QOpenGLShaderProgram *program)
{
	int numCollideInfo = collisiondetection->getNumCollideInfoSize();

	for (int i = 0; i < numCollideInfo; i++)
	{
		CollideInfo info  = collisiondetection->getCollideInfo(i);
		Vector3d position = trimesh->getOriPosition(info.nodeid);
		sphere1->drawMesh(program, position, 0.1, 3);
	}
}

void LoboTriMeshRender::updateBuffer()
{
	if (!trimesh->getMeshUpdated())
	{
		return;
	}

	if (getSmoothshading())
	updateNorms();

	int numShapes = trimesh->getNumShapes();

	for (int i = 0; i < numShapes; i++)
	{
		//convert buffer
		updateShapeArrayBuffer(i, shapeBuffer[i].sizePerVertex);
		shapeBuffer[i].arrayBuf->bind();
		shapeBuffer[i].arrayBuf->write(0, shapeBuffer[i].vb.data(), shapeBuffer[i].vb.size()*sizeof(float));
		shapeBuffer[i].material_id = (*shapes)[i].mesh.material_ids[0];
	}
	trimesh->setMeshUpdated(false);
}

void LoboTriMeshRender::setDiffuesColor(double R, double G, double B)
{
	for (int i = 0; i < materials->size(); i++)
	{
		(*materials)[i].diffuse[0] = R;
		(*materials)[i].diffuse[1] = G;
		(*materials)[i].diffuse[2] = B;
	}
}

void LoboTriMeshRender::registerCollisionDetection(TriMeshCollisionDetector* collisiondetection)
{
	this->collisiondetection = collisiondetection;
	setRenderCollision(true);
}

void LoboTriMeshRender::setRenderCollision(bool val)
{
	if (collisiondetection == NULL)
	{
		ifrenderCollision = false;
		return;
	}
	ifrenderCollision = val;
}

void LoboTriMeshRender::updateNorms()
{
	int numtrivertex = trimesh->getNumMeshVertes();
	updateSmoothNorm(*tri_attributes, *shapes);
}

void LoboTriMeshRender::drawShape(QOpenGLShaderProgram *program, RenderBufferShape &shapebuffer, double alpha /*= 1.0*/)
{
	//texture
	int textureloc = program->uniformLocation("usetexture");
	if (getUseTexutre() && shapebuffer.bool_texture)
	{
		glActiveTexture(GL_TEXTURE0);
		shapebuffer.texture->bind();
		int texLoc = program->uniformLocation("colortexture");
		glUniform1i(texLoc, 0);
	}

	if (!shapebuffer.bool_texture)
	{
		program->setUniformValue(textureloc, 0);
	}

	//set color

	program->setUniformValue(AmbientProduct_loc, QVector4D((*materials)[shapebuffer.material_id].ambient[0],
		(*materials)[shapebuffer.material_id].ambient[1],
		(*materials)[shapebuffer.material_id].ambient[2], alpha));

	program->setUniformValue(DiffuseProduct_loc, QVector4D((*materials)[shapebuffer.material_id].diffuse[0],
		(*materials)[shapebuffer.material_id].diffuse[1],
		(*materials)[shapebuffer.material_id].diffuse[2], alpha));

	program->setUniformValue(SpecularProduct_loc, QVector4D((*materials)[shapebuffer.material_id].specular[0],
		(*materials)[shapebuffer.material_id].specular[1],
		(*materials)[shapebuffer.material_id].specular[2], alpha));

	program->setUniformValue(Shininess_loc, GLfloat((*materials)[shapebuffer.material_id].shininess));

	quintptr offset = 0;


	shapebuffer.arrayBuf->bind();

	int vertexcolorloc = program->attributeLocation("a_inputcolor");
	glEnableVertexAttribArray(vertexcolorloc);
	 glVertexAttribPointer(vertexcolorloc, 3, GL_FLOAT, GL_FALSE, shapebuffer.sizePerVertex * sizeof(float), (const void*)(sizeof(float) * 8));

	int textueLocation = program->attributeLocation("a_texcoord");
	glEnableVertexAttribArray(textueLocation);
	glVertexAttribPointer(textueLocation, 2, GL_FLOAT, GL_FALSE, shapebuffer.sizePerVertex * sizeof(float), (const void*)(sizeof(float) * 6));

	int normLocation = program->attributeLocation("a_normal");
	glEnableVertexAttribArray(normLocation);
	glVertexAttribPointer(normLocation, 3, GL_FLOAT, GL_FALSE, shapebuffer.sizePerVertex * sizeof(float), (const void*)(sizeof(float) * 3));
	
	int vertexLocation = program->attributeLocation("a_position");
	glEnableVertexAttribArray(vertexLocation);
	glVertexAttribPointer(vertexLocation, 3, GL_FLOAT, GL_FALSE, shapebuffer.sizePerVertex * sizeof(float), (const void*)0);



	glDrawArrays(GL_TRIANGLES, 0, shapebuffer.vb.size() / shapebuffer.sizePerVertex);

	//shapebuffer.indexBuf->bind();
	//glDrawElements(GL_TRIANGLES, shapebuffer.indexBuf->size() / sizeof(GL_UNSIGNED_INT), GL_UNSIGNED_INT, 0);

	shapebuffer.arrayBuf->release();

	glDisableVertexAttribArray(textueLocation);
	glDisableVertexAttribArray(vertexLocation);
	glDisableVertexAttribArray(normLocation);
	glDisableVertexAttribArray(vertexcolorloc);

	if (getUseTexutre() && shapebuffer.bool_texture)
	{
		shapebuffer.texture->release();
	}

	if (!shapebuffer.bool_texture)
	{
		program->setUniformValue(textureloc, 1);
	}
}

//need update later
void LoboTriMeshRender::drawShapeFlat(QOpenGLShaderProgram *program,tinyobj::shape_t* shape)
{
	int DiffuseProduct_loc = program->uniformLocation("DiffuseProduct");
	

	//double size = shape->mesh.indices.size();
	//for (int j = 0; j < size/3; j++)
	//{
	//	int nv[3];
	//	nv[0] = shape->mesh.indices[j * 3 + 0].vertex_index;
	//	nv[1] = shape->mesh.indices[j * 3 + 1].vertex_index;
	//	nv[2] = shape->mesh.indices[j * 3 + 2].vertex_index;
	//	
	//	Vector3d colorv3 = Vector3d::Zero();
	//	Vector3d tempcolor[3];

	//	for (int t = 0; t < 3; t++)
	//	{
	//		tempcolor[t][0] = shape->mesh.vertexColors[nv[t] * 3 + 0];
	//		tempcolor[t][1] = shape->mesh.vertexColors[nv[t] * 3 + 1];
	//		tempcolor[t][2] = shape->mesh.vertexColors[nv[t] * 3 + 2];
	//	} 

	//	for (int t = 1; t < 3; t++)
	//	{
	//		double error = (tempcolor[t] - tempcolor[0]).norm();
	//		if (error < 0.01)
	//		{
	//			colorv3 = tempcolor[t];
	//			break;
	//		}
	//		else
	//		{
	//			if (t == 2)
	//			{
	//				colorv3 = tempcolor[t];
	//			}
	//		}
	//	}

	//	QVector4D color = QVector4D(colorv3.data()[0], colorv3.data()[1], colorv3.data()[2], 1.0);
	//	program->setUniformValue(DiffuseProduct_loc, color);
	//	glBegin(GL_TRIANGLES);
	//	for (int t = 0; t < 3; t++)
	//	{
	//		glVertex3f(shape->mesh.positions[nv[t] * 3 + 0], shape->mesh.positions[nv[t] * 3 + 1], shape->mesh.positions[nv[t] * 3 + 2]);
	//	}	
	//	glEnd();

	//}
}

void LoboTriMeshRender::initShapeBuffer()
{
	int numShapes = trimesh->getNumShapes();

	shapeBuffer.resize(numShapes);
	//init buffer
	for (int i = 0; i < numShapes; i++)
	{
		shapeBuffer[i].sizePerVertex = 11;
		int buffersize = shapes->at(i).mesh.indices.size()*(shapeBuffer[i].sizePerVertex);
		shapeBuffer[i].vb.resize(buffersize); //pos(3 float) normal(3 float), tex(2 float), color(3 float)

		//convert buffer
		updateShapeArrayBuffer(i, shapeBuffer[i].sizePerVertex);
		
		shapeBuffer[i].arrayBuf = new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
		shapeBuffer[i].arrayBuf->create();

		shapeBuffer[i].arrayBuf->bind();
		shapeBuffer[i].arrayBuf->allocate(shapeBuffer[i].vb.data(), shapeBuffer[i].vb.size()*sizeof(float));
		
		shapeBuffer[i].material_id = (*shapes)[i].mesh.material_ids[0];
	}

}

void LoboTriMeshRender::updateShapeArrayBuffer(int i, int sizePerVertex)
{

	float* vertexcolor = trimesh->getVertexColorData();
	for (int j = 0; j < shapes->at(i).mesh.indices.size(); j++)
	{
		int vid = shapes->at(i).mesh.indices[j].vertex_index;

		shapeBuffer[i].vb[j*sizePerVertex + 0] = tri_attributes->vertices[vid * 3 + 0];
		shapeBuffer[i].vb[j*sizePerVertex + 1] = tri_attributes->vertices[vid * 3 + 1];
		shapeBuffer[i].vb[j*sizePerVertex + 2] = tri_attributes->vertices[vid * 3 + 2];

		shapeBuffer[i].vb[j*sizePerVertex + 3] = tri_attributes->normals[vid * 3 + 0];
		shapeBuffer[i].vb[j*sizePerVertex + 4] = tri_attributes->normals[vid * 3 + 1];
		shapeBuffer[i].vb[j*sizePerVertex + 5] = tri_attributes->normals[vid * 3 + 2];

		int tid = shapes->at(i).mesh.indices[j].texcoord_index;

		if (tid == -1)
		{
			shapeBuffer[i].vb[j*sizePerVertex + 6] = 0;
			shapeBuffer[i].vb[j*sizePerVertex + 7] = 0;
			//continue;
		}
		else
		{
			shapeBuffer[i].vb[j*sizePerVertex + 6] = tri_attributes->texcoords[tid * 2 + 0];
			shapeBuffer[i].vb[j*sizePerVertex + 7] = tri_attributes->texcoords[tid * 2 + 1];
		}

		shapeBuffer[i].vb[j*sizePerVertex + 8] = vertexcolor[vid * 3 + 0];
		shapeBuffer[i].vb[j*sizePerVertex + 9] = vertexcolor[vid * 3 + 1];
		shapeBuffer[i].vb[j*sizePerVertex + 10] = vertexcolor[vid * 3 + 2];

	}
}

void LoboTriMeshRender::initTexture()
{
	int numShapes = trimesh->getNumShapes();
	for (int i = 0; i < numShapes; i++)
	{
		int materialid = (shapeBuffer)[i].material_id;

		if (materialid == -1)
		{
			shapeBuffer[i].bool_texture = false;
			setUseTexutre(false);
			continue;
		}

		if ((*materials)[materialid].diffuse_texname != "")
		{
			QString str = "texture/";
			str.append((*materials)[materialid].diffuse_texname.c_str());
			std::cout << str.toStdString() << std::endl;

			shapeBuffer[i].texture = new QOpenGLTexture(QImage(str).mirrored());
			shapeBuffer[i].texture->setMaximumAnisotropy(8);

			shapeBuffer[i].texture->setMinificationFilter(QOpenGLTexture::LinearMipMapLinear);
			shapeBuffer[i].texture->setMagnificationFilter(QOpenGLTexture::Linear);
			shapeBuffer[i].texture->setWrapMode(QOpenGLTexture::Repeat);

			shapeBuffer[i].bool_texture = true;
		}
		else
		{
			shapeBuffer[i].bool_texture = false;
		} 
	}
}

void LoboTriMeshRender::initRender()
{
	alphaChannel = 1.0;
	useTexutre = true;
	acceptShadow = true;
	setVisible(true);
	ifrenderCollision = false;
	setRenderVertexColor(false);
	setSmoothshading(false);
	setIsTransparent(false);

	collisiondetection = NULL;

	if (!getSpecial_mesh())
	sphere1 = new SphereRender("3dmodel/spherelow.obj");
}
