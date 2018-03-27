#include "LoboTriMeshRenderDNN.h"
#include <iostream>
#include "LoboTriMesh/LoboTriMesh.h"
#include <vector>
#include "Simulator/collision/TriMeshCollisionDetector.h"
#include "Render/SphereRender.h"
#include "TinyObjMeshControl.h"
#include <QElapsedTimer>



LoboTriMeshRenderDNN::LoboTriMeshRenderDNN(LoboTriMesh* trimesh_) :LoboTriMeshRender(trimesh_)
{
	for (int i = 0; i < shapeBuffer.size(); i++)
	{
		shapeBuffer[i].arrayBuf->destroy();
		delete shapeBuffer[i].arrayBuf;
	}
	initShapeBuffer();
}

LoboTriMeshRenderDNN::~LoboTriMeshRenderDNN()
{

}

void LoboTriMeshRenderDNN::updateBuffer()
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

void LoboTriMeshRenderDNN::drawShape(QOpenGLShaderProgram *program, RenderBufferShape &shapebuffer, double alpha /*= 1.0*/)
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

	int alignRotationcol3 = program->attributeLocation("alignRotationcol3");
	glEnableVertexAttribArray(alignRotationcol3);
	glVertexAttribPointer(alignRotationcol3, 3, GL_FLOAT, GL_FALSE, shapebuffer.sizePerVertex * sizeof(float), (const void*)(sizeof(float) * 24));

	int alignRotationcol2 = program->attributeLocation("alignRotationcol2");
	glEnableVertexAttribArray(alignRotationcol2);
	glVertexAttribPointer(alignRotationcol2, 3, GL_FLOAT, GL_FALSE, shapebuffer.sizePerVertex * sizeof(float), (const void*)(sizeof(float) * 21));

	int alignRotationcol1 = program->attributeLocation("alignRotationcol1");
	glEnableVertexAttribArray(alignRotationcol1);
	glVertexAttribPointer(alignRotationcol1, 3, GL_FLOAT, GL_FALSE, shapebuffer.sizePerVertex * sizeof(float), (const void*)(sizeof(float) * 18));

	int DNN2 = program->attributeLocation("DNN2");
	glEnableVertexAttribArray(DNN2);
	glVertexAttribPointer(DNN2, 3, GL_FLOAT, GL_FALSE, shapebuffer.sizePerVertex * sizeof(float), (const void*)(sizeof(float) * 15));

	int DNN1 = program->attributeLocation("DNN1");
	glEnableVertexAttribArray(DNN1);
	glVertexAttribPointer(DNN1, 4, GL_FLOAT, GL_FALSE, shapebuffer.sizePerVertex * sizeof(float), (const void*)(sizeof(float) * 11));

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

	glDisableVertexAttribArray(alignRotationcol3);
	glDisableVertexAttribArray(alignRotationcol2);
	glDisableVertexAttribArray(alignRotationcol1);

	glDisableVertexAttribArray(DNN2);
	glDisableVertexAttribArray(DNN1);
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

void LoboTriMeshRenderDNN::initShapeBuffer()
{
	int numShapes = trimesh->getNumShapes();

	shapeBuffer.resize(numShapes);
	//init buffer
	for (int i = 0; i < numShapes; i++)
	{
		shapeBuffer[i].sizePerVertex = 11+16;
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

void LoboTriMeshRenderDNN::updateShapeArrayBuffer(int i, int sizePerVertex)
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

		if (trimesh->vertexData.rows() !=0)
		{
			for (int k = 0; k < 16; k++)
			{
				shapeBuffer[i].vb[j*sizePerVertex + 11 + k] = trimesh->vertexData.data()[vid * 16 + k];
			}
		}
		else
		{
			for (int k = 0; k < 16; k++)
			{
				shapeBuffer[i].vb[j*sizePerVertex + 11 + k] = 0;
			}
		}
	}
}
