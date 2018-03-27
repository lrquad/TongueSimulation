#pragma once
#include  "LoboTriMesh/LoboTriMeshRender.h"
class LoboTriMesh;


class LoboTriMeshRenderDNN: public LoboTriMeshRender
{
public:
	LoboTriMeshRenderDNN(LoboTriMesh* trimesh);
	~LoboTriMeshRenderDNN();

	virtual void updateBuffer();


protected:

	virtual void drawShape(QOpenGLShaderProgram *program, RenderBufferShape &shapebuffer, double alpha = 1.0);

	virtual void initShapeBuffer();

	virtual void updateShapeArrayBuffer(int i, int sizePerVertex);



};

