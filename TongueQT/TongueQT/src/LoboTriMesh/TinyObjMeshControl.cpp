#include "TinyObjMeshControl.h"
#include "Functions/computeTriangle.h"

void copyTinyAttribute(tinyobj::attrib_t &inputAttribute, tinyobj::attrib_t &outputAttribute)
{
	outputAttribute = inputAttribute;
}

void scaleTinyAttribute(tinyobj::attrib_t &inputAttribute, tinyobj::attrib_t &outputAttribute, double scale)
{
	for (int i = 0; i < inputAttribute.vertices.size(); i++)
	{
		outputAttribute.vertices[i] = inputAttribute.vertices[i] * scale;
	}
}

void centerTinyAttribute(tinyobj::attrib_t &inputAttribute, tinyobj::attrib_t &outputAttribute, Vector3d &originalToCenter)
{
	originalToCenter.setZero();
	int numVertex = inputAttribute.vertices.size() / 3;

	for (int i = 0; i < numVertex; i++)
	{
		originalToCenter.data()[0] += inputAttribute.vertices[i * 3 + 0];
		originalToCenter.data()[1] += inputAttribute.vertices[i * 3 + 1];
		originalToCenter.data()[2] += inputAttribute.vertices[i * 3 + 2];
	}

	originalToCenter /= numVertex;

	for (int i = 0; i < numVertex; i++)
	{
		outputAttribute.vertices[i * 3 + 0] = inputAttribute.vertices[i * 3 + 0] - originalToCenter.data()[0];
		outputAttribute.vertices[i * 3 + 1] = inputAttribute.vertices[i * 3 + 1] - originalToCenter.data()[1];
		outputAttribute.vertices[i * 3 + 2] = inputAttribute.vertices[i * 3 + 2] - originalToCenter.data()[2];
	}
	originalToCenter *= -1;
}

void translateTinyAttribute(tinyobj::attrib_t &inputAttribute, tinyobj::attrib_t &outputAttribute, Vector3d translate)
{
	int numVertex = inputAttribute.vertices.size() / 3;
	for (int i = 0; i < numVertex; i++)
	{
		outputAttribute.vertices[i * 3 + 0] = inputAttribute.vertices[i * 3 + 0] + translate.data()[0];
		outputAttribute.vertices[i * 3 + 1] = inputAttribute.vertices[i * 3 + 1] + translate.data()[1];
		outputAttribute.vertices[i * 3 + 2] = inputAttribute.vertices[i * 3 + 2] + translate.data()[2];
	}
}

void rotateTinyAttribute(tinyobj::attrib_t &inputAttribute, tinyobj::attrib_t &outputAttribute, Matrix3d rotation)
{
	int numVertex = inputAttribute.vertices.size() / 3;
	for (int i = 0; i < numVertex; i++)
	{
		Vector3d inputat;
		inputat.data()[0] = inputAttribute.vertices[i * 3 + 0];
		inputat.data()[1] = inputAttribute.vertices[i * 3 + 1];
		inputat.data()[2] = inputAttribute.vertices[i * 3 + 2];
		inputat = rotation*inputat;
		outputAttribute.vertices[i * 3 + 0] = inputat.data()[0];
		outputAttribute.vertices[i * 3 + 1] = inputat.data()[1];
		outputAttribute.vertices[i * 3 + 2] = inputat.data()[2];
	}
}

void transformTinyAttribute(tinyobj::attrib_t &inputAttribute, tinyobj::attrib_t &outputAttribute, Matrix3d Transform, Vector3d translate)
{
	int numVertex = inputAttribute.vertices.size() / 3;
	for (int i = 0; i < numVertex; i++)
	{
		Vector3d inputat;
		inputat.data()[0] = inputAttribute.vertices[i * 3 + 0];
		inputat.data()[1] = inputAttribute.vertices[i * 3 + 1];
		inputat.data()[2] = inputAttribute.vertices[i * 3 + 2];
		inputat = Transform*inputat + translate;
		outputAttribute.vertices[i * 3 + 0] = inputat.data()[0];
		outputAttribute.vertices[i * 3 + 1] = inputat.data()[1];
		outputAttribute.vertices[i * 3 + 2] = inputat.data()[2];
	}
}

void updateSmoothNorm(tinyobj::attrib_t &inputAttribute, std::vector<shape_t> &meshShape)
{
	std::fill(inputAttribute.normals.begin(), inputAttribute.normals.end(), 0);
	int numVertex = inputAttribute.vertices.size() / 3;
	std::vector<double> node_areaface(numVertex);
	std::fill(node_areaface.begin(), node_areaface.end(), 0);
	std::vector<Vector3d> node_norm(numVertex);
	for (int i = 0; i < numVertex; i++)
	{
		node_norm[i].setZero();
	}

	for (int i = 0; i < meshShape.size(); i++)
	{
		int numface = meshShape[i].mesh.indices.size() / 3;

		for (int f = 0; f < numface; f++)
		{
			int v[3];
			for (int x = 0; x < 3;x++)
			v[x] = meshShape[i].mesh.indices[f * 3 + x].vertex_index;

			Vector3d position[3];
			for (int n = 0; n < 3; n++)
			{
				position[n][0] = inputAttribute.vertices[v[n] * 3 + 0];
				position[n][1] = inputAttribute.vertices[v[n] * 3 + 1];
				position[n][2] = inputAttribute.vertices[v[n] * 3 + 2];
			}

			Vector3d normal;
			computeTriangleNorm(position[0], position[1], position[2], normal);
			double area = computeTriangleArea(position[0], position[1], position[2]);
			normal *= area;

			for (int n = 0; n < 3; n++)
			{
				int vertexid = v[n];
				node_norm[vertexid] += normal;
				node_areaface[vertexid] += area;
			}
		}
	}

	for (int j = 0; j < numVertex; j++)
	{
		node_norm[j] /= node_areaface[j];
		node_norm[j].normalize();
		inputAttribute.normals[j * 3 + 0] = node_norm[j].data()[0];
		inputAttribute.normals[j * 3 + 1] = node_norm[j].data()[1];
		inputAttribute.normals[j * 3 + 2] = node_norm[j].data()[2];
	}
	
}
