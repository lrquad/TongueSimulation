#pragma once
#include <Eigen/Dense>
#include "ObjLoader/tiny_obj_loader.h"
using namespace tinyobj;
using namespace Eigen;

void copyTinyAttribute(tinyobj::attrib_t &inputAttribute, tinyobj::attrib_t &outputAttribute);
void scaleTinyAttribute(tinyobj::attrib_t &inputAttribute, tinyobj::attrib_t &outputAttribute, double scale);
void centerTinyAttribute(tinyobj::attrib_t &inputAttribute, tinyobj::attrib_t &outputAttribute, Vector3d &originalToCenter);
void translateTinyAttribute(tinyobj::attrib_t &inputAttribute, tinyobj::attrib_t &outputAttribute, Vector3d translate);

void rotateTinyAttribute(tinyobj::attrib_t &inputAttribute, tinyobj::attrib_t &outputAttribute, Matrix3d rotation);

void transformTinyAttribute(tinyobj::attrib_t &inputAttribute, tinyobj::attrib_t &outputAttribute, Matrix3d Transform,Vector3d translate);

void updateSmoothNorm(tinyobj::attrib_t &inputAttribute, std::vector<shape_t> &meshShape);

