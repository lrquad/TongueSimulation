#pragma once
#include "LoboVolumetricMesh\LoboVolumetricMesh.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <iostream>
#include <queue>
#include <vector>
using namespace Eigen;

void computeBFSDistance(LoboVolumetricMesh* volumetricmesh, int seedindex, std::vector<double> &nodedistance);
void computeDijkstraDistance(LoboVolumetricMesh* volumetricmesh, int seedindex, std::vector<double> &nodedistance);
int computeBFSDistance(LoboVolumetricMesh* volumetricmesh, int seedindex, std::vector<double> &nodedistance, std::vector<int> subnodesindex);