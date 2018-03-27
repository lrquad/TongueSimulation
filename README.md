# TongueSimulation
## Getting Started
```
std::ifstream readconfig("./config/tongue");
HyperelasticTongueSimulator* simulator = new HyperelasticTongueSimulator(readconfig,obj_mesh,true);
simulator->initSimulator();

//simulation loop start
while(true)
{
  simulator->updateSimulator(); 
}
```

### Prerequisites
Boost 1.61.0 library 
Your own render library

### Generate Subspace Modes
```
  //volumetric_mesh is a preload TetVolumetricMesh
  //massMatrix
  int R = volumetric_mesh->getNumVertices() * 3;
  SparseMatrix<double>* massMatrix = new SparseMatrix<double>(R, R);
	volumetric_mesh->computeMassMatrix(massMatrix);
  stvkmodel = new STVKModel(volumetric_mesh, massMatrix);
	stvkmodel->setGravity(false);
	stvkforce_model = new STVKForceModel(stvkmodel);

  //compute 20 modes
	modesCreater = new SubspaceModesCreator(stvkforce_model, massMatrix, numConstrainedDOFs, constrainedDOFs, 10, 20);
  
  modesCreater->generateModes();
	Eigen::MatrixXd reduced_modes = *(modesCreater->getFinalModes());
  
```


