#include "CubeNodeMap.h"
#define M_PI 3.14159265358979323846264338327950288

CubeNodeMap::CubeNodeMap()
{
}


CubeNodeMap::~CubeNodeMap()
{
}

CubeNodeMapType::CubeNodeMapType(std::vector<int> neighbors_)
{
	R_align_.setIdentity();
	reorderNeighbor.resize(6);
	for (int i = 0; i < 6; i++)
	{
		reorderNeighbor[i] = i;
	}

	this->neighbors = neighbors_;
	int count = 0;
	for (int i = 0; i < neighbors.size(); i++)
	{
		if (neighbors[i] != -1)
		{
			count++;
		}
	}

	mainType = count - 3;
	R_.setIdentity();
}

CubeNodeMapType::~CubeNodeMapType()
{

}

CubeNodeThreeNeighborType::CubeNodeThreeNeighborType(std::vector<int> neighbors) :CubeNodeMapType(neighbors)
{
	
}

void CubeNodeThreeNeighborType::computeSubTypeInfo()
{
	if (neighbors[0] != -1 && neighbors[3] != -1 && neighbors[4] != -1)
	{
		setSubtype(0);
		R_.setIdentity();
	}

	if (neighbors[1] != -1 && neighbors[3] != -1 && neighbors[4] != -1)
	{
		setSubtype(1);
		R_ = AngleAxisd(-0.5*M_PI, Vector3d::UnitY());
	}

	if (neighbors[1] != -1 && neighbors[2] != -1 && neighbors[4] != -1)
	{
		setSubtype(2);
		R_ = AngleAxisd(-M_PI, Vector3d::UnitY());
	}

	if (neighbors[0] != -1 && neighbors[2] != -1 && neighbors[4] != -1)
	{
		setSubtype(3);
		R_ = AngleAxisd(-1.5*M_PI, Vector3d::UnitY());
	}

	Matrix3d R_temp;
	R_temp = AngleAxisd(-0.5*M_PI, Vector3d::UnitY())
		*AngleAxisd(M_PI, Vector3d::UnitZ());

	if (neighbors[0] != -1 && neighbors[3] != -1 && neighbors[5] != -1)
	{
		setSubtype(4);
		R_ = R_temp;
	}

	if (neighbors[1] != -1 && neighbors[3] != -1 && neighbors[5] != -1)
	{
		setSubtype(5);
		R_ = AngleAxisd(-0.5*M_PI, Vector3d::UnitY())*R_temp;
	}

	if (neighbors[1] != -1 && neighbors[2] != -1 && neighbors[5] != -1)
	{
		setSubtype(6);
		R_ = AngleAxisd(-M_PI, Vector3d::UnitY())*R_temp;
	}

	if (neighbors[0] != -1 && neighbors[2] != -1 && neighbors[5] != -1)
	{
		setSubtype(7);
		R_ = AngleAxisd(-1.5*M_PI, Vector3d::UnitY())*R_temp;
	}
	computeNeighborOrderMap();

}

void CubeNodeThreeNeighborType::computeNeighborOrderMap()
{
	if (getSubtype() == 0)
	{
		//do nothing
	}

	if (getSubtype() == 1)
	{
		reorderNeighbor[1] = 3;
		reorderNeighbor[3] = 0;
		reorderNeighbor[4] = 4;
	}

	if (getSubtype() == 2)
	{
		reorderNeighbor[1] = 0;
		reorderNeighbor[2] = 3;
		reorderNeighbor[4] = 4;
	}

	if (getSubtype() == 3)
	{
		reorderNeighbor[0] = 3;
		reorderNeighbor[2] = 0;
		reorderNeighbor[4] = 4;
	}

	if (getSubtype() == 4)
	{
		reorderNeighbor[0] = 3;
		reorderNeighbor[3] = 0;
		reorderNeighbor[5] = 4;
	}

	if (getSubtype() == 5)
	{
		reorderNeighbor[1] = 0;
		reorderNeighbor[3] = 3;
		reorderNeighbor[5] = 4;
	}

	if (getSubtype() == 6)
	{
		reorderNeighbor[1] = 3;
		reorderNeighbor[2] = 0;
		reorderNeighbor[5] = 4;
	}

	if (getSubtype() == 7)
	{
		reorderNeighbor[0] = 0;
		reorderNeighbor[2] = 3;
		reorderNeighbor[5] = 4;
	}
}

CubeNodeFourNeighborType::CubeNodeFourNeighborType(std::vector<int> neighbors) :CubeNodeMapType(neighbors)
{
	
}

CubeNodeFourNeighborType::~CubeNodeFourNeighborType()
{
	
}

void CubeNodeFourNeighborType::computeSubTypeInfo()
{
	Matrix3d R_temp;
	if (neighbors[2] != - 1 && neighbors[3] != -1)
	{
		R_temp = Matrix3d::Identity();
		directionType = 0;

		if (neighbors[1] != -1 && neighbors[5] != -1)
		{
			setSubtype(2);
		}

		if (neighbors[1] != -1 && neighbors[4] != -1)
		{
			setSubtype(3);
		}

		if (neighbors[0] != -1 && neighbors[4] != -1)
		{
			setSubtype(0);
		}

		if (neighbors[0] != -1 && neighbors[5] != -1)
		{
			setSubtype(1);
		}
	}

	if (neighbors[4] != -1 && neighbors[5] != -1)
	{
		R_temp = AngleAxisd(-0.5*M_PI, Vector3d::UnitZ());
		directionType = 1;

		if (neighbors[1] != -1 && neighbors[3] != -1)
		{
			setSubtype(2);
		}

		if (neighbors[1] != -1 && neighbors[2] != -1)
		{
			setSubtype(3);
		}

		if (neighbors[0] != -1 && neighbors[2] != -1)
		{
			setSubtype(0);
		}

		if (neighbors[0] != -1 && neighbors[3] != -1)
		{
			setSubtype(1);
		}
	}

	if (neighbors[0] != -1 && neighbors[1] != -1)
	{
		R_temp = AngleAxisd(0.5*M_PI, Vector3d::UnitY());
		directionType = 2;

		if (neighbors[3] != -1 && neighbors[5] != -1)
		{
			setSubtype(2);
		}

		if (neighbors[3] != -1 && neighbors[4] !=-1)
		{
			setSubtype(3);
		}

		if (neighbors[2] != -1 && neighbors[4] != -1)
		{
			setSubtype(0);
		}

		if (neighbors[2] != -1 && neighbors[5] != -1)
		{
			setSubtype(1);
		}
	}


	if (getSubtype() == 0)
	{
		R_ = R_temp;
	}

	if (getSubtype() == 1)
	{
		R_ = AngleAxisd(-0.5*M_PI, Vector3d::UnitX())*R_temp;
	}

	if (getSubtype() == 2)
	{
		R_ = AngleAxisd(-1.0*M_PI, Vector3d::UnitX())*R_temp;
	}

	if (getSubtype() == 3)
	{
		R_ = AngleAxisd(-1.5*M_PI, Vector3d::UnitX())*R_temp;
	}
	computeNeighborOrderMap();

}

void CubeNodeFourNeighborType::computeNeighborOrderMap()
{
	if (directionType == 0)
	{
		if (getSubtype() == 0)
		{
			// do nothing

		}

		if (getSubtype() == 1)
		{
			reorderNeighbor[0] = 4;
			reorderNeighbor[2] = 2;
			reorderNeighbor[3] = 3;
			reorderNeighbor[5] = 0;
		}

		if (getSubtype() == 2)
		{
			reorderNeighbor[1] = 0;
			reorderNeighbor[2] = 2;
			reorderNeighbor[3] = 3;
			reorderNeighbor[5] = 4;
		}

		if (getSubtype() == 3)
		{
			reorderNeighbor[1] = 4;
			reorderNeighbor[2] = 2;
			reorderNeighbor[3] = 3;
			reorderNeighbor[4] = 0;
			
		}
	}

	if (directionType == 1)
	{
		if (getSubtype() == 0)
		{
			reorderNeighbor[0] = 0;
			reorderNeighbor[2] = 4;
			reorderNeighbor[4] = 3;
			reorderNeighbor[5] = 2;
		}

		if (getSubtype() == 1)
		{
			reorderNeighbor[0] = 4;
			reorderNeighbor[3] = 0;
			reorderNeighbor[4] = 3;
			reorderNeighbor[5] = 2;
		}

		if (getSubtype() == 2)
		{
			reorderNeighbor[1] = 0;
			reorderNeighbor[3] = 4;
			reorderNeighbor[4] = 3;
			reorderNeighbor[5] = 2;
		}

		if (getSubtype() == 3)
		{
			reorderNeighbor[1] = 4;
			reorderNeighbor[2] = 0;
			reorderNeighbor[4] = 3;
			reorderNeighbor[5] = 2;
		}
	}

	if (directionType == 2)
	{
		if (getSubtype() == 0)
		{
			reorderNeighbor[0] = 3;
			reorderNeighbor[1] = 2;
			reorderNeighbor[2] = 0;
			reorderNeighbor[4] = 4;
		}

		if (getSubtype() == 1)
		{
			reorderNeighbor[0] = 3;
			reorderNeighbor[1] = 2;
			reorderNeighbor[2] = 4;
			reorderNeighbor[5] = 0;
		}

		if (getSubtype() == 2)
		{
			reorderNeighbor[0] = 3;
			reorderNeighbor[1] = 2;
			reorderNeighbor[3] = 0;
			reorderNeighbor[5] = 4;
		}

		if (getSubtype() == 3)
		{
			reorderNeighbor[0] = 3;
			reorderNeighbor[1] = 2;
			reorderNeighbor[3] = 4;
			reorderNeighbor[4] = 0;
		}
	}
}

CubeNodeFiveNeighborType::CubeNodeFiveNeighborType(std::vector<int> neighbors) :CubeNodeMapType(neighbors)
{
	
}

void CubeNodeFiveNeighborType::computeSubTypeInfo()
{
	Matrix3d R_temp;
	if (neighbors[5] == -1)
	{
		setSubtype(0);
		R_.setIdentity();
	}

	if (neighbors[4] == -1)
	{
		setSubtype(1);

		R_ = AngleAxisd(-M_PI, Vector3d::UnitZ());
	}

	if (neighbors[2] == -1)
	{
		setSubtype(2);
		R_ = AngleAxisd(0.5*M_PI, Vector3d::UnitZ());
	}

	if (neighbors[3] == -1)
	{
		setSubtype(3);
		R_ = AngleAxisd(-0.5*M_PI, Vector3d::UnitZ());
	}

	if (neighbors[0] == -1)
	{
		setSubtype(4);
		R_ = AngleAxisd(0.5*M_PI, Vector3d::UnitX());

	}

	if (neighbors[1] == -1)
	{
		setSubtype(5);
		R_ = AngleAxisd(-0.5*M_PI, Vector3d::UnitX());
	}
	computeNeighborOrderMap();
}

void CubeNodeFiveNeighborType::computeNeighborOrderMap()
{
	if (getSubtype() == 0)
	{

	}

	if (getSubtype() == 1)
	{
		reorderNeighbor[0] = 0;
		reorderNeighbor[1] = 1;
		reorderNeighbor[2] = 3;
		reorderNeighbor[3] = 2;
		reorderNeighbor[5] = 4;
	}

	if (getSubtype() == 2)
	{
		reorderNeighbor[0] = 0;
		reorderNeighbor[1] = 1;
		reorderNeighbor[3] = 4;
		reorderNeighbor[4] = 2;
		reorderNeighbor[5] = 3;
	}

	if (getSubtype() == 3)
	{
		reorderNeighbor[0] = 0;
		reorderNeighbor[1] = 1;
		reorderNeighbor[2] = 4;
		reorderNeighbor[4] = 3;
		reorderNeighbor[5] = 2;
	}

	if (getSubtype() == 4)
	{
		reorderNeighbor[1] = 4;
		reorderNeighbor[2] = 2;
		reorderNeighbor[3] = 3;
		reorderNeighbor[4] = 0;
		reorderNeighbor[5] = 1;
	}

	if (getSubtype() == 5)
	{
		reorderNeighbor[0] = 4;
		reorderNeighbor[2] = 2;
		reorderNeighbor[3] = 3;
		reorderNeighbor[4] = 1;
		reorderNeighbor[5] = 0;
	}
}

CubeNodeALLNeighborType::CubeNodeALLNeighborType(std::vector<int> neighbors) :CubeNodeMapType(neighbors)
{

}

CubeNodeALLNeighborType::~CubeNodeALLNeighborType()
{

}

void CubeNodeALLNeighborType::computeSubTypeInfo()
{

}

void CubeNodeALLNeighborType::computeNeighborOrderMap()
{

}
