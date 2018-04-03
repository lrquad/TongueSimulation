#pragma once

#include <Eigen/Dense>
using namespace Eigen;
#define MIN(x,y) x<y?x:y
#define MAX(x,y) x>y?x:y

class MouseProjection
{
public:
	MouseProjection();
	~MouseProjection();

	inline void setBindedNodeid(int nodeid){ selectedNodeIndex = nodeid; }
	inline int getBindedNodeid(){ return selectedNodeIndex; }

	inline int getMouseX(){ return mouseXY.x(); }
	inline int getMouseY(){ return mouseXY.y(); }

	inline int getPreMouseX(){ return preMouseXY.x(); }
	inline int getPreMouseY(){ return preMouseXY.y(); }

	inline void setMouseXY(double x, double y){ mouseXY.x() = x; mouseXY.y() = y;}
	inline void setPreMouseXY(double x, double y){ preMouseXY.x() = x; preMouseXY.y() = y;}

	inline void setWidth(int width){ this->width = width; };
	inline void setHeight(int height){ this->height = height; };

	virtual void projectWorldToScreen(Vector3d position, Vector2d &screenresult,double &depth);
	virtual void projectWorldToScreenDepth(Vector3d position, Vector2d &screenresult, double &depth);

	bool isInMouseRect(double &x, double &y);
	virtual void projectScreenToWorld(Vector2d screenpos, double depth, Vector3d &resultposition);
	
	inline void setMouseRect(double x1, double x2, double y1, double y2)
	{
		this->x1 = MIN(x1, x2);
		this->x2 = MAX(x1, x2);
		this->y1 = MIN(y1, y2);
		this->y2 = MAX(y1, y2);
	}

	inline void setMVPmatrix(Matrix4d mvp){ MVP = mvp; }

	Vector3d getLastClickPosition() const { return lastClickPosition; }
	void setLastClickPosition(Vector3d val) { lastClickPosition = val; }
protected:

	Matrix4d MVP;
	Vector2d mouseXY;
	Vector2d preMouseXY;

	double x1, x2;
	double y1, y2;

	int width;
	int height;

	int selectedNodeIndex;
	Vector3d lastClickPosition;
};

