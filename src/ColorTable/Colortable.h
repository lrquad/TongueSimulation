#pragma once

static float colortable[48] =
{
	1, 0.2, 0,
	0.2, 1, 0,
	0, 0.2, 1,
	1, 0.2, 1,
	1, 1, 0.2,
	0.3, 0.9, 1,
	1, 0.5, 0.5,
	0.5, 1, 0.5,
	0.5, 0.5, 1,
	0.8, 0.8, 0.8,
	0.22,0.22,0.22,
	1,1,0.5,
	0.5,1,1,
	1,0.5,1,
	1,0.2,1,
	0.2,1,1
};

int getColor(int index, double* colors);

int getColorSize();

