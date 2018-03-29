#include "Colortable.h"
int getColor(int index, double* colors)
{
	int tablesize = sizeof(colortable) / sizeof(float)/3;
	index %= tablesize;

	colors[0] = colortable[index * 3 + 0];
	colors[1] = colortable[index * 3 + 1];
	colors[2] = colortable[index * 3 + 2];
	return 0;
}

int getColorSize()
{
	int tablesize = sizeof(colortable) / sizeof(float) / 3;
	return tablesize;
}

