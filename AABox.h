#ifndef AABOX_H
#define AABOX_H

#include <iostream>
#include <math.h>
#include "Plane.h"

using namespace std;

class AABox
{
public:

	// A box can be described as 6 planes
	AABox() :rx(1), ry(1), rz(1)
	{
		box[0] = Plane(1, 0, 0, 0); box[0] = Plane(1, 0, 0, -rx);
		box[1] = Plane(0, 1, 0, 0); box[0] = Plane(0, 1, 0, -ry);
		box[2] = Plane(0, 0, 1, 0); box[0] = Plane(0, 0, 1, -rz);
	}
	AABox(float width, float height, float depth) :rx(width), ry(height), rz(depth) {}

	void printSelf();

	void set(float w, float h, float d);
	float getRx() const { return rx; }
	float getRy() const { return ry; }
	float getRz() const { return rz; }

private:
	float rx, ry, rz; // Width, height, and depth of box
	Plane box[6];

};
#endif

