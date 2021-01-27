#include "AABox.h"


void AABox::printSelf()
{
	cout << "The AABox is described by six planes:\n"
		<< "x = 0\tx = " << rx << "\n"
		<< "y = 0\ty = " << ry << "\n"
		<< "z = 0\tz = " << rz << endl;
}

void AABox::set(float w, float h, float d)
{
	this->rx = w;
	this->ry = h;
	this->rz = d;
}
