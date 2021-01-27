#ifndef COLLIDERPHYSICS_H
#define COLLIDERPHYSICS_H

#include <iostream>
#include <math.h>
#include <algorithm>
#include "AABox.h"
#include "Sphere.h"
#include "Body.h"

using namespace std;

class ColliderPhysics
{
public:
	static bool SphereSphereCollisionDetected(const Body& body1, const Body& body2);
	static bool SpherePlaneCollisionDetected(const Body& body1, const Plane& plane);
	static void SphereSphereCollisionResponse(const Body& body1, const Body& body2, float c);
	static Vec3 SphereStaticSphereCollisionResponse(const Body& body1, const Body& staticsphere);
	static Vec3 SpherePlaneCollisionResponse(const Body& body1, const Plane& plane);
};


#endif