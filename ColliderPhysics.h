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
	static Vec4 quadraticEquation(const Ray& ray, const Sphere& sphere);
	static bool RaySphereCollisionDetected(const Ray& ray, const Sphere& sphere);
	static void RaySphereCollisionPoint(const Ray& ray, const Sphere& sphere);
	static bool RayAABCollisionDetected(const Ray& ray, const AABox& box);
	static void RayAABCollisionPoint(const Ray& ray, const AABox& box);

	static bool SphereSphereCollisionDetected(const Body& body1, const Body& body2);
	static bool SpherePlaneCollisionDetected(const Body& body1, const Plane& plane);
	static void SphereSphereCollisionResponse(const Body& body1, const Body& body2, float c);
	static Vec3 SphereStaticSphereCollisionResponse(const Body& body1, const Body& staticsphere);
	static Vec3 SpherePlaneCollisionResponse(const Body& body1, const Plane& plane);
};


#endif