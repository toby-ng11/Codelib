#ifndef COLLIDER_H
#define COLLIDER_H

#include <iostream>
#include <math.h>
#include <algorithm>
#include "AABox.h"
#include "Sphere.h"
#include "Body.h"

using namespace std;

class Collider
{
public:

	static Vec4 quadraticEquation(const Ray& ray, const Sphere& sphere);
	static bool RaySphereCollisionDetected(const Ray& ray, const Sphere& sphere);
	static void RaySphereCollisionPoint(const Ray& ray, const Sphere& sphere);
	static bool RayAABCollisionDetected(const Ray& ray, const AABox& box);
	static void RayAABCollisionPoint(const Ray& ray, const AABox& box);

};
#endif