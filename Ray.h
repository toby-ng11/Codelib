#ifndef RAY_H
#define RAY_H

#include <iostream>
#include <math.h>
#include "Vector.h"
#include "Plane.h"

using namespace std;

class Plane;

class Ray
{
	// Ray's parametric equation :            | x = x0 + at
	//                                        | y = y0 + bt
	//                                        | z = z0 + ct
	// Ray's canonical equation : (x - x0) / a = (y - y0) / b = (z - z0) / c
	//             _____________________________________________________
	// which :    |  Vec3(x0, y0, z0) is a point in the ray (p)         |
	//            |  Vec3(a, b, c) is direction vector of the ray (v)   |
	//            |_____________________________________________________|
	// We have p (point) = Vec3(x0, y0, z0), v (direction) = Vec3(a, b, c) we have this ray equation | d = p + vt  |
public:
	
	Ray() :point(Vec3(0, 0, 0)), direction(Vec3(0, 0, 0)) {}             // Ray definition
	Ray(const Vec3& p, const Vec3& v) : point(p), direction(v) {}        // Construct ray from a point and a direction vector
	~Ray() {}                                                            // Deconstructor

	void printSelf();                                                    // Print ray

	void set(const Vec3&, const Vec3&);                              // Set ray from a point and a direction vector

	void setPoint(Vec3& p) { point = p; }                                // Set ray's point
	void setDirection(const Vec3& v) { direction = v; }                  // Set ray's direction vector

	const Vec3& getPoint() const { return point; }                       // Get ray's point
	const Vec3& getDirection() const { return direction; }               // Get ray's direction vector

	bool isIntersected(const Ray&) const;                            // Check if ray intersects with ray

	float getDistance(const Vec3&) const;                              // Distance from a point to a ray
	float getDistance(const Ray&) const;                               // Distance between two rays (no intersect)

	bool isBelongTo(const Vec3&) const;                                // Check if a point is belong to a ray

	Vec3 currentPosition(const float) const;                           // Return a point of ray with t
	Vec3 intersect(const Ray&) const;                                // Ray intersects with ray (return a point)
	Vec3 projectPoint(const Vec3&) const;                              // Return a point's projection on a ray

	Ray fromTwoPoint(const Vec3&, const Vec3&);                    // Construct ray from 2 points
private:
	Vec3 point;                                                          // Starting position of ray
	Vec3 direction;                                                      // Direction of ray


};
#endif
