#ifndef PLANE_H
#define PLANE_H

#include <math.h>
#include "Ray.h"

using namespace std;

class Ray;

class Plane
{
	// Plane equation : a(x - x0) + b(y - y0) + c(z - z0) = 0
	//             ___________________________________________________________
	// which :    | Vec3(a, b, c) is plane's normal vector                    |
	//            | Vec3(x0, y0, z0) is a point in the plane                  |
	//            |___________________________________________________________|
	// Plane equation : ax + by + cz + d = 0
	//             ______________________________________________________________________________________________________________________________
	// which :    | Vec3(a, b, c) is plane's normal vector                                                                                       |
	//            | d = - (a * x0 + b * y0 + c * z0) which Vec3(x0, y0, z0) is a point in the plane                                              | 
	//            | ( d = - normal.dot(point in the plane) (d là - (The dot (scalar) product of the normal vector to the point in the plane))    |   
	//            |______________________________________________________________________________________________________________________________|
	// We can assume a plane is a Vec4(a, b, c, d) 
public:

	Plane() :normal(Vec3(0, 0, 1)), d(0), normalLength(1), distance(0) {}
	Plane(float x_, float y_, float z_, float d_);           // Plane equation: ax + by + cz + d = 0
	Plane(const Vec3& p1, const Vec3& p2, const Vec3& p3);   // Construct plane from 3 points
	Plane(const Vec3& normal, const Vec3& point);            // Construct plane from plane's normal and a point
	Plane(const Vec4& v);                                    // Construct plane as a Vec4
	~Plane() {};

	// Check plane
	void printSelf() const;

	void set(float a, float b, float c, float d);            // Set plane as a Vec4
	void set(const Vec3& normal, const Vec3& point);         // Set plane from plane's normal vec and a point in the plane
	void set(const Vec3& p1, const Vec3& p2, const Vec3& p3);// Set plane from 3 points

	
	const Vec3& getNormal() const { return normal; }         // Get plane's normal vector
	float getD() const { return d; }                         // Get d value in plane equation
	float getNormalLength() const { return normalLength; }   // Get length of plane's normal vector
	float getDistance() const { return distance; }           // Get the distance from plane to the Origin

	float getDistance(const Vec3& point) const;              // Get the distance from plane to a point
	float getAngle(const Ray& ray) const;                    // Get the angle between plane and a ray
	
	void normalize();                                        // Normalize plane

	bool isIntersected(const Ray& ray) const;                // Check if plane intersects with ray
	Vec3 intersect(const Ray& ray) const;                    // Plane intersects ray (return a point)

	bool isIntersected(const Plane& plane) const;            // Check if plane intersects with plane
	Ray intersect(const Plane& plane) const;                 // Plane intersects plane (return a ray)

	Vec3 projectPoint(const Vec3& p) const;                  // Return a point's projection on a plane
	Vec3 projectVec(const Vec3& v) const;                    // Return a vector's projection on a plane
	Ray projectRay(const Ray& ray) const;                    // Return a ray's projection on a plane

private:
	Vec3 normal;                                             // Normal vector of the plane
	float d;                                                 // d = - (a * x0 + b * y0 + c * z0)
	float normalLength;                                      // Plane's normal vector's length
	float distance;                                          // Distance to the Origin
};
#endif
