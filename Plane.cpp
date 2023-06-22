#include "Plane.h"

// Plane equation: ax + by + cz + d = 0.
Plane::Plane(float x_, float y_, float z_, float d_)
{
	set(x_, y_, z_, d_);
}

// Construct plane from 3 points.
Plane::Plane(const Vec3& p1, const Vec3& p2, const Vec3& p3)
{
	set(p1, p2, p3);
}

// Construct plane from plane's normal and a point.
Plane::Plane(const Vec3& normal, const Vec3& point)
{
	set(normal, point);
}

// Construct plane as a Vec4.
Plane::Plane(const Vec4& v)
{
	normal.set(v.x, v.y, v.z);
	this->d = v.w;
	normalLength = sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
	distance = -d / normalLength;
}

// Print plane.
void Plane::printSelf() const
{
	cout << "Plane(" << normal.x << ", " << normal.y << ", " << normal.z
		<< ", " << d << ")" << endl;
}

// Set plane as a Vec4.
void Plane::set(float a, float b, float c, float d)
{
	normal.set(a, b, c);
	this->d = d;

	// compute distance
	normalLength = sqrtf(a * a + b * b + c * c);
	distance = -d / normalLength;
}

// Set plane from plane's normal vec and a point in the plane.
void Plane::set(const Vec3& normal, const Vec3& point)
{
	this->normal = normal;
	normalLength = normal.length();
	d = -normal.dot(point);         // -(a*x0 + b*y0 + c*z0)
	distance = -d / normalLength;
}

// Set plane from 3 points.
void Plane::set(const Vec3& p1, const Vec3& p2, const Vec3& p3)
{
	Vec3 a = p2 - p1;
	Vec3 b = p3 - p1;
	this->normal = a.cross(b);
	normalLength = normal.length();
	this->d = -normal.dot(p1);
	distance = -d / normalLength;
}

// Get the distance from plane to a point.
float Plane::getDistance(const Vec3& point) const
{   
	// M is a point, Alpha is a plane, H is the projection of M onto the plane Alpha.
	// d(M , Alpha) = HM = |A*x0 + B*y0 + C*z0 + D|/sqrt(A^2+B^2+B^2) = | dot product(plane's normal , point) + D|/length(plane's normal)
	//     M(x0, y0, z0), Alpha(A, B, C, D)

	float dot = normal.dot(point);
	return (dot + d) / normalLength;
}

// Get the angle between plane and a ray.
float Plane::getAngle(const Ray& ray) const
{   
	// u is ray direction, n is plane's normal
	// sin a = | u . n | / |u|.|n|

	Vec3 u = ray.getDirection();
	Vec3 n = this->normal;
	float sinAngle = (u * n) / (u.length() * n.length());
	return asinf(sinAngle);
}

// Normalize plane.
void Plane::normalize()
{
	float lengthInv = 1.0f / normalLength;
	normal *= lengthInv;
	normalLength = 1.0f;
	d *= lengthInv;
	distance = -d;
}

// Check if plane intersect with ray.
bool Plane::isIntersected(const Ray& ray) const
{
	// direction vector of line
	Vec3 v = ray.getDirection();

	// dot product with normal of the plane
	float dot = normal.dot(v);  // a*Vx + b*Vy + c*Vz

	if (dot == 0) return false;
	else return true;
}

// Plane intersects ray (return a point).
Vec3 Plane::intersect(const Ray& ray) const
{   
	// Ray d = p + vt, p is a point on ray, v is direction of ray

	if (isIntersected(ray))
	{
		Vec3 p = ray.getPoint();      // (x0, y0, z0)
		Vec3 v = ray.getDirection();  // (x, y, z)

		float dot1 = normal.dot(p);
		float dot2 = normal.dot(v);

		float t = -(dot1 + d) / dot2;

		return p + v * t;
	}
	
	else return Vec3(NAN, NAN, NAN);
}

// Check if plane intersect with plane.
bool Plane::isIntersected(const Plane& plane) const
{
	// check if 2 plane normals are same direction
	Vec3 cross = normal.cross(plane.getNormal());
	if (cross == 0) return false;
	else return true;
}

// Plane intersects plane (return a ray).
Ray Plane::intersect(const Plane& plane) const
{
	if (isIntersected(plane))
	{
		// find direction vector of the intersection line
		Vec3 v = normal.cross(plane.getNormal());

		// find a point on the line, which is also on both planes
	    // choose simple plane where d=0: ax + by + cz = 0
		float dot = v.dot(v);                       // V dot V
		Vec3 n1 = plane.getD() * normal;           // d2 * N1
		Vec3 n2 = -d * plane.getNormal();          //-d1 * N2
		Vec3 p = (n1 + n2).cross(v) / dot;       // (d2*N1-d1*N2) X V / V dot V

		return Ray(v, p);
	}
	else return Ray(Vec3(NAN, NAN, NAN), Vec3(NAN, NAN, NAN));
}

// Return a point's projection on a plane.
Vec3 Plane::projectPoint(const Vec3& p) const
{
	// Construct a ray d go through M has u == n, u is ray direction, n is plane's normal.
	Ray d = Ray(p, this->normal);

	// Projection is the intersect point between ray d and the plane.
    return this->intersect(d);
}


// Return a vector's projection on a plane.
Vec3 Plane::projectVec(const Vec3& v) const
{
	// Find projection of vector A on plane's normal vector.
	Vec3 n = this->normal;
	Vec3 A2 = n.project(v);

	// Result = vector A - projection vector.
	return v - A2;
}

// Return a ray's projection on a plane.
Ray Plane::projectRay(const Ray& ray) const
{
	// Find vector n1 = cross product(u (ray direction), n (this plane's normal)).
	Vec3 M = ray.getPoint();
	Vec3 u = ray.getDirection();
	Vec3 n = this->normal;

	// Construct a plane Q has point M (point on ray) and n1 as plane's normal.
	Vec3 n1 = u.cross(n);
	Plane Q = Plane(n1, M);

	// Projection is the intersect ray between plane Q and this plane. 
	return this->intersect(Q);
}
