#include "Ray.h"

// Set ray from a point and a direction vector.
void Ray::set(const Vec3& p, const Vec3& v)
{
	this->direction = v;
	this->point = p;
}

// Print ray.
void Ray::printSelf()
{
	cout << "Ray: " << this->point << " + " << this->direction << "t" << endl;
}

// Distance from a point to a ray.
float Ray::getDistance(const Vec3& p) const
{
	/* Distance from a point A to a ray(d) = | [MA, u]|/|u |
	which u is direction vector of ray (d), M is a point on ray (d) */
	return ((p - this->point).cross(this->direction)).length() / this->direction.length();
}

// Check if ray intersects with ray.
bool Ray::isIntersected(const Ray& ray) const
{
	// If 2 rays is parallel, [u1, u2] = 0
	Vec3 v = this->direction.cross(ray.getDirection());
	if (v == 0)
		return false; // (coincide, cross, or parallel)
	else
		return true; // intersect
}

// Distance between two rays (no intersect).
float Ray::getDistance(const Ray& r) const
{
	/* d = | [u, v].MM'| / |[u , v]|
	u, v are direction vectors of two rays, respectively. M and M' are point on two rays, respectively. */
	if (isIntersected(r)) return 0.0f;
	else
	{
		Vec3 u_u = this->direction.cross(r.getDirection());
		Vec3 MM = r.getPoint() - this->point;
		float d = (u_u * MM) / u_u.length();
		return d;
	}
}

// Check if a point is belong to a ray.
bool Ray::isBelongTo(const Vec3& p) const
{
	if (this->getDistance(p) == 0) return true;
	else return false;
}

// Return a point from ray with t
Vec3 Ray::currentPosition(const float t) const
{
	return point + direction * t;
}

// Ray intersects with ray (return a point).
Vec3 Ray::intersect(const Ray& ray) const
{
	const Vec3 v2 = ray.getDirection();
	const Vec3 p2 = ray.getPoint();
	Vec3 result = Vec3(NAN, NAN, NAN);

	if (isIntersected(ray)) {

	    // Find a = [(p2 - p1), v2]
		Vec3 a = (p2 - point).cross(v2);

		// Find b = [V1, V2] 
		Vec3 b = direction.cross(v2);

		// find t = a / b
		float alpha = 0;
		if (b.x != 0) alpha = a.x / b.x;
		else if (b.y != 0) alpha = a.y / b.y;
		else if (b.z != 0) alpha = a.z / b.z;
		else return result;

		// find intersect point
		result = currentPosition(alpha);
		return result;
	}
	else return result;
}

// Return a point's projection on a ray.
Vec3 Ray::projectPoint(const Vec3& p) const
{
	// Construct a plane (P) has point A and perpendicular with ray (d) (which has plane's normal vector is ray's direction vector)
	Ray d = Ray(this->getPoint(), this->getDirection());
	Plane P = Plane(this->direction, p);

	// The projection point is the intersection between ray (d) and plane (P) 
	return P.intersect(d);
}

// Construct ray from 2 points
Ray Ray::fromTwoPoint(const Vec3& p1, const Vec3& p2)
{
	Ray d;
	d.direction = p2 - p1;
	d.point = p1;
	return d;
}