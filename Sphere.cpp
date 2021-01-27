#include "Sphere.h"

Sphere::Sphere(const Vec3& position_, float radius_)
{
	set(position_, radius_);
}

Sphere::Sphere(const Vec3& position, const Vec3& point)
{
	set(position, point);
}

/*Sphere::Sphere(const Vec3& p1, const Vec3& p2, const Vec3& p3, const Vec3& p4)
{
	set(p1, p2, p3, p4);
}*/

void Sphere::printSelf()
{
	cout << "The equation of the sphere: (x - " << position.x << ")^2 + (y -" << position.y << ")^2 + (z - " << position.z << ")^2 = " << radius * radius  << endl;
}

void Sphere::set(const Vec3& position, float radius)
{
	this->position = position;
	this->radius = radius;
	this->distance = sqrtf(position * position);
	
}

void Sphere::set(const Vec3& position, const Vec3& point)
{
	this->position = position;
	this->radius = sqrtf((point - position).length());
	this->distance = sqrtf(position * position);
}

//void Sphere::set(const Vec3& p1, const Vec3& p2, const Vec3& p3, const Vec3& p4)
//{
//}

// Lấy khoảng cách từ tâm mặt cầu dến một điểm
float Sphere::getDistance(const Vec3& point) const
{
	return sqrtf((position - point) * (position - point));
}

// Lấy khoảng cách từ tâm mặt cầu đến đường thẳng (hình chiếu của tâm)
float Sphere::getDistance(const Ray& ray) const
{
	return ray.getDistance(this->position);
}

// Lấy khoảng cách từ tâm mặt cầu dến mặt phẳng (hình chiếu của tâm)
float Sphere::getDistance(const Plane& plane) const
{
	return plane.getDistance(this->position);
}
