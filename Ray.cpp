#include "Ray.h"

/*Ray::Ray(const Vec3& p1, const Vec3& p2)
{
	set2P(p1, p2);
}*/

void Ray::set(const Vec3& p, const Vec3 v)
{
	this->direction = v;
	this->point = p;
}

/*void Ray::set2P(const Vec3& p1, const Vec3& p2)
{
	this->direction = p2 - p1;
	this->point = p1;

}*/

// Khoảng cách từ điểm đến đường thằng
float Ray::getDistance(const Vec3& p) const
{
	// Khoảng cách từ điểm A đến đường thẳng d = |[MA , u]|/|u| , u là VTCP, M là điểm thuộc đường thẳng, A là điểm bất kì
	return ((p - this->point).cross(this->direction)).length() / this->direction.length();

}

// khoảng cách giữa hai đường thẳng chéo nhau 
float Ray::getDistance(const Ray& r) const
{
	// d = |[u , v].MM'| / |[u , v]| u, v lần lượt là VTCP của 2 đường thẳng; M, M' lllà điểm thuộc 2 đường thẳng
	if (isIntersected(r)) return 0.0f;
	else
	{
		Vec3 u_u = this->direction.cross(r.getDirection());
		Vec3 MM = r.getPoint() - this->point;
		float d = (u_u * MM) / u_u.length();
		return d;
	}
}

// In đường thẳng
void Ray::printSelf()
{
	cout << "Ray: " << this->point << " + " << this->direction << "t" << endl;
}

/*Where are we along the Ray?
Calculate posotion = start + dir*t */
Vec3 Ray::currentPosition(const float t) const
{
	return point + direction * t;
}

// Đường thẳng giao đường thẳng
Vec3 Ray::intersect(const Ray& ray) const
{
	const Vec3 v2 = ray.getDirection();
	const Vec3 p2 = ray.getPoint();
	Vec3 result = Vec3(NAN, NAN, NAN);    // default with NaN

	// find v3 = (p2 - p1) x V2
	Vec3 v3 = (p2 - point).cross(v2);

	// find v4 = V1 x V2
	Vec3 v4 = direction.cross(v2);

	// if both V1 and V2 are same direction, return NaN point
	if (v4.x == 0 && v4.y == 0 && v4.z == 0)
		return result;

	// find a = (p2-p1)xV2 / (V1xV2)
	float alpha = 0;
	if (v4.x != 0) alpha = v3.x / v4.x;
	else if (v4.y != 0) alpha = v3.y / v4.y;
	else if (v4.z != 0) alpha = v3.z / v4.z;
	else return result;

	// find intersect point
	result = currentPosition(alpha);
	return result;
}

// Vị trí tương đối của 2 đường thẳng
bool Ray::isIntersected(const Ray& ray) const
{
	// nếu 2 đường thẳng song song, [u1 , u2] = 0
	Vec3 v = this->direction.cross(ray.getDirection());
	if (v == 0)
		return false; // (trùng nhau, chéo nhau, song song)
	else
		return true; // cắt nhau
}

// Tìm hình chiếu của điểm lên đường thẳng
Vec3 Ray::projectPoint(const Vec3& p) const
{
	//B1: lập mặt phẳng P chứa điểm A và vuông góc d (có VTPT n là VTCP u của đường thẳng
	//B2: H là giao điểm của d và P
	Ray d = Ray(this->getPoint(), this->getDirection());
	Plane P = Plane(this->direction, p);
	Vec3 H = P.intersect(d);
	return H;
}

