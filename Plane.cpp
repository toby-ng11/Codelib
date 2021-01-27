#include "Plane.h"


Plane::Plane(float x_, float y_, float z_, float d_)
{
	set(x_, y_, z_, d_);
}


// Lập mặt phẳng từ ba điểm
Plane::Plane(const Vec3& p1, const Vec3& p2, const Vec3& p3)
{
	set(p1, p2, p3);
}

Plane::Plane(const Vec3& normal, const Vec3& point)
{
	set(normal, point);
}

// Mặt phẳng là 1 vec4(a,b,c,d)
Plane::Plane(const Vec4& v)
{
	normal.set(v.x, v.y, v.z);
	this->d = v.w;
	normalLength = sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
	distance = -d / normalLength;
}

// In mặt phẳng
void Plane::printSelf() const
{
	cout << "Plane(" << normal.x << ", " << normal.y << ", " << normal.z
		<< ", " << d << ")" << endl;
}

// Lập mặt phẳng từ 4 tham số
void Plane::set(float a, float b, float c, float d)
{
	normal.set(a, b, c);
	this->d = d;

	// compute distance
	normalLength = sqrtf(a * a + b * b + c * c);
	distance = -d / normalLength;
}

// Lập mặt phẳng từ VTPT và 1 điểm trên mặt phẳng
void Plane::set(const Vec3& normal, const Vec3& point)
{
	this->normal = normal;
	normalLength = normal.length();
	d = -normal.dot(point);         // -(a*x0 + b*y0 + c*z0)
	distance = -d / normalLength;
}

// Lập mặt phẳng từ 3 điểm
void Plane::set(const Vec3& p1, const Vec3& p2, const Vec3& p3)
{
	Vec3 a = p2 - p1;
	Vec3 b = p3 - p1;
	this->normal = a.cross(b);
	normalLength = normal.length();
	this->d = -normal.dot(p1);
	distance = -d / normalLength;
}

// Khoảng cách từ 1 điểm tới mặt phẳng
float Plane::getDistance(const Vec3& point) const
{
	// d(M , alpha) = HM = |A*x0 + B*y0 + C*z0 + D|/sqrt(A^2+B^2+B^2) = | tích vô hướng (VTPT , điểm) + D|/độ dài VTPT
	// với H là hình chiếu của M lên mặt phẳng alpha,
	//     M(x0, y0, z0), alpha(A, B, C, D)
	float dot = normal.dot(point);
	return (dot + d) / normalLength;
}

// Góc giữa đường thẳng và mặt phẳng
float Plane::getAngle(const Ray& ray) const
{
	// sin a = | u . n | / |u|.|n|
	Vec3 u = ray.getDirection();
	Vec3 n = this->normal;
	float sinAngle = (u * n) / (u.length() * n.length());
	return asinf(sinAngle);
}

// Chuẩn hoá mặt phẳng
void Plane::normalize()
{
	float lengthInv = 1.0f / normalLength;
	normal *= lengthInv;
	normalLength = 1.0f;
	d *= lengthInv;
	distance = -d;
}


// Đường thẳng giao mặt phẳng
Vec3 Plane::intersect(const Ray& ray) const
{
	// Đường thẳng d có dạng d = P + Vt
	Vec3 p = ray.getPoint();      // (x0, y0, z0)
	Vec3 v = ray.getDirection();  // (x, y, z)

	float dot1 = normal.dot(p);
	float dot2 = normal.dot(v);

	if (dot2 == 0) return Vec3(NAN, NAN, NAN);

	float t = -(dot1 + d) / dot2;

	return p + v * t;
}

Ray Plane::intersect(const Plane& plane) const
{
	// find direction vector of the intersection line
	Vec3 v = normal.cross(plane.getNormal());

	// if |direction| = 0, 2 planes are parallel (no intersect)
	// return a line with NaN
	if (v == 0)
		return Ray(Vec3(NAN, NAN, NAN), Vec3(NAN, NAN, NAN));

	// find a point on the line, which is also on both planes
	// choose simple plane where d=0: ax + by + cz = 0
	float dot = v.dot(v);                       // V dot V
	Vec3 n1 = plane.getD() * normal;           // d2 * N1
	Vec3 n2 = -d * plane.getNormal();          //-d1 * N2
	Vec3 p = (n1 + n2).cross(v) / dot;       // (d2*N1-d1*N2) X V / V dot V

	return Ray(v, p);
}

bool Plane::isIntersected(const Ray& ray) const
{
	// direction vector of line
	Vec3 v = ray.getDirection();

	// dot product with normal of the plane
	float dot = normal.dot(v);  // a*Vx + b*Vy + c*Vz

	if (dot == 0) return false;
	else return true;
}

bool Plane::isIntersected(const Plane& plane) const
{
	// check if 2 plane normals are same direction
	Vec3 cross = normal.cross(plane.getNormal());
	if (cross.x == 0 && cross.y == 0 && cross.z == 0) return false;
	else return true;
}

// Tìm hình chiếu của điểm (M) lên mặt phẳng
Vec3 Plane::projectPoint(const Vec3& p) const
{
	// B1: lập đường thẳng d chứa M có VTCP u là VTPT n của mp
	// B2: hình chiếu là điểm giao giữa đth d và mp
	Ray d = Ray(p, this->normal);
	Vec3 H = this->intersect(d);
	return H;
}


// Tìm hình chiếu của vector (A) lên mặt phẳng
Vec3 Plane::projectVec(const Vec3& v) const
{
	//B1: tìm hình chiếu của vec A lên VTPT của mp
	//B2: lấy A - vec hình chiếu = kết quả
	Vec3 n = this->normal;
	Vec3 A2 = n.project(v);
	return v - A2;
}

// Tìm hình chiếu của đường thẳng lên mặt phẳng
Ray Plane::projectRay(const Ray& ray) const
{
	// B1: tìm mặt phẳng Q chứa d (có VTPT n1 = tích có hướng (u, n))
	// B2: lập mặt phẳng Q điểm thuộc d và VTPT n1
	// B3: hình chiếu là đường thẳng giao nhau giữa 2 mp
	Vec3 M = ray.getPoint();
	Vec3 u = ray.getDirection();
	Vec3 n = this->normal;

	Vec3 n1 = u.cross(n);
	Plane Q = Plane(n1, M);
	Ray projection = this->intersect(Q);
	return projection;
}
