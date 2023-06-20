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
	// definition
	Plane() :normal(Vec3(0, 0, 1)), d(0), normalLength(1), distance(0) {}
	Plane(float x_, float y_, float z_, float d_);           // ax + by + cz + d = 0
	Plane(const Vec3& p1, const Vec3& p2, const Vec3& p3);   // Construct plane from 3 points
	Plane(const Vec3& normal, const Vec3& point);            // Construct plane from plane's normal and a point
	Plane(const Vec4& v);                                    // Construct plane as a Vec4
	~Plane() {};

	// check 
	void printSelf() const;

	// setter
	void set(float a, float b, float c, float d);            // mặt phẳng là Vec4
	void set(const Vec3& normal, const Vec3& point);         // mặt phẳng từ một điểm và vector pháp tuyến
	void set(const Vec3& p1, const Vec3& p2, const Vec3& p3);// mặt phẳng từ 3 điểm

	
	const Vec3& getNormal() const { return normal; }         // lấy VPT của mp
	float getD() const { return d; }                         // lấy giá trị d
	float getNormalLength() const { return normalLength; }   // lấy độ dài VTPT của mp
	float getDistance() const { return distance; }           // lấy khoảng cách từ mặt phẳng tới gốc toạ độ
	float getDistance(const Vec3& point) const;              // lấy khoảng cách từ mặt phẳng tới một điểm
	float getAngle(const Ray& ray) const;                    // góc giữa đường thẳng và mặt phẳng

	// lấy giá trị normalize của mặt phẳng
	void normalize();

	// hàm toán mặt phẳng
	Vec3 intersect(const Ray& ray) const;                    // mp giao đường thẳng
	Ray intersect(const Plane& plane) const;                 // mp giao mp
	bool isIntersected(const Ray& ray) const;
	bool isIntersected(const Plane& plane) const;
	Vec3 projectPoint(const Vec3& p) const;                  // tìm hình chiếu của điểm lên mặt phẳng
	Vec3 projectVec(const Vec3& v) const;                    // tìm hình chiếu vector lên mặt phẳng
	Ray projectRay(const Ray& ray) const;                    // tìm hình chiếu của đường thẳng lên mặt phẳng


private:
	Vec3 normal;                                             // Normal vector of the plane
	float d;                                                 // d = - (a * x0 + b * y0 + c * z0)
	float normalLength;                                      // Plane's normal vector's length
	float distance;                                          // Distance to the Origin

};
#endif
