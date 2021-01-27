#ifndef PLANE_H
#define PLANE_H

#include <math.h>
#include "Ray.h"

using namespace std;

class Ray;

class Plane
{

	// Phương trình mặt phẳng : a(x - x0) + b(y - y0) + c(z - z0) = 0
	//            ___________________________________________________________
	// trong đó :| Vec3(a, b, c) là vector pháp tuyến của mặt phẳng (normal) |
	//           | Vec3(x0, y0, z0) là điểm thuộc mặt phẳng                  |
	//            ___________________________________________________________
	// Mặt phẳng có dạng ax + by + cz + d = 0
	//             ________________________________________________________________________________
	// trong đó : | Vec3(a, b, c) là vector pháp tuyến của mặt phẳng (normal)                      |
	//            | d = - (a * x0 + b * y0 + c * z0) với Vec3(x0, y0, z0) là điểm thuộc mặt phẳng  |
	//              ( d = - VTPT.dot(điểm thuộc mặt phẳng) (d là - (tích vô hướng của VTPT với điểm thuộc mặt phẳng))
	//             ________________________________________________________________________________
	// Ta xem mặt phẳng là 1 Vec4(a, b, c, d) 
public:
	//định nghĩa
	Plane() :normal(Vec3(0, 0, 1)), d(0), normalLength(1), distance(0) {}
	Plane(float x_, float y_, float z_, float d_);           //ax + by + cz + d = 0
	Plane(const Vec3& p1, const Vec3& p2, const Vec3& p3);   // lập mặt phẳng từ 3 điểm
	Plane(const Vec3& normal, const Vec3& point);            // mặt phẳng từ một điểm và vector pháp tuyến
	Plane(const Vec4& v);                                    // mặt phẳng là 1 vec4(a,b,c,d)
	~Plane() {};

	// kiểm tra mặt phẳng
	void printSelf() const;

	// khai báo hàm
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
	Vec3 normal;                                             // VTPT của mặt phẳng
	float d;                                                 //d = - (a * x0 + b * y0 + c * z0)
	float normalLength;                                      // độ dài vector pháp tuyến
	float distance;                                          // khoảng cách từ mặt phẳng tới gốc toạ độ

};
#endif
