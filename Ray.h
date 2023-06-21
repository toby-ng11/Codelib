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
	// we have p (point) = Vec3(x0, y0, z0), v (direction) = Vec3(a, b, c) we have this ray equation | d = p + vt  |
public:
	// khai báo các kiểu con đà điểu
	Ray() :point(Vec3(0, 0, 0)), direction(Vec3(0, 0, 0)) {}                //d = p + vt   //constructor
	Ray(const Vec3& p, const Vec3& v) : point(p), direction(v) {}           // đường thẳng gồm 2 tham số là điểm và VTCP
	//Ray(const Vec3& p1, const Vec3& p2);                                  // đường thẳng đi qua 2 điểm
	~Ray() {} // deconstructor

	// lấy giá trị + khai báo
	void set(const Vec3& p, const Vec3 v);                                  // đường thẳng từ 1 điểm và VTCP
	//void set2P(const Vec3& p1, const Vec3& p2);                           // đường thẳng đi qua 2 điểm
	void setPoint(Vec3& p) { point = p; }
	void setDirection(const Vec3& v) { direction = v; }

	const Vec3& getPoint() const { return point; }
	const Vec3& getDirection() const { return direction; }
	float getDistance(const Vec3& p) const;                                 // khoảng cách từ điểm đến đường thằng
	float getDistance(const Ray& r) const;                                  // khoảng cách giữa hai đường thẳng chéo nhau  

	// kiểm tra đường thẳng
	void printSelf();

	// toán đường thẳng
	Vec3 currentPosition(const float t) const;
	Vec3 intersect(const Ray& ray) const;                                   // đường thẳng giao đường thẳng
	bool isIntersected(const Ray& ray) const;
	Vec3 projectPoint(const Vec3& p) const;                                 // hình chiếu của điểm lên đường thẳng


private:
	Vec3 point;                                                             // Starting position of ray
	Vec3 direction;                                                         // Direction of ray


};
#endif
