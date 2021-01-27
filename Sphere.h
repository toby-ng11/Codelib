#ifndef SPHERE_H
#define SPHERE_H

#include <iostream>
#include <math.h>
#include "Vector.h"
#include "Plane.h"

using namespace std;

class Sphere
{
public:

	// Phương trình chính tắc mặt cầu : (x - a)^2 + (y - b)^2 + (z - c)^2 = r^2        (r > 0)
	//                 __________________________________
	// trong đó :     |  Vec3(a, b, c) là tâm mặt cầu    |
	//                |  r là bán kính mặt cầu           |
	//                 __________________________________
	//
	// Phương trình tổng quát : x^2 + y^2 + z^2 - 2ax - 2by - 2cz + d = 0 
	// trong đó        |  d = a^2 + b^2 + c^2 - r^2       |

	Sphere() :position(Vec3(0, 0, 0)), radius(1), distance(0) {};
	Sphere(const Vec3& position_, float radius_);                             // lập mặt cầu từ tâm, bán kính
	Sphere(const Vec3& position, const Vec3& point);                          // lập mặt cầu từ tâm và một điểm thuộc mặt cầu
	//Sphere(const Vec3& p1, const Vec3& p2, const Vec3& p3, const Vec3& p4); // lập mặt cầu từ bốn điểm
	~Sphere() {}

	void printSelf();                                                         // in mặt cầu

	void set(const Vec3& position, float radius);                             // lập mặt cầu từ tâm và bán kính
	void set(const Vec3& position, const Vec3& point);                        // lập mặt cầu từ tâm và một điểm thuộc mặt cầu
	//void set(const Vec3& p1, const Vec3& p2, const Vec3& p3, const Vec3& p4); // lập mặt cầu từ bốn điểm

	const Vec3& getPosition()const { return position; }                       // lấy tâm mặt cầu
	float getRadius() const { return radius; }                                // lấy bán kính mặt cầu
	float getDistance() const { return distance; }                            // lấy khoảng cách từ tâm mặt cầu đến gốc toạ độ
	float getDistance(const Vec3& point)const;                                // lấy khoảng cách từ tâm mặt cầu dến một điểm
	float getDistance(const Ray& ray) const;                                  // lấy khoảng cách từ tâm mặt cầu đến đường thẳng (hình chiếu của tâm)   
	float getDistance(const Plane& plane) const;                              // lấy khoảng cách từ tâm mặt cầu dến mặt phẳng (hình chiếu của tâm)


private:
	Vec3 position;                                                            // tâm I mặt cầu
	float radius;                                                             // bán kính mặt cầu (r)
	float distance;                                                           // khoảng cách từ tâm mặt cầu đến gốc toạ độ

};
#endif

