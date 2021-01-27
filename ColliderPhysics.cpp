#include "ColliderPhysics.h"


bool ColliderPhysics::SpherePlaneCollisionDetected(const Body& body1, const Plane& plane)
{
	Vec3 Position = body1.getPosition();            // vị trí hiện tại của vật
	float Radius = body1.getRadius();               // bán kính vật
	float distance = plane.getDistance(Position);   // khoảng cách từ tâm vật đến mặt phẳng
	if (distance <= Radius)
	{
		return true; // mặt cầu tiếp xúc mặt phẳng khi d(tâm mặt cầu đến mặt phẳng) = d(bán kính mặt cầu)
	}
	else return false;
}

Vec3 ColliderPhysics::SpherePlaneCollisionResponse(const Body& body1, const Plane& plane)
{
	Vec3 vi = body1.getVelocity();             // vector chuyển động
	Vec3 n = plane.getNormal();                // vTPT của mp
	n.normalize();

	if (SpherePlaneCollisionDetected(body1, plane))
	{

		Vec3 P = -n.project(vi); //cout << P << endl;
		Vec3 vf = vi + 2 * P;

		//cout << "The outgoing velocity: " << vf << endl;
		return vf;
	}
	else return vi;
}

bool ColliderPhysics::SphereSphereCollisionDetected(const Body& body1, const Body& body2)
{
	// Hai mặt cầu tiếp xúc nhau khi R1 + R2 = d(A, B) với A, B lần lượt là tâm mc
	Vec3 A = body1.getPosition(); Vec3 B = body2.getPosition();
	float R = body1.getRadius() + body2.getRadius();
	float d = (B - A).length();
	if (d <= R) return true;
	else return false;
}

Vec3 ColliderPhysics::SphereStaticSphereCollisionResponse(const Body& body1, const Body& staticsphere)
{
	Vec3 A = body1.getPosition(); Vec3 B = staticsphere.getPosition();
	Vec3 vi = body1.getVelocity();
	Vec3 normal = (B - A).normalize();

	if (SphereSphereCollisionDetected(body1, staticsphere))
	{
		Vec3 P = -normal.project(vi); cout << P << endl;
		Vec3 vf = vi + 2 * P;
		return vf;
	}
	else return vi;

}

void ColliderPhysics::SphereSphereCollisionResponse(const Body& body1, const Body& body2, float c)
{
	Vec3 A = body1.getPosition(); Vec3 B = body2.getPosition();
	Vec3 v1i = body1.getVelocity(); Vec3 v2i = body2.getVelocity();
	if (SphereSphereCollisionDetected(body1, body2))
	{
		Vec3 n = (B - A).normalize();
		float v1ni = v1i * n;
		float v2ni = v2i * n;

		float v2nf = v1ni * (1 + c) / 2;
		float v1nf = v1ni - v2nf;

		float v1n = v1nf - v1ni;
		float v2n = v2nf - v2ni;

		Vec3 v1f = v1i + v1n * n;
		Vec3 v2f = v2i + v2n * n;

		cout << "The outgoing velocity of ball 1 = " << v1f << endl;
		cout << "The outgoing velocity of ball 2 = " << v2f << endl;

	}

}
