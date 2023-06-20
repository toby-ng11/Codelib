#include "ColliderPhysics.h"

Vec4 ColliderPhysics::quadraticEquation(const Ray& ray, const Sphere& sphere)
{
	Vec4 quaVec;
	Vec3 V = ray.getDirection();
	Vec3 S = ray.getPoint();       // start
	Vec3 C = sphere.getPosition(); // center
	float R = sphere.getRadius();

	//A=V^2=V.V
	quaVec.x = V * V; //cout << quaVec.x << endl;
	//B=2(S-C).V
	quaVec.y = 2 * V.dot(S - C); //cout << quaVec.y << endl;
	//C =(S-C)^2 -R^2
	quaVec.z = (S - C) * (S - C) - R * R;// cout << quaVec.z << endl;
	//discriminant = B*B-4*A*C
	//if (quaVec.y * quaVec.y)
	quaVec.w = quaVec.y * quaVec.y - 4 * quaVec.x * quaVec.z; //cout << quaVec.w << endl;// discriminant
	return quaVec;
}

bool ColliderPhysics::RaySphereCollisionDetected(const Ray& ray, const Sphere& sphere)
{
	Vec4 quaVec = quadraticEquation(ray, sphere);
	// check if the discriminant (quaVec.w) is negative
	if (quaVec.w < 0.0f)
		return false;// No collision
	else
	{
		return true;// It depends of t :):D*/
		cout << "Ray-Sphere Collision Detected!" << endl;;
	}
}

void ColliderPhysics::RaySphereCollisionPoint(const Ray& ray, const Sphere& sphere)
{
	Vec4 quaVec = quadraticEquation(ray, sphere);
	if (RaySphereCollisionDetected(ray, sphere))
	{
		//t1= -B-sqrt(Discriminant)/2A
		float t1 = (-quaVec.y - sqrt(quaVec.w)) / (2 * quaVec.x); //cout << t1 << endl;

		//t2= -B+sqrt(Discriminant)/2A
		float t2 = (-quaVec.y + sqrt(quaVec.w)) / (2 * quaVec.x); //cout << t2 << endl;

		if (t1 >= 0 && t2 <= 0 || t1 <= 0 && t2 >= 0)
			cout << "There are two collision points: " << ray.currentPosition(t1) << " with t = " << t1 << " and " << ray.currentPosition(t2) << " with t = " << t2 << endl;
		else if (t1 <= 0 && t2 <= 0 && t1 != t2 || t1 >= 0 && t2 >= 0 && t1 != t2)
			cout << "There are two collision points: " << ray.currentPosition(t1) << " with t = " << t1 << " and " << ray.currentPosition(t2) << " with t = " << t2 << endl;
		else if (t1 == t2)
			cout << "There are only one collision point: " << ray.currentPosition(t1) << " with t = " << t1 << endl;
	}
	else cout << "No intersection between ray and sphere.";
}

bool ColliderPhysics::RayAABCollisionDetected(const Ray& ray, const AABox& box)
{
	Vec3 V = ray.getDirection();
	if (V == 0) return false;
	else {
		return true;
		cout << "Ray - Box collision detected !" << endl;
	}
}

void ColliderPhysics::RayAABCollisionPoint(const Ray& ray, const AABox& box)
{
	float rx = box.getRx();
	float ry = box.getRy();
	float rz = box.getRz();
	float tx = 0, ty = 0, tz = 0, t = NAN;

	Vec3 boxD = Vec3(rx, ry, rz);
	Vec3 S = ray.getPoint();
	Vec3 V = ray.getDirection();
	Vec3 interPx, interPy, interPz;

	if (V.x != 0)
		if (V.x > 0) tx = (-S.x) / V.x;
		else tx = (rx - S.x) / V.x;
	interPx = ray.currentPosition(tx);

	if (V.y != 0)
		if (V.y > 0) ty = (-S.y) / V.y;
		else ty = (ry - S.y) / V.y;
	interPy = ray.currentPosition(ty);

	if (V.z != 0)
		if (V.z > 0) tz = (-S.z) / V.z;
		else tx = (rz - S.z) / V.z;
	interPz = ray.currentPosition(tz);

	//cout << tx << "\n" << ty << "\n" << tz << endl;

	if (0 <= interPx && interPx <= boxD) t = tx;
	if (0 <= interPy && interPy <= boxD) t = ty;
	if (0 <= interPz && interPz <= boxD) t = tz;
	//cout << t << endl;
	cout << "The closest collision point is: " << ray.currentPosition(t) << endl;

}

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
