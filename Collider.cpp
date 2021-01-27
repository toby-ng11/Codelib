#include "Collider.h"



Vec4 Collider::quadraticEquation(const Ray& ray, const Sphere& sphere)
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

bool Collider::RaySphereCollisionDetected(const Ray& ray, const Sphere& sphere)
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

void Collider::RaySphereCollisionPoint(const Ray& ray, const Sphere& sphere)
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

bool Collider::RayAABCollisionDetected(const Ray& ray, const AABox& box)
{
	Vec3 V = ray.getDirection();
	if (V == 0) return false;
	else {
		return true;
		cout << "Ray - Box collision detected !" << endl;
	}
}

void Collider::RayAABCollisionPoint(const Ray& ray, const AABox& box)
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







