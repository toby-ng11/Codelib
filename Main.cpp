#include <iostream>
#include <fstream>
#include "ColliderPhysics.h"
#include "Quaternion.h"
#include <stdio.h>

using namespace std;

int main()
{
	/* ofstream file;
	file.open("SpherePlane.csv");


	// Physic
	ColliderPhysics c;
	Plane Q(-1, 1, 0, 0);
	Body DynamicSphere(Vec3(-10, 2, 0), Vec3(1, 0, 0), Vec3(0, 0, 0), 1);
	DynamicSphere.printSelf();
	
	cout << "Time\tPosition\tVeclocity" << endl;
	file << "Time, Pos1.x, Pos1.y, Pos2.x, Pos2.y" << endl;
	
	for (float time = 0.0f; time <= 10.0f; time++)
	{
		DynamicSphere.Update(time);

	    Vec3 responseV = c.SpherePlaneCollisionResponse(DynamicSphere, Q);
		DynamicSphere.setVel(responseV);

		Vec3 Pos = DynamicSphere.getPosition();
		Vec3 Vel = DynamicSphere.getVelocity();

		cout << time << "\t" << Pos << "\t" << Vel << endl;
		file << time << ","
			<< Pos.x << "," << Pos.y << endl;
	}
	
	cout << "====================================" << endl;

	Body DynamicA(Vec3(3, 4, 0), Vec3(0, -4, 0), Vec3(0, 0, 0), 1.12f);
	Body StaticB(Vec3(2, 2, 0), Vec3(0, 0, 0), Vec3(0, 0, 0), 1.12f);

	DynamicA.printSelf(); StaticB.printSelf();

	for (float time = 0.0f; time <= 10.0f; time++)
	{
		DynamicA.Update(time);

		Vec3 vf = c.SphereStaticSphereCollisionResponse(DynamicA, StaticB);
		DynamicA.setVel(vf);

		Vec3 Pos = DynamicA.getPosition();
		Vec3 Vel = DynamicA.getVelocity();

		cout << time << "\t" << Pos << "\t" << Vel << endl;
		file << "," << "," << ","
			<< Pos.x << "," << Pos.y << endl;
	}

	file.close();

	Body Ball1(Vec3(5.0f, -8.66f, 0.0f), Vec3(0.0f, 6.0f, 0.0f), Vec3(0, 0, 0), 10.0f);
	Body Ball2(Vec3(0.0f, 0.0f, 0.0f), Vec3(0.0f, 0.0f, 0.0f), Vec3(0, 0, 0), 10.0f);
	cout << "\n";
	c.SphereSphereCollisionResponse(Ball1, Ball2, 0.9f); */
	
	/*
	Matrix3 I1 = { 20.0f, 0.0f, 0.0f, 0.0f, 40.0f, 0.0f, 0.0f, 0.0f, 20.0f };
	Matrix3 I2 = { 0.1f, 0.0f, 0.0f, 0.0f, 0.1f, 0.0f, 0.0f, 0.0f, 0.1f };

	Vec3 n(2.0f / 3.0f, 1.0f / 3.0f, 2.0f / 3.0f);
	Vec3 r1(-8.0f, 1.0f, 0.0f), r2(2.0f, -1.0f, 0.0f);
	Vec3 v1i(0.0f, 0.0f, 0.0f), v2i(20000.0f, 10000.0f, 20000.0f);

	float m1 = 100.0f, m2 = 1.0f, e = 0.5f;

	float vr = n.dot(v1i - v2i);

	float numerator = -vr * (e + 1.0f);
	float denominator = 1.0f / m1 + 1.0f / m2 + n.dot((I1.inverse() * r1.cross(n)).cross(r1)) + n.dot((I2.inverse() * r2.cross(n)).cross(r2));
	float J = numerator / denominator; cout << "J = " << J << endl;

	Vec3 v_ship = v1i + J * n / m1; cout << "Starship's linear velocity = " << v_ship << endl;
	Vec3 angular_ship = v1i + I1 * r1.cross(J * n); cout << "Starship's angular velocity = " << angular_ship << endl;
	*/

	Vec3 up(0.0f, 1.0f, 0.0f), object_vel(1.0f, 0.0f, 0.0f);
	Vec3 rotationAxis = up.cross(object_vel); rotationAxis.normalize(); cout << rotationAxis << endl;
	Vec3 angularVec = rotationAxis * object_vel.length(); cout << angularVec << endl;
	Quaternion angularVecQuat(0.0f, angularVec); cout << angularVecQuat << endl;
	Quaternion q(up, -45.0f); cout << q << endl;
	Quaternion newQuat(0.0f, 0.0f, 0.0f, 0.0f);
	newQuat += q * angularVecQuat*0.5f; cout << newQuat << endl;



	cout << "\n\n\nCreated by Tue Nguyen - N01303773" << endl;

	return 0;
}
