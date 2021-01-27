#include <iostream>
#include <fstream>
#include "Collider.h"
#include "ColliderPhysics.h"
#include "Quaternion.h"

using namespace std;

class Queue {
	int size;
	int* queue;

public:
	Queue() {
		size = 0;
		queue = new int[100];
	}
	void remove() {
		if (size == 0) {
			cout << "Queue is empty" << endl;
			return;
		}
		else {
			for (int i = 0; i < size - 1; i++) {
				queue[i] = queue[i + 1];
			}
			size--;
		}
	}
	void print() {
		if (size == 0) {
			cout << "Queue is empty" << endl;
			return;
		}
		for (int i = 0; i < size; i++) {
			cout << queue[i] << " <- ";
		}
		cout << endl;
	}
	//your code goes here

	void add(int x) {
		queue[size] = x;
		size++;
	}

	Queue operator+(Queue q) {

	}

};



int main()
{
	Queue q;
	q.add(42); q.add(2); q.add(8); q.add(1);
	q.print();
	q.remove();
	q.add(128);
	q.print();
	q.remove();
	q.remove();
	q.print();

	

















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


	cout << "\n\n\nCreated by Tue Nguyen - N01303773" << endl;

	return 0;
}
