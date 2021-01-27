#ifndef BODY_H
#define BODY_H


#include "Vector.h"


using namespace std;

class Body
{
public:
	

	Body() :position(Vec3(0, 0, 0)), velocity(Vec3(0, 0, 0)), acceleration(Vec3(0, 0, 0)), radius(1) {}
	Body(const Vec3& position_, const Vec3& velocity_, const Vec3& accel_, const float radius_)
	: position(position_), velocity(velocity_), acceleration(accel_), radius(radius_) {}
	~Body() {}

	void printSelf();

	void Update(float deltaTime);

	void set(const Vec3& position_, const Vec3& velocity_, const Vec3& accel_, const float radius_);
	void setPos(const Vec3& position_);
	void setVel(const Vec3& velocity_);
	void setAccel(const Vec3& accel_);

	Vec3 getPosition() const { return position; }
	Vec3 getVelocity() const { return velocity; }

	float getRadius() const { return radius; }




	

private:
	Vec3 position, velocity, acceleration;
	float radius;




};

#endif // ! BODY_H
