#include "Body.h"


void Body::set(const Vec3& position_, const Vec3& velocity_, const Vec3& accel_, const float radius_)
{
	this->position = position_;
	this->velocity = velocity_;
	this->acceleration = accel_;
	this->radius = radius_;
}

void Body::setPos(const Vec3& position_)
{
	this->position = position_;
}

void Body::setVel(const Vec3& velocity_)
{
	this->velocity = velocity_;
}

void Body::setAccel(const Vec3& accel_)
{
	this->acceleration = accel_;
}

void Body::printSelf()
{
	cout << "The object has position = " << position << " , velocity = " << velocity << ", acceleration = " << acceleration << " and radius = " << radius << endl;
}

void Body::Update(float deltaTime)
{
	velocity += acceleration * deltaTime;
	position += velocity * deltaTime + 0.5f * acceleration * deltaTime * deltaTime;
}