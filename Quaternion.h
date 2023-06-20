#ifndef QUATERNION_H
#define QUATERNION_H

#include "Vector.h"
#include "Matrix.h"

using namespace std;

class Quaternion;
class EulerAngles
{
private: float yaw, pitch, roll;
public:

	EulerAngles() :yaw(0), pitch(0), roll(0) {};
	EulerAngles(float yaw_, float pitch_, float roll_) :yaw(yaw_), pitch(pitch_), roll(roll_) { };

	Quaternion ConvertQuat();

	friend ostream& operator<<(ostream& os, const EulerAngles& q);
};

class Quaternion
{
private:
	float x, y, z, w;
public:

	Quaternion() :w(1), x(0), y(0), z(0) {};
	Quaternion(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}
	Quaternion(const Vec4& vector) { this->w = vector.x; this->x = vector.y; this->y = vector.z; this->z = vector.w; }
	Quaternion(const float w_, const Vec3& vector) { this->w = w_; this->x = vector.x; this->y = vector.y; this->z = vector.z; }
	Quaternion(const Vec3& axis, float angleInDegrees) // quater theo trục và góc
	{
		Vec3 temVec = axis;
		temVec.normalize();
		//normalize(temVec);
		angleInDegrees *= DEGREES_TO_RADIANS;
		this->w = cosf(angleInDegrees / 2);
		this->x = temVec.x * sinf(angleInDegrees / 2);
		this->y = temVec.y * sinf(angleInDegrees / 2);
		this->z = temVec.z * sinf(angleInDegrees / 2);
	}

	void print() { cout << "Quaternion: [" << this->w << ", (" << this->x << ", " << this->y << ", " << this->z << ")]" << endl; }
	void set(float w, float x, float y, float z);
	float getW() const { return w; }
	Vec3 getVec() const { return Vec3(x, y, z); }
	float getAngle() const { return acosf(w) * 2 * RADIANS_TO_DEGREES; }      // angle of rotation
	Vec3 getaxis() const { return Vec3(getVec() / sinf(getAngle() * DEGREES_TO_RADIANS / 2)); }        // axis of rotation

	// Các phép toán
	Quaternion  operator-() const;                                             // lấy quat âm
	Quaternion  operator+(const Quaternion& q) const;                          // cộng 2 quaternion
	Quaternion& operator+=(const Quaternion& q);                               // cộng quaternion với chính nó
	Quaternion  operator-(const Quaternion& q) const;                          // trừ 2 quaternion
	Quaternion& operator-=(const Quaternion& rhs);                             // trừ quaternion với chính nó
	Quaternion  operator*(const Quaternion& q) const;                          // nhân 2 quaternion (quaternion multiplication)
	Quaternion  operator*(const Vec3& v) const;                                // liên hợp để xoay
	Quaternion  operator*(float s) const;                                      // nhân thành phần scalar
	Quaternion  operator/(const float s) const;                                // chia thành phần quaternion cho 1 số

	friend Quaternion operator*(const float a, const Quaternion q);
	friend ostream& operator<<(ostream& os, const Quaternion& q);

	float dot(const Quaternion& q);                                           // tích vô hướng quaternion
	float length() const;                                                     // độ dài quaternion
	Quaternion& normalize();                                                  // chuẩn hoá quater
	Quaternion& conjuagate();                                                 // lấy liên hợp
	Quaternion& inverse();                                                    // lấy nghịch đảo
	Vec3 rotate(const Vec3& v);                                               // xoay 1 vector theo 1 quater
	Matrix3 convertMatrix3();                                                 // quater -> mat3
	Matrix4 convertMatrix4();                                                 // quater -> mat4
	EulerAngles ConvertEuler();
};



///////////////////////////////////////////////////////////////////////////
// Các tính toán
///////////////////////////////////////////////////////////////////////////

inline ostream& operator<<(ostream& os, const EulerAngles& angle)
{
	os << "EulerAngles = (yaw: " << angle.yaw << ", roll: " << angle.roll << ", pitch: " << angle.pitch << ")";
	return os;
}

inline void Quaternion::set(float w, float x, float y, float z) { this->w = w, this->x = x; this->y = y; this->z = z; }

inline Quaternion Quaternion::operator-() const
{
	return Quaternion(-w, -x, -y, -z);
}

inline Quaternion Quaternion::operator+(const Quaternion& q) const { return Quaternion(w + q.w, x + q.x, y + q.y, z + q.z); }

inline Quaternion& Quaternion::operator+=(const Quaternion& q)
{
	w += q.w; x += q.x; y += q.y; z += q.z;
	return *this;
}

inline Quaternion Quaternion::operator-(const Quaternion& q) const
{
	return Quaternion(w - q.w, x - q.z, y - q.y, z - q.z);
}

inline Quaternion& Quaternion::operator-=(const Quaternion& rhs)
{
	w -= rhs.w;  x -= rhs.x;  y -= rhs.y;  z -= rhs.z;
	return *this;
}

inline Quaternion Quaternion::operator*(const Quaternion& q) const
{
	Vec3 v1 = this->getVec();
	Vec3 v2 = q.getVec();
	float w1 = this->getW();
	float w2 = q.getW();
	return Quaternion(w1 * w2 - v1 * v2, w1 * v2 + w2 * v1 + v1.cross(v2));
}

inline Quaternion Quaternion::operator*(const Vec3& v) const
{
	Quaternion q(0, x, y, z);
	return *this * q;
}

inline Quaternion Quaternion::operator*(float s) const { 
	return Quaternion(w * s, x * s, y * s, z * s); 
}

inline Quaternion Quaternion::operator/(const float s) const {
	return Quaternion(w / s, x / s, y / s, z / s); 
}

inline Quaternion operator*(const float a, const Quaternion q)
{
	return Quaternion(a * q.w, a * q.x, a * q.y, a * q.z);
}

inline ostream& operator<<(ostream& os, const Quaternion& q)
{
	os << "(" << q.w << ", " << q.x << ", " << q.y << ", " << q.z << ")";
	return os;
}

inline float Quaternion::dot(const Quaternion& q)
{
	Vec3 v1 = this->getVec();
	Vec3 v2 = q.getVec();
	float w1 = this->getW();
	float w2 = q.getW();
	return w1 * w2 + v1 * v2;
}

inline float Quaternion::length() const
{
	return sqrtf(w * w + x * x + y * y + z * z);
}

inline Quaternion& Quaternion::normalize()
{
	float m = this->length();
	w /= m; x /= m; y /= m; z /= m;
	return *this;
}

inline Quaternion& Quaternion::conjuagate()
{
	x = -x; y = -y; z = -z;
	return *this;
}

inline Quaternion& Quaternion::inverse()
{
	float m = this->length();
	w /= m * m;
	x /= -m * m;
	y /= -m * m;
	z /= -m * m;
	return *this;
}

inline Vec3 Quaternion::rotate(const Vec3& v)
{
	Quaternion P(0, v.x, v.y, v.z);
	Quaternion Q = *this;
	Quaternion Qinversed = this->inverse();
	Vec3 rotatedVec = (Q * P * Qinversed).getVec();
	return rotatedVec;
}

inline Matrix3 Quaternion::convertMatrix3()
{
	this->normalize();
	return Matrix3(1.0f - 2.0f * y * y - 2.0f * z * z, 2.0f * x * y - 2.0f * w * z,        2.0f * x * z + 2.0f * w * y,
		           2.0f * x * y + 2.0f * w * z,        1.0f - 2.0f * x * x - 2.0f * z * z, 2.0f * y * z - 2.0f * w * x,
		           2.0f * x * z - 2.0f * w * y,        2.0f * y * z + 2.0f * w * x,        1.0f - 2.0f * x * x - 2.0f * y * y);
}

inline Matrix4 Quaternion::convertMatrix4()
{
	this->normalize();
	return Matrix4(1.0f - 2.0f * y * y - 2.0f * z * z, 2.0f * x * y - 2.0f * w * z,        2.0f * x * z + 2 * w * y,           0.0f,
		           2.0f * x * y + 2.0f * w * z,        1.0f - 2.0f * x * x - 2.0f * z * z, 2.0f * y * z - 2.0f * w * x,        0.0f,
		           2.0f * x * z - 2.0f * w * y,        2.0f * y * z + 2.0f * w * x,        1.0f - 2.0f * x * x - 2.0f * y * y, 0.0f,
		           0.0f,                               0.0f,                               0.0f,                               1.0f);
}

inline EulerAngles Quaternion::ConvertEuler()
{
	float roll, pitch, yaw;

	// roll (x-axis rotation)
	float sinr_cosp = 2 * (w * x + y * z);
	float cosr_cosp = 1 - 2 * (x * x + y * y);
	roll = atan2f(sinr_cosp, cosr_cosp);

	// pitch (y-axis rotation)
	float sinp = 2 * (w * y - z * x);
	if (abs(sinp) >= 1)
	{
		pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	}
	else
	{
		pitch = asinf(sinp);
	}

	// yaw (z-axis rotation)
	float siny_cosp = 2 * (w * z + x * y);
	float cosy_cosp = 1 - 2 * (y * y + z * z);
	yaw = atan2f(siny_cosp, cosy_cosp);

	return EulerAngles(yaw, pitch, roll);
}

inline Quaternion EulerAngles::ConvertQuat()
{
	// Abbreviations for the various angular functions
	float cy = cos(yaw * 0.5f);
	float sy = sin(yaw * 0.5f);
	float cp = cos(pitch * 0.5f);
	float sp = sin(pitch * 0.5f);
	float cr = cos(roll * 0.5f);
	float sr = sin(roll * 0.5f);

	return Quaternion(cr * cp * cy + sr * sp * sy,
		sr * cp * cy - cr * sp * sy,
		cr * sp * cy + sr * cp * sy,
		cr * cp * sy - sr * sp * cy);
}

#endif // !QUATERNION_H

