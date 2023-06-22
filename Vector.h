// Created by Tue Nguyen

#ifndef VECTOR_H
#define VECTOR_H

#ifndef VERY_SMALL
#define VERY_SMALL 1.0e-7f
#endif

#ifndef M_PI
#define M_PI 3.14159265359f
#endif // !PI

#ifndef DEGREES_TO_RADIANS
#define DEGREES_TO_RADIANS (M_PI / 180.0f)
#endif	

#ifndef RADIANS_TO_DEGREES
#define RADIANS_TO_DEGREES (180.0f/M_PI)
#endif

#include <iostream>
#include <math.h>

using namespace std;

// VECTOR 2D
class Vec2
{
public:
	float x, y;

	Vec2() :x(0), y(0) {};
	Vec2(float x_, float y_) :x(x_), y(y_) {};

	Vec2    operator=(const Vec2& v);
	Vec2    operator-() const;
	Vec2    operator+(const Vec2& v) const;
	Vec2    operator-(const Vec2& v) const;
	Vec2&   operator+=(const Vec2& v);
	Vec2&   operator-=(const Vec2& v);
	Vec2    operator*(const float s) const;
	Vec2&   operator*=(const float s);
	Vec2    operator/(const float s) const;
	Vec2&   operator/=(const float s);
	bool    operator==(const Vec2& v) const;
	bool    operator!=(const Vec2& v) const;
	bool    operator<(const Vec2& v) const;
	float   operator*(const Vec2& v) const;
	float   operator[](int index) const;
	float&  operator[](int index);

	friend  Vec2 operator*(const float s, const Vec2 v);
	friend  ostream& operator<<(ostream& os, const Vec2& v);

	void    set(float x, float y);
	float   length() const;
	float   distance(const Vec2& v) const;
	Vec2&   normalize();
	float   dot(const Vec2& v) const;
};

// VECTOR 3D
class Vec3
{
public:
	float x, y, z;

	Vec3() :x(0), y(0), z(0) {};
	Vec3(float x_, float y_, float z_) :x(x_), y(y_), z(z_) {};
	Vec3(Vec2 v, float z) { this->x = v.x; this->y = v.y, this->z = z; }

	Vec3    operator=(const Vec3& v);
	Vec3    operator-() const;
	Vec3    operator+(const Vec3& v) const;
	Vec3    operator-(const Vec3& v) const;
	Vec3&   operator+=(const Vec3& v);
	Vec3&   operator-=(const Vec3& v);
	Vec3    operator*(const float s) const;
	Vec3&   operator*=(const float s);
	Vec3    operator/(const float s) const;
	Vec3&   operator/=(const float s);
	bool    operator==(const Vec3& v) const;
	bool    operator==(const float s) const;
	bool    operator!=(const Vec3& v) const;
	bool    operator<(const Vec3& v) const;
	bool    operator<=(const Vec3& v) const;
	float   operator*(const Vec3& v) const;
	bool    operator<=(const float s) const;
	float   operator[](int index) const;
	float&  operator[](int index);

	friend  Vec3 operator*(const float s, const Vec3& v);
	friend  bool operator<=(const float s, const Vec3& v);
	friend  ostream& operator<<(ostream& os, const Vec3& vec);

	void    set(float x, float y, float z);
	float   length() const;
	float   distance(const Vec3& v) const;
	float   dot(const Vec3& b) const;
	float   dot(const Vec3& v1, const Vec3& v2);
	Vec3    cross(const Vec3& vector) const;
	float   angleR(const Vec3& v) const;
	float   angleD(const Vec3& v) const;
	Vec3&   normalize();
	void    lerp(const Vec3& b, const float t);
	void    rotateZ(const float angle);
	Vec3    project(const Vec3& vectorb) const;
};

// VECTOR 4D
class Vec4
{
public:
	float  x, y, z, w;

	Vec4() :x(0), y(0), z(0), w(0) {};
	Vec4(float x_, float y_, float z_, float w_) :x(x_), y(y_), z(z_), w(w_) {}
	Vec4(const Vec2& v, float z, float w) { this->x = v.x; this->y = v.y; this->z = z; this->w = w; }
	Vec4(const Vec3& v, float w) { this->x = v.x; this->y = v.y; this->z = v.z; this->w = w; }

	Vec4    operator=(const Vec4& v);
	Vec4    operator-() const;
	Vec4    operator+(const Vec4& v) const;
	Vec4    operator-(const Vec4& v) const;
	Vec4&   operator+=(const Vec4& v);
	Vec4&   operator-=(const Vec4& v);
	Vec4    operator*(const float s) const;
	Vec4&   operator*=(const float s);
	Vec4    operator/(const float s) const;
	Vec4&   operator/=(const float s);
	bool    operator==(const Vec4& v) const;
	bool    operator!=(const Vec4& v) const;
	bool    operator<(const Vec4& v) const;
	float   operator*(const Vec4& v) const;
	float   operator[](int index) const;
	float&  operator[](int index);

	friend  Vec4 operator*(const float s, const Vec4 v);
	friend  ostream& operator<<(ostream& os, const Vec4& v);

	void    set(float x, float y, float z, float w);
	float   length() const;
	float   distance(const Vec4& v) const;
	Vec4&   normalize();
	float   dot(const Vec4& v) const;
};

//////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////// VECTOR 2D ////////////////////////////////////////////////

//////////////////////////////////////////// Toán tử /////////////////////////////////////////////////

// Assign vector
inline Vec2 Vec2::operator=(const Vec2& v) {
	set(v.x, v.y);
	return *this;
}

// - Vec2
inline Vec2 Vec2::operator-() const { return Vec2(-x, -y); };

// Vec2 + Vec2
inline Vec2 Vec2::operator+(const Vec2& v) const { return Vec2(x + v.x, y + v.y); }

// Vec2 - Vec2
inline Vec2 Vec2::operator-(const Vec2& v) const { return Vec2(x - v.x, y - v.y); }

// Vec2 += Vec2
inline Vec2& Vec2::operator+=(const Vec2& v) { x += v.x; y += v.y; return *this; }

// Vec2 -= Vec2
inline Vec2& Vec2::operator-=(const Vec2& v) { x -= v.x; y -= v.y; return *this; }

// Vec2 * number
inline Vec2 Vec2::operator*(const float s) const { return Vec2(x * s, y * s); }

// Vec2 *= number
inline Vec2& Vec2::operator*=(const float s) { x *= s; y *= s; return *this; }

// Vec2 / number
inline Vec2 Vec2::operator/(const float s) const { return Vec2(x / s, y / s); }

// Vec2 /= number
inline Vec2& Vec2::operator/=(const float s) { x /= s; y /= s; return *this; }

// Vec2 == Vec2 ?
inline bool Vec2::operator==(const Vec2& v) const { return (x == v.x) && (y == v.y); }

// Vec2 != Vec2 ?
inline bool Vec2::operator!=(const Vec2& v) const { return (x != v.x) || (y != v.y); }

// Vec2 < Vec2 ?
inline bool Vec2::operator<(const Vec2& v) const { return (x < v.x) && (y < v.y); }

// Dot (scalar) product
inline float Vec2::operator*(const Vec2& v) const { return x * v.x + y * v.y; }

// Vector array
inline float Vec2::operator[](int index) const { return (&x)[index]; }

// Vector array
inline float& Vec2::operator[](int index) { return (&x)[index]; }

// number * Vec2
inline Vec2 operator*(const float s, const Vec2 v) { return Vec2(s * v.x, s * v.y); }

// Print self
inline ostream& operator<<(ostream& os, const Vec2& v) {
	os << "(" << v.x << ", " << v.y << ")";
	return os;
}

//////////////////////////////////////////// Các hàm /////////////////////////////////////////////////

inline void Vec2::set(float x, float y) { this->x = x; this->y = y; }

// Get vector lenght
inline float Vec2::length() const { return sqrtf(x * x + y * y); }

// Distance between vector
inline float Vec2::distance(const Vec2& v) const { return sqrtf((v.x - x) * (v.x - x) + (v.y - y) * (v.y - y)); }

// Normalize vector
inline Vec2& Vec2::normalize() {
	float m = this->length();
	x /= m;
	y /= m;
	return *this;
}

// Dot (scalar) product
inline float Vec2::dot(const Vec2& v) const { return *this * v; }

//////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////// VECTOR 3D ////////////////////////////////////////////////

///////////////////////////////////// Overload phép toán /////////////////////////////////////////////

//Gán vector
inline Vec3 Vec3::operator=(const Vec3& v) {
	set(v.x, v.y, v.z);
	return *this;
}

// Lấy vector đối 
inline Vec3 Vec3::operator-() const { return Vec3(-x, -y, -z); }

// Cộng 2 vector
inline Vec3 Vec3::operator+(const Vec3& v) const { return Vec3(x + v.x, y + v.y, z + v.z); }

// Trừ 2 vector
inline Vec3 Vec3::operator-(const Vec3& v) const { return Vec3(x - v.x, y - v.y, z - v.z); }

// Cộng vector với chính nó
inline Vec3& Vec3::operator+=(const Vec3& v) {
	x += v.x;
	y += v.y;
	z += v.z;
	return *this;
}

// Trừ vector với chính nó
inline Vec3& Vec3::operator-=(const Vec3& v) {
	x -= v.x;
	y -= v.y;
	z -= v.z;
	return *this;
}

// Nhân vector với 1 số
inline Vec3 Vec3::operator*(const float s) const { return Vec3(x * s, y * s, z * s); }

// Nhân 1 số với chính vector đó
inline Vec3& Vec3::operator*=(const float s) {
	x *= s;
	y *= s;
	z *= s;
	return *this;
}

// Chia vector với 1 số
inline Vec3 Vec3::operator/(const float s) const {
#ifdef _DEBUG  /// If in debug mode let's worry about divide by zero or nearly zero!!! 
	if (fabs(s) < VERY_SMALL) {
		string errorMsg("Divide by nearly zero! ");
		throw errorMsg;
	}
#endif
	float r = 1.0f / s;
	return *this * r;
}

// Chia 1 số với chính vector đó
inline Vec3& Vec3::operator/=(const float s) {
#ifdef _DEBUG  /// If in debug mode let's worry about divide by zero or nearly zero!!! 
	if (fabs(s) < VERY_SMALL) {
		string errorMsg("Divide by nearly zero! ");
		throw errorMsg;
	}
#endif // DEBUG
	float r = 1.0f / s;
	*this *= r;
	return *this;
}

// So sánh bằng
inline bool Vec3::operator==(const Vec3& v) const { return (x == v.x) && (y == v.y) && (z == v.z); }

// So sánh bằng một số
inline bool Vec3::operator==(const float s) const { return (x == s) && (y == s) && (z == s); }

// So sánh khác
inline bool Vec3::operator!=(const Vec3& v) const { return (x != v.x) || (y != v.y) || (z != v.z); }

// So sánh hơn
inline bool Vec3::operator<(const Vec3& v) const { return (x < v.x&& y < v.y&& z < v.z); }

// So sánh hơn bằng
inline bool Vec3::operator<=(const Vec3& v) const { return (x <= v.x && y <= v.y && z <= v.z); }

// So sánh ba thành phần của vector với một số
inline bool Vec3::operator<=(const float s) const { return (x <= s && y <= s && z <= s); }

// Tích vô hướng
inline float Vec3::operator*(const Vec3& v) const { return x * v.x + y * v.y + z * v.z; }

// Mảng vector
inline float Vec3::operator[](int index) const { return (&x)[index]; }

// Mảng vector
inline float& Vec3::operator[](int index) { return (&x)[index]; }

// Nhân 1 số với vector (cách ghi số trước)
inline Vec3 operator*(const float s, const Vec3& v) { return Vec3(s * v.x, s * v.y, s * v.z); }

// So sánh một số với ba thành phần vector
inline bool operator<=(const float s, const Vec3& v) { return (s <= v.x && s <= v.y && s <= v.z); }

// In vector
inline ostream& operator<<(ostream& os, const Vec3& vec) {
	os << "(" << vec.x << ", " << vec.y << ", " << vec.z << ")";
	return os;
}

/////////////////////////////////////////// Các hàm ///////////////////////////////////////////////////

// Lập vector
inline void Vec3::set(float x, float y, float z) { this->x = x; this->y = y; this->z = z; }

// Độ dài vector
inline float Vec3::length() const { return sqrtf(x * x + y * y + z * z); }

// Khoảng cách giữa 2 vector
inline float Vec3::distance(const Vec3& v) const {
	// căn bậc 2((x2-x1)^2 + (y2-y1)^2 + (z2-z1)^2)
	return sqrtf((v.x - x) * (v.x - x) + (v.y - y) * (v.y - y) + (v.z - z) * (v.z - z));
}

// Tích vô hướng
inline float Vec3::dot(const Vec3& b) const { return *this * b; }

// Tích vô hướng (cách ghi khác)
inline float Vec3::dot(const Vec3& v1, const Vec3& v2) { return v1 * v2; }

// Tích có hướng
inline Vec3 Vec3::cross(const Vec3& vector) const { return Vec3(y * vector.z - vector.y * z, z * vector.x - vector.z * x, x * vector.y - vector.x * y); }

// Góc giữa 2 vector (radian)
inline float Vec3::angleR(const Vec3& v) const {
	// cos(góc) = tích vô hướng/tích độ dài
	float l1 = this->length();
	float l2 = v.length();
	float d = this->dot(v);
	float angle = acosf(d / (l1 * l2));
	return angle;
}

// Góc giữa 2 vector (độ)
inline float Vec3::angleD(const Vec3& v) const {
	// cos(góc) = tích vô hướng/tích độ dài
	float l1 = this->length();
	float l2 = v.length();
	float d = this->dot(v);
	float angle = acosf(d / (l1 * l2)) / 3.141592f * 180.0f;
	return angle;
}

// Chuẩn hoá
inline Vec3& Vec3::normalize() {
	float m = this->length();
	x /= m;
	y /= m;
	z /= m;
	return *this;
}

// Vị trí trên vector theo thời gian
inline void Vec3::lerp(const Vec3& b, const float t) {
	x = (1 - t) * x + t * b.x;
	y = (1 - t) * y + t * b.y;
	z = (1 - t) * z + t * b.z;
}

// Xoay vector theo 1 góc
inline void Vec3::rotateZ(const float angle) {
	float x0 = x, y0 = y;
	x = x0 * cos(angle) - y0 * sin(angle);
	y = x0 * sin(angle) + y0 * cos(angle);
}

// Chiếu 1 vector lên 1 vector
inline Vec3 Vec3::project(const Vec3& vectorb) const {
	// Gọi vector C là hình chiếuc của vector B lên vector A, ta có
	// C = (b dot a) / (a.length)^2 * vector a
	Vec3 tem = *this;
	return tem * (tem.dot(vectorb) / (tem.length() * tem.length()));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////// VECTOR 4D ////////////////////////////////////////////////

//////////////////////////////////////// Overload phép toán //////////////////////////////////////////

// Phép gán vector
inline Vec4 Vec4::operator=(const Vec4& v) {
	set(v.x, v.y, v.z, v.w);
	return *this;
}

// Lấy vector đối
inline Vec4  Vec4::operator-() const { return Vec4(-x, -y, -z, -w); }

// Cộng 2 vector
inline Vec4  Vec4::operator+(const Vec4& v) const { return Vec4(x + v.x, y + v.y, z + v.z, w + v.w); }

// Trừ 2 vector
inline Vec4  Vec4::operator-(const Vec4& v) const { return Vec4(x - v.x, y - v.y, z - v.z, w - v.w); }

// Cộng vector với chính nó
inline Vec4& Vec4::operator+=(const Vec4& v) {
	x += v.x;
	y += v.y;
	z += v.z;
	w += v.w;
	return *this;
}

// Trừ vector với chính nó
inline Vec4& Vec4::operator-=(const Vec4& v) {
	x -= v.x;
	y -= v.y;
	z -= v.z;
	w -= v.w;
	return *this;
}

// Nhân vector với 1 số
inline Vec4  Vec4::operator*(const float s) const { return Vec4(x * s, y * s, z * s, w * s); }

// Nhân vector chính nó với 1 số
inline Vec4& Vec4::operator*=(const float s) {
	x *= s;
	y *= s;
	z *= s;
	w *= s;
	return *this;
}

// Chia vector với 1 số
inline Vec4 Vec4::operator/(const float s) const { return Vec4(x / s, y / s, z / s, z / s); }

// Chia 1 số với chính vector đó
inline Vec4& Vec4::operator/=(const float s) {
	x /= s;
	y /= s;
	z /= s;
	w /= s;
	return *this;
}

// So sánh bằng
inline bool  Vec4::operator==(const Vec4& v) const { return (x == v.x) && (y == v.y) && (z == v.z) && (w == v.w); }

// So sánh khác
inline bool Vec4::operator!=(const Vec4& v) const { return (x != v.x) || (y != v.y) || (z != v.z) || (w != v.w); }

// So sánh hơn
inline bool  Vec4::operator<(const Vec4& v) const { return (x < v.x&& y < v.y&& z < v.z&& w < v.w); }

// Tích vô hướng
inline float Vec4::operator*(const Vec4& v) const { return x * v.x + y * v.y + z * v.z + w * v.w; }

// Mảng vector
inline float Vec4::operator[](int index) const { return (&x)[index]; }

// Mảng vector
inline float& Vec4::operator[](int index) { return (&x)[index]; }

// Nhân 1 số với vector (cách ghi số trước)
inline Vec4 operator*(const float s, const Vec4 v) { return Vec4(s * v.x, s * v.y, s * v.z, s * v.w); }

// In vector
inline ostream& operator<<(ostream& os, const Vec4& v) {
	os << "(" << v.x << ", " << v.y << ", " << v.z << ", " << v.w << ")";
	return os;
}

/////////////////////////////////////////////// Các hàm //////////////////////////////////////////////////////

// Lập vector
inline void Vec4::set(float x, float y, float z, float w) { this->x = x; this->y = y; this->z = z; this->w = w; }

// Độ dài vector
inline float Vec4::length() const { return sqrt(x * x + y * y + z * z + w * w); }

// Khoảng cách giữa 2 vector
inline float Vec4::distance(const Vec4& v) const { return sqrtf((v.x - x) * (v.x - x) + (v.y - y) * (v.y - y) + (v.z - z) * (v.z - z) + (v.w - w) * (v.w - w)); }

// Chuẩn hoá vector
inline Vec4& Vec4::normalize() {
	float m = length();
	x /= m;
	y /= m;
	z /= m;
	return *this;
}

// Tích vô hướng
inline float Vec4::dot(const Vec4& v) const { return x * v.x + y * v.y + z * v.z + w * v.w; }

#endif // !VECTOR_H

