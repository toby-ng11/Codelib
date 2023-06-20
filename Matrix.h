#ifndef MATRIX_H
#define MATRIX_H

#include <iostream>
#include <iomanip>
#include <math.h>
#include <algorithm>
#include "Vector.h"

using namespace std;

class Matrix2
{
	// Ma trận 2 x 2
	// A = | 0   2 |
	//     | 1   3 |

private: float m[4];
public:
	Matrix2() { identity(); }                                      // create identity matrix
	Matrix2(const float a[4]) { set(a); }
	Matrix2(float a0, float a1, float a2, float a3) {
		set(a0, a1, a2, a3);
	}

	void set(const float a[4]) { m[0] = a[0]; m[1] = a[1]; m[2] = a[2]; m[3] = a[3]; }
	void set(float a0, float a1, float a2, float a3) { m[0] = a0;  m[1] = a1;  m[2] = a2;  m[3] = a3; }
	void setRow(int index, const float row[2]) { m[index] = row[0];  m[index + 2] = row[1]; }
	void setRow(int index, const Vec2& v) { m[index] = v.x;  m[index + 2] = v.y; }
	void setColumn(int index, const float col[2]) { m[index * 2] = col[0];  m[index * 2 + 1] = col[1]; }
	void setColumn(int index, const Vec2& v) { m[index * 2] = v.x;  m[index * 2 + 1] = v.y; }

	const float* get() const;
	float       getDeterminant() const;
	float       getAngle() const;                       // retrieve angle (degree) from matrix

	Matrix2& identity();
	Matrix2& transpose();                            // transpose itself and return reference
	Matrix2& inverse();

	// operators
	Matrix2     operator+(const Matrix2& rhs) const;    // add rhs
	Matrix2     operator-(const Matrix2& rhs) const;    // subtract rhs
	Matrix2& operator+=(const Matrix2& rhs);         // add rhs and update this object
	Matrix2& operator-=(const Matrix2& rhs);         // subtract rhs and update this object
	Vec2        operator*(const Vec2& rhs) const;    // multiplication: v' = M * v
	Matrix2     operator*(const Matrix2& rhs) const;    // multiplication: M3 = M1 * M2
	Matrix2& operator*=(const Matrix2& rhs);         // multiplication: M1' = M1 * M2
	bool        operator==(const Matrix2& rhs) const;   // exact compare, no epsilon
	bool        operator!=(const Matrix2& rhs) const;   // exact compare, no epsilon
	float       operator[](int index) const;            // subscript operator v[0], v[1]
	float& operator[](int index);                  // subscript operator v[0], v[1]

	// friends functions
	friend Matrix2 operator-(const Matrix2& m);                     // unary operator (-)
	friend Matrix2 operator*(float scalar, const Matrix2& m);       // pre-multiplication
	friend Vec2 operator*(const Vec2& vec, const Matrix2& m); // pre-multiplication
	friend std::ostream& operator<<(std::ostream& os, const Matrix2& m);
};

class Matrix3 {
	/// 3x3 matrix - COLUMN MAJOR
	///	0	3	6
	///	1	4	7
	///	2	5	8

private:
	float  m[9];
	float tm[9];
public:
	// constructors
	Matrix3();  // init with identity
	Matrix3(const float src[9]);
	Matrix3(float m0, float m1, float m2,           // 1st column
		float m3, float m4, float m5,           // 2nd column
		float m6, float m7, float m8);          // 3rd column

	void        set(const float src[9]);
	void        set(float m0, float m1, float m2,   // 1st column
		float m3, float m4, float m5,   // 2nd column
		float m6, float m7, float m8);  // 3rd column
	void        setRow(int index, const float row[3]);
	void        setRow(int index, const Vec3& v);
	void        setColumn(int index, const float col[3]);
	void        setColumn(int index, const Vec3& v);

	const float* get() const;
	const float* getTranspose();
	Vec3     getRow(int index) const;
	Vec3     getColumn(int index) const;
	float    getDeterminant() const;
	Vec3     getAngle() const;                       // return (pitch, yaw, roll) in degree
	Matrix3& identity();
	Matrix3& transpose();                            // transpose itself and return reference
	Matrix3& inverse();

	// operators
	Matrix3     operator+(const Matrix3& rhs) const;    // add rhs
	Matrix3     operator-(const Matrix3& rhs) const;    // subtract rhs
	Matrix3& operator+=(const Matrix3& rhs);         // add rhs and update this object
	Matrix3& operator-=(const Matrix3& rhs);         // subtract rhs and update this object
	Vec3     operator*(const Vec3& rhs) const;    // multiplication: v' = M * v
	Matrix3     operator*(const Matrix3& rhs) const;    // multiplication: M3 = M1 * M2
	Matrix3& operator*=(const Matrix3& rhs);         // multiplication: M1' = M1 * M2
	bool        operator==(const Matrix3& rhs) const;   // exact compare, no epsilon
	bool        operator!=(const Matrix3& rhs) const;   // exact compare, no epsilon
	float       operator[](int index) const;            // subscript operator v[0], v[1]
	float& operator[](int index);                  // subscript operator v[0], v[1]

	// friends functions
	friend Matrix3 operator-(const Matrix3& m);                     // unary operator (-)
	friend Matrix3 operator*(float scalar, const Matrix3& m);       // pre-multiplication
	friend Vec3 operator*(const Vec3& vec, const Matrix3& m); // pre-multiplication
	friend std::ostream& operator<<(std::ostream& os, const Matrix3& m);

	/// These allow me convert from type Matrix to const float * without issues
	inline operator float* () { return static_cast<float*>(&m[0]); }
	inline operator const float* () const { return static_cast<const float*>(&m[0]); }
};

class Matrix4
{
	// Ma trận 4 x 4
	// A = | 0   4   8   12 |
	//     | 1   5   9   13 |
	//     | 2   6   10  14 |
	//     | 3   7   11  15 |
private:
	float m[16];
	float tm[16];
	float       getCofactor(float m0, float m1, float m2,
		float m3, float m4, float m5,
		float m6, float m7, float m8) const {
			{
				return m0 * (m4 * m8 - m5 * m7) -
					m1 * (m3 * m8 - m5 * m6) +
					m2 * (m3 * m7 - m4 * m6);
			}
	}
public:
	explicit      Matrix4();
	explicit      Matrix4(const float m_[16]);
	explicit      Matrix4(float x0, float x1, float x2, float x3,
		float y0, float y1, float y2, float y3,
		float z0, float z1, float z2, float z3,
		float w0, float w1, float w2, float w3);

	void           set(const float m_[16]);
	void           set(float x0, float x1, float x2, float x3,
		float y0, float y1, float y2, float y3,
		float z0, float z1, float z2, float z3,
		float w0, float w1, float w2, float w3);

	Vec4           getColumn(int index);
	Vec4           getRow(int index);
	const float* getTranspose();                        // return transposed matrix
	float          getDeterminant() const;
	Matrix3        getRotationMatrix() const;              // return 3x3 rotation part
	Vec3           getAngle() const;                       // return (pitch, yaw, roll)

	Matrix4& identity();
	Matrix4& transpose();
	Matrix4& inverse();
	Matrix4& inverseEuclidean();                      // inverse of Euclidean transform matrix
	Matrix4& inverseAffine();                         // inverse of affine transform matrix
	Matrix4& inverseProjective();                     // inverse of projective matrix using partitioning
	Matrix4& inverseGeneral();                        // inverse of generic matrix
	Matrix4        inverse(const Matrix4& m);               // Scott's inverse (same as generic)

	static Matrix4 rotate(const float degrees_, const float x_, const float y_, const float z_);
	static Matrix4 rotate(const float degrees_, const Vec3& axis_);

	static Matrix4 translate(const float x_, const float y_, const float z_);
	static Matrix4 translate(const Vec3& translate_);

	static Matrix4 scale(const float x_, const float y_, const float z_);
	static Matrix4 scale(const Vec3& scale);

	static Matrix4 perspective(const float fovy_, const float aspect_, const float zNear_, const float zFar_);
	static Matrix4 viewportNDC(const int width_, const int height_);
	static Matrix4 orthographic(const float xMin_, const float xMax_,
		const float yMin_, const float yMax_,
		const float zMin_, const float zMax_);
	static Matrix4 unOrtho(const Matrix4& ortho);
	static Matrix4 lookAt(const float eyeX_, const float eyeY_, const float eyeZ_,
		const float atX_, const float atY_, const float atZ_,
		const float upX_, const float upY_, const float upZ_);
	static Matrix4 lookAt(const Vec3& eye, const Vec3& at, const Vec3& up);

	Matrix4& operator=(const Matrix4& m_);
	Matrix4        operator*(const Matrix4& m_) const;
	Matrix4& operator*=(const Matrix4& m_);
	Vec4           operator* (const Vec4& v) const;
	Vec3           operator* (const Vec3& v) const;
	float    operator [] (int index) const;
	float& operator [] (int index);

	friend ostream& operator<<(ostream& os, const Matrix4& m);

	operator float* () { return static_cast<float*>(&m[0]); }
	operator const float* () const { return static_cast<const float*>(&m[0]); }
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////// MATRIX 2 


inline const float* Matrix2::get() const {
	return m;
}

// Tính định thức của ma trận 2x2 (det)
inline float Matrix2::getDeterminant() const {
	return m[0] * m[3] - m[1] * m[2];
}

// Lập ma trận 2x2 đơn vị
inline Matrix2& Matrix2::identity()
{
	m[0] = m[3] = 1.0f;
	m[1] = m[2] = 0.0f;
	return *this;
}

// Matrix2 + matrix2
inline Matrix2 Matrix2::operator+(const Matrix2& rhs) const
{
	return Matrix2(m[0] + rhs[0], m[1] + rhs[1], m[2] + rhs[2], m[3] + rhs[3]);
}

// Matrix2 - matrix2
inline Matrix2 Matrix2::operator-(const Matrix2& rhs) const
{
	return Matrix2(m[0] - rhs[0], m[1] - rhs[1], m[2] - rhs[2], m[3] - rhs[3]);
}

// Matrix2 cộng với chính nó
inline Matrix2& Matrix2::operator+=(const Matrix2& rhs)
{
	m[0] += rhs[0];  m[1] += rhs[1];  m[2] += rhs[2];  m[3] += rhs[3];
	return *this;
}

// Matrix2 trừ với chính nó
inline Matrix2& Matrix2::operator-=(const Matrix2& rhs)
{
	m[0] -= rhs[0];  m[1] -= rhs[1];  m[2] -= rhs[2];  m[3] -= rhs[3];
	return *this;
}

// Matrix2 x vector2
inline Vec2 Matrix2::operator*(const Vec2& rhs) const
{
	return Vec2(m[0] * rhs.x + m[2] * rhs.y, m[1] * rhs.x + m[3] * rhs.y);
}

// Matrix2 x matrix2
inline Matrix2 Matrix2::operator*(const Matrix2& rhs) const
{
	return Matrix2(m[0] * rhs[0] + m[2] * rhs[1], m[1] * rhs[0] + m[3] * rhs[1],
		m[0] * rhs[2] + m[2] * rhs[3], m[1] * rhs[2] + m[3] * rhs[3]);
}

// Matrix2 nhân với chính nó
inline Matrix2& Matrix2::operator*=(const Matrix2& rhs)
{
	*this = *this * rhs;
	return *this;
}

// So sánh bằng 2 matrix2
inline bool Matrix2::operator==(const Matrix2& rhs) const {
	return (m[0] == rhs[0]) && (m[1] == rhs[1]) && (m[2] == rhs[2]) && (m[3] == rhs[3]);
}

// So sánh khác 2 matrix2
inline bool Matrix2::operator!=(const Matrix2& rhs) const
{
	return (m[0] != rhs[0]) || (m[1] != rhs[1]) || (m[2] != rhs[2]) || (m[3] != rhs[3]);
}

// Iterator
inline float Matrix2::operator[](int index) const
{
	return m[index];
}

// Iterator
inline float& Matrix2::operator[](int index)
{
	return m[index];
}

// Lấy âm các phần từ
inline Matrix2 operator-(const Matrix2& rhs)
{
	return Matrix2(-rhs[0], -rhs[1], -rhs[2], -rhs[3]);
}

// Nhân vô hướng với 1 số
inline Matrix2 operator*(float s, const Matrix2& rhs)
{
	return Matrix2(s * rhs[0], s * rhs[1], s * rhs[2], s * rhs[3]);
}

// Nhân với vector2
inline Vec2 operator*(const Vec2& v, const Matrix2& rhs)
{
	return Vec2(v.x * rhs[0] + v.y * rhs[1], v.x * rhs[2] + v.y * rhs[3]);
}

// In ma trận2
inline std::ostream& operator<<(std::ostream& os, const Matrix2& m)
{
	os << std::fixed << std::setprecision(5);
	os << "[" << std::setw(10) << m[0] << " " << std::setw(10) << m[2] << "]\n"
		<< "[" << std::setw(10) << m[1] << " " << std::setw(10) << m[3] << "]\n";
	os << std::resetiosflags(std::ios_base::fixed | std::ios_base::floatfield);
	return os;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////// MATRIX 3 //////////////////////////////////////////////////////////

inline Matrix3::Matrix3()
{
	// initially identity matrix
	identity();
}

inline Matrix3::Matrix3(const float src[9])
{
	set(src);
}

inline Matrix3::Matrix3(float m0, float m1, float m2,
	float m3, float m4, float m5,
	float m6, float m7, float m8)
{
	set(m0, m1, m2, m3, m4, m5, m6, m7, m8);
}

inline void Matrix3::set(const float src[9])
{
	m[0] = src[0];  m[1] = src[1];  m[2] = src[2];
	m[3] = src[3];  m[4] = src[4];  m[5] = src[5];
	m[6] = src[6];  m[7] = src[7];  m[8] = src[8];
}

inline void Matrix3::set(float m0, float m1, float m2,
	float m3, float m4, float m5,
	float m6, float m7, float m8)
{
	m[0] = m0;  m[1] = m1;  m[2] = m2;
	m[3] = m3;  m[4] = m4;  m[5] = m5;
	m[6] = m6;  m[7] = m7;  m[8] = m8;
}

inline void Matrix3::setRow(int index, const float row[3])
{
	m[index] = row[0];  m[index + 3] = row[1];  m[index + 6] = row[2];
}

inline void Matrix3::setRow(int index, const Vec3& v)
{
	m[index] = v.x;  m[index + 3] = v.y;  m[index + 6] = v.z;
}

inline void Matrix3::setColumn(int index, const float col[3])
{
	m[index * 3] = col[0];  m[index * 3 + 1] = col[1];  m[index * 3 + 2] = col[2];
}

inline void Matrix3::setColumn(int index, const Vec3& v)
{
	m[index * 3] = v.x;  m[index * 3 + 1] = v.y;  m[index * 3 + 2] = v.z;
}

inline const float* Matrix3::get() const
{
	return m;
}

inline const float* Matrix3::getTranspose()
{
	tm[0] = m[0];   tm[1] = m[3];   tm[2] = m[6];
	tm[3] = m[1];   tm[4] = m[4];   tm[5] = m[7];
	tm[6] = m[2];   tm[7] = m[5];   tm[8] = m[8];
	return tm;
}

inline Vec3 Matrix3::getRow(int index) const
{
	return Vec3(m[index], m[index + 3], m[index + 6]);
}

inline Vec3 Matrix3::getColumn(int index) const
{
	return Vec3(m[index * 3], m[index * 3 + 1], m[index * 3 + 2]);
}

inline Matrix3& Matrix3::identity()
{
	m[0] = m[4] = m[8] = 1.0f;
	m[1] = m[2] = m[3] = m[5] = m[6] = m[7] = 0.0f;
	return *this;
}

inline Matrix3 Matrix3::operator+(const Matrix3& rhs) const
{
	return Matrix3(m[0] + rhs[0], m[1] + rhs[1], m[2] + rhs[2],
		m[3] + rhs[3], m[4] + rhs[4], m[5] + rhs[5],
		m[6] + rhs[6], m[7] + rhs[7], m[8] + rhs[8]);
}

inline Matrix3 Matrix3::operator-(const Matrix3& rhs) const
{
	return Matrix3(m[0] - rhs[0], m[1] - rhs[1], m[2] - rhs[2],
		m[3] - rhs[3], m[4] - rhs[4], m[5] - rhs[5],
		m[6] - rhs[6], m[7] - rhs[7], m[8] - rhs[8]);
}

inline Matrix3& Matrix3::operator+=(const Matrix3& rhs)
{
	m[0] += rhs[0];  m[1] += rhs[1];  m[2] += rhs[2];
	m[3] += rhs[3];  m[4] += rhs[4];  m[5] += rhs[5];
	m[6] += rhs[6];  m[7] += rhs[7];  m[8] += rhs[8];
	return *this;
}

inline Matrix3& Matrix3::operator-=(const Matrix3& rhs)
{
	m[0] -= rhs[0];  m[1] -= rhs[1];  m[2] -= rhs[2];
	m[3] -= rhs[3];  m[4] -= rhs[4];  m[5] -= rhs[5];
	m[6] -= rhs[6];  m[7] -= rhs[7];  m[8] -= rhs[8];
	return *this;
}

inline Vec3 Matrix3::operator*(const Vec3& rhs) const
{
	return Vec3(m[0] * rhs.x + m[3] * rhs.y + m[6] * rhs.z,
		m[1] * rhs.x + m[4] * rhs.y + m[7] * rhs.z,
		m[2] * rhs.x + m[5] * rhs.y + m[8] * rhs.z);
}

inline Matrix3 Matrix3::operator*(const Matrix3& rhs) const
{
	return Matrix3(m[0] * rhs[0] + m[3] * rhs[1] + m[6] * rhs[2], m[1] * rhs[0] + m[4] * rhs[1] + m[7] * rhs[2], m[2] * rhs[0] + m[5] * rhs[1] + m[8] * rhs[2],
		m[0] * rhs[3] + m[3] * rhs[4] + m[6] * rhs[5], m[1] * rhs[3] + m[4] * rhs[4] + m[7] * rhs[5], m[2] * rhs[3] + m[5] * rhs[4] + m[8] * rhs[5],
		m[0] * rhs[6] + m[3] * rhs[7] + m[6] * rhs[8], m[1] * rhs[6] + m[4] * rhs[7] + m[7] * rhs[8], m[2] * rhs[6] + m[5] * rhs[7] + m[8] * rhs[8]);
}

inline Matrix3& Matrix3::operator*=(const Matrix3& rhs)
{
	*this = *this * rhs;
	return *this;
}

inline bool Matrix3::operator==(const Matrix3& rhs) const
{
	return (m[0] == rhs[0]) && (m[1] == rhs[1]) && (m[2] == rhs[2]) &&
		(m[3] == rhs[3]) && (m[4] == rhs[4]) && (m[5] == rhs[5]) &&
		(m[6] == rhs[6]) && (m[7] == rhs[7]) && (m[8] == rhs[8]);
}

inline bool Matrix3::operator!=(const Matrix3& rhs) const
{
	return (m[0] != rhs[0]) || (m[1] != rhs[1]) || (m[2] != rhs[2]) ||
		(m[3] != rhs[3]) || (m[4] != rhs[4]) || (m[5] != rhs[5]) ||
		(m[6] != rhs[6]) || (m[7] != rhs[7]) || (m[8] != rhs[8]);
}

inline float Matrix3::operator[](int index) const
{
	return m[index];
}

inline float& Matrix3::operator[](int index)
{
	return m[index];
}

inline Matrix3 operator-(const Matrix3& rhs)
{
	return Matrix3(-rhs[0], -rhs[1], -rhs[2], -rhs[3], -rhs[4], -rhs[5], -rhs[6], -rhs[7], -rhs[8]);
}

inline Matrix3 operator*(float s, const Matrix3& rhs)
{
	return Matrix3(s * rhs[0], s * rhs[1], s * rhs[2], s * rhs[3], s * rhs[4], s * rhs[5], s * rhs[6], s * rhs[7], s * rhs[8]);
}

inline Vec3 operator*(const Vec3& v, const Matrix3& m)
{
	return Vec3(v.x * m[0] + v.y * m[1] + v.z * m[2], v.x * m[3] + v.y * m[4] + v.z * m[5], v.x * m[6] + v.y * m[7] + v.z * m[8]);
}

inline ostream& operator<<(ostream& os, const Matrix3& m)
{
	os << fixed << setprecision(5);
	os << "[" << setw(10) << m[0] << " " << setw(10) << m[3] << " " << setw(10) << m[6] << setw(5) << "]\n"
		<< "[" << setw(10) << m[1] << " " << setw(10) << m[4] << " " << setw(10) << m[7] << setw(5) << "]\n"
		<< "[" << setw(10) << m[2] << " " << setw(10) << m[5] << " " << setw(10) << m[8] << setw(5) << "]\n";
	os << std::resetiosflags(ios_base::fixed | ios_base::floatfield);
	return os;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////// MATRIX 4 //////////////////////////////////////////////////////////

// Constructor
inline Matrix4::Matrix4() {
	identity();
}

// Constructor
inline Matrix4::Matrix4(float x0, float x1, float x2, float x3,
	float y0, float y1, float y2, float y3,
	float z0, float z1, float z2, float z3,
	float w0, float w1, float w2, float w3)
{
	set(x0, x1, x2, x3, y0, y1, y2, y3, z0, z1, z2, z3, w0, w1, w2, w3);
}

// Constructor
inline Matrix4::Matrix4(const float m_[16]) {
	set(m_);
}

inline void Matrix4::set(const float m_[16]) {
	m[0] = m_[0];   m[1] = m_[1];   m[2] = m_[2];   m[3] = m_[3];
	m[4] = m_[4];   m[5] = m_[5];   m[6] = m_[6];   m[7] = m_[7];
	m[8] = m_[8];   m[9] = m_[9];   m[10] = m_[10]; m[11] = m_[11];
	m[12] = m_[12]; m[13] = m_[13]; m[14] = m_[14]; m[15] = m_[15];
}

inline void Matrix4::set(float x0, float x1, float x2, float x3,
	float y0, float y1, float y2, float y3,
	float z0, float z1, float z2, float z3,
	float w0, float w1, float w2, float w3)
{
	m[0] = x0; m[4] = y0; m[8] = z0;  m[12] = w0;
	m[1] = x1; m[5] = y1; m[9] = z1;  m[13] = w1;
	m[2] = x2; m[6] = y2; m[10] = z2; m[14] = w2;
	m[3] = x3; m[7] = y3; m[11] = z3; m[15] = w3;
}

// Lập ma trận 4x4 đơn vị
inline Matrix4& Matrix4::identity() {
	m[0] = m[5] = m[10] = m[15] = 1.0f;
	m[1] = m[2] = m[3] = m[4] = m[6] = m[7] = m[8] = m[9] = m[11] = m[12] = m[13] = m[14] = 0.0f;
	return *this;
}

inline Matrix4& Matrix4::transpose() {
	swap(m[1], m[4]);
	swap(m[2], m[8]);
	swap(m[3], m[12]);
	swap(m[6], m[9]);
	swap(m[7], m[13]);
	swap(m[11], m[14]);

	return *this;
}

// Trả về cột (ở dạng vector)
inline Vec4 Matrix4::getColumn(int index) { return Vec4(m[4 * index + 0], m[4 * index + 1], m[4 * index + 2], m[4 * index + 3]); }

// Trả về hàng (ở dạng vector)
inline Vec4 Matrix4::getRow(int index) { return Vec4(m[0 + index], m[4 + index], m[8 + index], m[12 + index]); }

// Ma trận chuyển vị
inline const float* Matrix4::getTranspose()
{
	tm[0] = m[0];   tm[1] = m[4];   tm[2] = m[8];   tm[3] = m[12];
	tm[4] = m[1];   tm[5] = m[5];   tm[6] = m[9];   tm[7] = m[13];
	tm[8] = m[2];   tm[9] = m[6];   tm[10] = m[10];  tm[11] = m[14];
	tm[12] = m[3];   tm[13] = m[7];   tm[14] = m[11];  tm[15] = m[15];
	return tm;
}

inline Matrix3 Matrix4::getRotationMatrix() const
{
	Matrix3 mat(m[0], m[1], m[2],
		m[4], m[5], m[6],
		m[8], m[9], m[10]);
	return mat;
}

// Gán ma trận
inline Matrix4& Matrix4::operator=(const Matrix4& m_) {
	this->m[0] = m_[0]; this->m[1] = m_[1]; this->m[2] = m_[2]; this->m[3] = m_[3];
	this->m[4] = m_[4]; this->m[5] = m_[5]; this->m[6] = m_[6]; this->m[7] = m_[7];
	this->m[8] = m_[8]; this->m[9] = m_[9]; this->m[10] = m_[10]; this->m[11] = m_[11];
	this->m[12] = m_[12]; this->m[13] = m_[13]; this->m[14] = m_[14]; this->m[15] = m_[15];
	return *this;
}

inline Matrix4 Matrix4::operator*(const Matrix4& m_) const {
	return Matrix4(m[0] * m_[0] + m[4] * m_[1] + m[8] * m_[2] + m[12] * m_[3],
		m[1] * m_[0] + m[5] * m_[1] + m[9] * m_[2] + m[13] * m_[3],
		m[2] * m_[0] + m[6] * m_[1] + m[10] * m_[2] + m[14] * m_[3],
		m[3] * m_[0] + m[7] * m_[1] + m[11] * m_[2] + m[15] * m_[3],
		m[0] * m_[4] + m[4] * m_[5] + m[8] * m_[6] + m[12] * m_[7],
		m[1] * m_[4] + m[5] * m_[5] + m[9] * m_[6] + m[13] * m_[7],
		m[2] * m_[4] + m[6] * m_[5] + m[10] * m_[6] + m[14] * m_[7],
		m[3] * m_[4] + m[7] * m_[5] + m[11] * m_[6] + m[15] * m_[7],
		m[0] * m_[8] + m[4] * m_[9] + m[8] * m_[10] + m[12] * m_[11],
		m[1] * m_[8] + m[5] * m_[9] + m[9] * m_[10] + m[13] * m_[11],
		m[2] * m_[8] + m[6] * m_[9] + m[10] * m_[10] + m[14] * m_[11],
		m[3] * m_[8] + m[7] * m_[9] + m[11] * m_[10] + m[15] * m_[11],
		m[0] * m_[12] + m[4] * m_[13] + m[8] * m_[14] + m[12] * m_[15],
		m[1] * m_[12] + m[5] * m_[13] + m[9] * m_[14] + m[13] * m_[15],
		m[2] * m_[12] + m[6] * m_[13] + m[10] * m_[14] + m[14] * m_[15],
		m[3] * m_[12] + m[7] * m_[13] + m[11] * m_[14] + m[15] * m_[15]);
}
inline Matrix4& Matrix4::operator*=(const Matrix4& m_) {
	*this = *this * m_;
	return *this;
}

// Nhân ma trận với vector 4D
inline Vec4 Matrix4::operator*(const Vec4& v) const {
	float x = v.x * m[0] + v.y * m[4] + v.z * m[8] + v.w * m[12];
	float y = v.x * m[1] + v.y * m[5] + v.z * m[9] + v.w * m[13];
	float z = v.x * m[2] + v.y * m[6] + v.z * m[10] + v.w * m[14];
	float w = v.x * m[3] + v.y * m[7] + v.z * m[11] + v.w * m[15];
	return Vec4(x, y, z, w);
}

// Nhân ma trận với vector 3D
inline Vec3 Matrix4::operator*(const Vec3& v) const {
	float x = v.x * m[0] + v.y * m[4] + v.z * m[8] + m[12];
	float y = v.x * m[1] + v.y * m[5] + v.z * m[9] + m[13];
	float z = v.x * m[2] + v.y * m[6] + v.z * m[10] + m[14];
	return Vec3(x, y, z);
}

inline float Matrix4::operator[](int index) const
{
	return m[index];
}

inline float& Matrix4::operator[](int index)
{
	return m[index];
}

inline ostream& operator<<(ostream& os, const Matrix4& m)
{
	os << fixed << setprecision(5);
	os << "[" << setw(10) << m[0] << " " << setw(10) << m[4] << " " << setw(10) << m[8] << " " << setw(10) << m[12] << setw(5) << "]\n"
		<< "[" << setw(10) << m[1] << " " << setw(10) << m[5] << " " << setw(10) << m[9] << " " << setw(10) << m[13] << setw(5) << "]\n"
		<< "[" << setw(10) << m[2] << " " << setw(10) << m[6] << " " << setw(10) << m[10] << " " << setw(10) << m[14] << setw(5) << "]\n"
		<< "[" << setw(10) << m[3] << " " << setw(10) << m[7] << " " << setw(10) << m[11] << " " << setw(10) << m[15] << setw(5) << "]\n";
	os << resetiosflags(ios_base::fixed | ios_base::floatfield);
	return os;
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////// MATRIX 2 //////////////////////////////////////////////////////////

#endif // !MATRIX_H
