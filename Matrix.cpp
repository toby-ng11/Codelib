#include "Matrix.h"

const float DEG2RAD = 3.141593f / 180.0f;
const float RAD2DEG = 180.0f / 3.141593f;
const float EPSILON = 0.00001f;

Vec3 Matrix4::getAngle() const
{
	float pitch, yaw, roll;         // 3 angles

	// find yaw (around y-axis) first
	// NOTE: asin() returns -90~+90, so correct the angle range -180~+180
	// using z value of forward vector
	yaw = RAD2DEG * asinf(m[8]);
	if (m[10] < 0)
	{
		if (yaw >= 0) yaw = 180.0f - yaw;
		else         yaw = -180.0f - yaw;
	}

	// find roll (around z-axis) and pitch (around x-axis)
	// if forward vector is (1,0,0) or (-1,0,0), then m[0]=m[4]=m[9]=m[10]=0
	if (m[0] > -EPSILON && m[0] < EPSILON)
	{
		roll = 0;  //@@ assume roll=0
		pitch = RAD2DEG * atan2f(m[1], m[5]);
	}
	else
	{
		roll = RAD2DEG * atan2f(-m[4], m[0]);
		pitch = RAD2DEG * atan2f(-m[9], m[10]);
	}

	return Vec3(pitch, yaw, roll);
}

Matrix4& Matrix4::inverse()
{
	{
		// If the 4th row is [0,0,0,1] then it is affine matrix and
		// it has no projective transformation.
		if (m[3] == 0 && m[7] == 0 && m[11] == 0 && m[15] == 1)
			this->inverseAffine();
		else
		{
			this->inverseGeneral();
			/*@@ invertProjective() is not optimized (slower than generic one)
			if(fabs(m[0]*m[5] - m[1]*m[4]) > EPSILON)
				this->invertProjective();   // inverse using matrix partition
			else
				this->invertGeneral();      // generalized inverse
			*/
		}

		return *this;
	}
}

Matrix4& Matrix4::inverseEuclidean()
{
	// transpose 3x3 rotation matrix part
	// | R^T | 0 |
	// | ----+-- |
	// |  0  | 1 |
	float tmp;
	tmp = m[1];  m[1] = m[4];  m[4] = tmp;
	tmp = m[2];  m[2] = m[8];  m[8] = tmp;
	tmp = m[6];  m[6] = m[9];  m[9] = tmp;

	// compute translation part -R^T * T
	// | 0 | -R^T x |
	// | --+------- |
	// | 0 |   0    |
	float x = m[12];
	float y = m[13];
	float z = m[14];
	m[12] = -(m[0] * x + m[4] * y + m[8] * z);
	m[13] = -(m[1] * x + m[5] * y + m[9] * z);
	m[14] = -(m[2] * x + m[6] * y + m[10] * z);

	// last row should be unchanged (0,0,0,1)

	return *this;
}


// Nghịch đảo ma trận affine
Matrix4& Matrix4::inverseAffine()
{
	// R^-1
	Matrix3 r(m[0], m[1], m[2], m[4], m[5], m[6], m[8], m[9], m[10]);
	r.inverse();
	m[0] = r[0];  m[1] = r[1];  m[2] = r[2];
	m[4] = r[3];  m[5] = r[4];  m[6] = r[5];
	m[8] = r[6];  m[9] = r[7];  m[10] = r[8];

	// -R^-1 * T
	float x = m[12];
	float y = m[13];
	float z = m[14];
	m[12] = -(r[0] * x + r[3] * y + r[6] * z);
	m[13] = -(r[1] * x + r[4] * y + r[7] * z);
	m[14] = -(r[2] * x + r[5] * y + r[8] * z);

	// last row should be unchanged (0,0,0,1)
	//m[3] = m[7] = m[11] = 0.0f;
	//m[15] = 1.0f;

	return *this;
}

Matrix4& Matrix4::inverseProjective()
{
	// partition
	Matrix2 a(m[0], m[1], m[4], m[5]);
	Matrix2 b(m[8], m[9], m[12], m[13]);
	Matrix2 c(m[2], m[3], m[6], m[7]);
	Matrix2 d(m[10], m[11], m[14], m[15]);

	// pre-compute repeated parts
	a.invert();             // A^-1
	Matrix2 ab = a * b;     // A^-1 * B
	Matrix2 ca = c * a;     // C * A^-1
	Matrix2 cab = ca * b;   // C * A^-1 * B
	Matrix2 dcab = d - cab; // D - C * A^-1 * B

	// check determinant if |D - C * A^-1 * B| = 0
	//NOTE: this function assumes det(A) is already checked. if |A|=0 then,
	//      cannot use this function.
	float determinant = dcab[0] * dcab[3] - dcab[1] * dcab[2];
	if (fabs(determinant) <= EPSILON)
	{
		return identity();
	}

	// compute D' and -D'
	Matrix2 d1 = dcab;      //  (D - C * A^-1 * B)
	d1.invert();            //  (D - C * A^-1 * B)^-1
	Matrix2 d2 = -d1;       // -(D - C * A^-1 * B)^-1

	// compute C'
	Matrix2 c1 = d2 * ca;   // -D' * (C * A^-1)

	// compute B'
	Matrix2 b1 = ab * d2;   // (A^-1 * B) * -D'

	// compute A'
	Matrix2 a1 = a - (ab * c1); // A^-1 - (A^-1 * B) * C'

	// assemble inverse matrix
	m[0] = a1[0];  m[4] = a1[2]; /*|*/ m[8] = b1[0];  m[12] = b1[2];
	m[1] = a1[1];  m[5] = a1[3]; /*|*/ m[9] = b1[1];  m[13] = b1[3];
	/*-----------------------------+-----------------------------*/
	m[2] = c1[0];  m[6] = c1[2]; /*|*/ m[10] = d1[0];  m[14] = d1[2];
	m[3] = c1[1];  m[7] = c1[3]; /*|*/ m[11] = d1[1];  m[15] = d1[3];

	return *this;
}

Matrix4& Matrix4::inverseGeneral()
{
	// get cofactors of minor matrices
	float cofactor0 = getCofactor(m[5], m[6], m[7], m[9], m[10], m[11], m[13], m[14], m[15]);
	float cofactor1 = getCofactor(m[4], m[6], m[7], m[8], m[10], m[11], m[12], m[14], m[15]);
	float cofactor2 = getCofactor(m[4], m[5], m[7], m[8], m[9], m[11], m[12], m[13], m[15]);
	float cofactor3 = getCofactor(m[4], m[5], m[6], m[8], m[9], m[10], m[12], m[13], m[14]);

	// get determinant
	float determinant = m[0] * cofactor0 - m[1] * cofactor1 + m[2] * cofactor2 - m[3] * cofactor3;
	if (fabs(determinant) <= EPSILON)
	{
		return identity();
	}

	// get rest of cofactors for adj(M)
	float cofactor4 = getCofactor(m[1], m[2], m[3], m[9], m[10], m[11], m[13], m[14], m[15]);
	float cofactor5 = getCofactor(m[0], m[2], m[3], m[8], m[10], m[11], m[12], m[14], m[15]);
	float cofactor6 = getCofactor(m[0], m[1], m[3], m[8], m[9], m[11], m[12], m[13], m[15]);
	float cofactor7 = getCofactor(m[0], m[1], m[2], m[8], m[9], m[10], m[12], m[13], m[14]);

	float cofactor8 = getCofactor(m[1], m[2], m[3], m[5], m[6], m[7], m[13], m[14], m[15]);
	float cofactor9 = getCofactor(m[0], m[2], m[3], m[4], m[6], m[7], m[12], m[14], m[15]);
	float cofactor10 = getCofactor(m[0], m[1], m[3], m[4], m[5], m[7], m[12], m[13], m[15]);
	float cofactor11 = getCofactor(m[0], m[1], m[2], m[4], m[5], m[6], m[12], m[13], m[14]);

	float cofactor12 = getCofactor(m[1], m[2], m[3], m[5], m[6], m[7], m[9], m[10], m[11]);
	float cofactor13 = getCofactor(m[0], m[2], m[3], m[4], m[6], m[7], m[8], m[10], m[11]);
	float cofactor14 = getCofactor(m[0], m[1], m[3], m[4], m[5], m[7], m[8], m[9], m[11]);
	float cofactor15 = getCofactor(m[0], m[1], m[2], m[4], m[5], m[6], m[8], m[9], m[10]);

	// build inverse matrix = adj(M) / det(M)
	// adjugate of M is the transpose of the cofactor matrix of M
	float invDeterminant = 1.0f / determinant;
	m[0] = invDeterminant * cofactor0;
	m[1] = -invDeterminant * cofactor4;
	m[2] = invDeterminant * cofactor8;
	m[3] = -invDeterminant * cofactor12;

	m[4] = -invDeterminant * cofactor1;
	m[5] = invDeterminant * cofactor5;
	m[6] = -invDeterminant * cofactor9;
	m[7] = invDeterminant * cofactor13;

	m[8] = invDeterminant * cofactor2;
	m[9] = -invDeterminant * cofactor6;
	m[10] = invDeterminant * cofactor10;
	m[11] = -invDeterminant * cofactor14;

	m[12] = -invDeterminant * cofactor3;
	m[13] = invDeterminant * cofactor7;
	m[14] = -invDeterminant * cofactor11;
	m[15] = invDeterminant * cofactor15;

	return *this;
}

Matrix4 Matrix4::inverse(const Matrix4& m)
{
	Matrix4 inverseM;
	float determinate;

	inverseM[0] = m[5] * m[10] * m[15] - m[5] * m[11] * m[14] - m[9] * m[6] * m[15] + m[9] * m[7] * m[14] + m[13] * m[6] * m[11] - m[13] * m[7] * m[10];
	inverseM[1] = -m[1] * m[10] * m[15] + m[1] * m[11] * m[14] + m[9] * m[2] * m[15] - m[9] * m[3] * m[14] - m[13] * m[2] * m[11] + m[13] * m[3] * m[10];
	inverseM[2] = m[1] * m[6] * m[15] - m[1] * m[7] * m[14] - m[5] * m[2] * m[15] + m[5] * m[3] * m[14] + m[13] * m[2] * m[7] - m[13] * m[3] * m[6];
	inverseM[3] = -m[1] * m[6] * m[11] + m[1] * m[7] * m[10] + m[5] * m[2] * m[11] - m[5] * m[3] * m[10] - m[9] * m[2] * m[7] + m[9] * m[3] * m[6];
	inverseM[4] = -m[4] * m[10] * m[15] + m[4] * m[11] * m[14] + m[8] * m[6] * m[15] - m[8] * m[7] * m[14] - m[12] * m[6] * m[11] + m[12] * m[7] * m[10];
	inverseM[5] = m[0] * m[10] * m[15] - m[0] * m[11] * m[14] - m[8] * m[2] * m[15] + m[8] * m[3] * m[14] + m[12] * m[2] * m[11] - m[12] * m[3] * m[10];
	inverseM[6] = -m[0] * m[6] * m[15] + m[0] * m[7] * m[14] + m[4] * m[2] * m[15] - m[4] * m[3] * m[14] - m[12] * m[2] * m[7] + m[12] * m[3] * m[6];
	inverseM[7] = m[0] * m[6] * m[11] - m[0] * m[7] * m[10] - m[4] * m[2] * m[11] + m[4] * m[3] * m[10] + m[8] * m[2] * m[7] - m[8] * m[3] * m[6];
	inverseM[8] = m[4] * m[9] * m[15] - m[4] * m[11] * m[13] - m[8] * m[5] * m[15] + m[8] * m[7] * m[13] + m[12] * m[5] * m[11] - m[12] * m[7] * m[9];
	inverseM[9] = -m[0] * m[9] * m[15] + m[0] * m[11] * m[13] + m[8] * m[1] * m[15] - m[8] * m[3] * m[13] - m[12] * m[1] * m[11] + m[12] * m[3] * m[9];
	inverseM[10] = m[0] * m[5] * m[15] - m[0] * m[7] * m[13] - m[4] * m[1] * m[15] + m[4] * m[3] * m[13] + m[12] * m[1] * m[7] - m[12] * m[3] * m[5];
	inverseM[11] = -m[0] * m[5] * m[11] + m[0] * m[7] * m[9] + m[4] * m[1] * m[11] - m[4] * m[3] * m[9] - m[8] * m[1] * m[7] + m[8] * m[3] * m[5];
	inverseM[12] = -m[4] * m[9] * m[14] + m[4] * m[10] * m[13] + m[8] * m[5] * m[14] - m[8] * m[6] * m[13] - m[12] * m[5] * m[10] + m[12] * m[6] * m[9];
	inverseM[13] = m[0] * m[9] * m[14] - m[0] * m[10] * m[13] - m[8] * m[1] * m[14] + m[8] * m[2] * m[13] + m[12] * m[1] * m[10] - m[12] * m[2] * m[9];
	inverseM[14] = -m[0] * m[5] * m[14] + m[0] * m[6] * m[13] + m[4] * m[1] * m[14] - m[4] * m[2] * m[13] - m[12] * m[1] * m[6] + m[12] * m[2] * m[5];
	inverseM[15] = m[0] * m[5] * m[10] - m[0] * m[6] * m[9] - m[4] * m[1] * m[10] + m[4] * m[2] * m[9] + m[8] * m[1] * m[6] - m[8] * m[2] * m[5];

	determinate = m[0] * inverseM[0] + m[1] * inverseM[4] + m[2] * inverseM[8] + m[3] * inverseM[12];

#ifdef _DEBUG  /// If in debug mode let's worry about divide by zero or nearly zero!!! 
	if (fabs(determinate) < VERY_SMALL) {
		std::string errorMsg("Divide by nearly zero in MMath::inverse!");
		throw errorMsg;
	}
#endif
	determinate = 1.0f / determinate;
	for (int i = 0; i < 16; i++) {
		inverseM[i] *= determinate;
	}
	return inverseM;
}


Matrix4 Matrix4::rotate(float degrees_, float x_, float y_, float z_)
{
	float cosang, sinang, cosm;
	Vec3 rotAxis(x_, y_, z_);
	rotAxis = rotAxis.normalize();
	degrees_ *= DEG2RAD;
	cosang = cos(degrees_);
	sinang = sin(degrees_);
	cosm = (1.0f - cosang);

	Matrix4 m;

	m[0] = (rotAxis.x * rotAxis.x * cosm) + cosang;
	m[1] = (rotAxis.x * rotAxis.y * cosm) + (rotAxis.z * sinang);
	m[2] = (rotAxis.x * rotAxis.z * cosm) - (rotAxis.y * sinang);
	m[3] = 0.0;
	m[4] = (rotAxis.x * rotAxis.y * cosm) - (rotAxis.z * sinang);
	m[5] = (rotAxis.y * rotAxis.y * cosm) + cosang;
	m[6] = (rotAxis.y * rotAxis.z * cosm) + (rotAxis.x * sinang);
	m[7] = 0.0;
	m[8] = (rotAxis.x * rotAxis.z * cosm) + (rotAxis.y * sinang);
	m[9] = (rotAxis.y * rotAxis.z * cosm) - (rotAxis.x * sinang);
	m[10] = (rotAxis.z * rotAxis.z * cosm) + cosang;
	m[11] = 0.0;
	m[12] = 0.0;
	m[13] = 0.0;
	m[14] = 0.0;
	m[15] = 1.0;
	return m;
}

Matrix4 Matrix4::rotate(const float degrees_, const Vec3& axis_)
{
	return rotate(degrees_, axis_.x, axis_.y, axis_.z);
}

Matrix4 Matrix4::translate(float x_, float y_, float z_)
{
	return Matrix4(1.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 1.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 1.0f, 0.0f,
		x_, y_, z_, 1.0f);
}

Matrix4 Matrix4::translate(const Vec3& translate_)
{
	return translate(translate_.x, translate_.y, translate_.z);
}

Matrix4 Matrix4::scale(const Vec3& scale) {
	return Matrix4::scale(scale.x, scale.y, scale.z);
}

Matrix4 Matrix4::scale(float x_, float y_, float z_) {
	return Matrix4(x_, 0.0f, 0.0f, 0.0f,
		0.0f, y_, 0.0f, 0.0f,
		0.0f, 0.0f, z_, 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f);
}

Matrix4 Matrix4::perspective(const float fovy_, const float aspect_, const float zNear_, const float zFar_) {
	float cot = 1.0f / tan(fovy_ * 0.5f * DEGREES_TO_RADIANS);
	/// Don't forget, this looks row centric but it really is a column matrix - right-hand rule rules
	Matrix4 result(cot / aspect_, 0.0f, 0.0f, 0.0f,
		0.0f, cot, 0.0f, 0.0f,
		0.0f, 0.0f, (zNear_ + zFar_) / (zNear_ - zFar_), -1.0,
		0.0, 0.0, (2.0f * zNear_ * zFar_) / (zNear_ - zFar_), 0.0);
	return result;
}

/// This creates a transform from Normalized Device Coordinates (NDC) to 
/// screen coordinates. OpenGL uses NDC as			 
///	              ------------------------------
///	             /|                           /|
///	            / |                          / |
///	           /  |                         /  |
///	          /   |                        /   |
///	         /    |                       /    |
///	        /     |                      /     |
///	       /      |                     /      |
///	      /       |                    /       |
///	     /        |                   /        |
///	    /         |                  /         |
///	   /----------------------------/ (1.0,1.0)|
///	   |          |                 |          |
///	   |          |                 |          |      +Y
///	   |          |                 |          | 
///	   |          |-----------------|----------|      ^
///	   |         /                  |         /       |
///	   |        /                   |        /        |       -Z
///	   |       /                    |       /         |
///	   |      /                     |      /          |     /
///	   |     /                      |     /           |    /
///	   |    /                       |    /            |   /
///	   |   /                        |   /             |  /
///	   |  /                         |  /              | /
///	   | /                          | /               |/
///	   |/ (-1.0,-1.0)               |/                ----------------> +X
///	   ------------------------------
Matrix4 Matrix4::viewportNDC(int width_, int height_) {
	float minZ = 0.0f;
	float maxZ = 1.0f;

	Matrix4 m;
	Matrix4 m1 = scale(1.0f, -1.0f, 1.0f);
	Matrix4 m2 = scale(float(width_) / 2.0f, float(height_) / 2.0f, maxZ - minZ);
	Matrix4 m3 = translate(float(width_) / 2.0f, float(height_) / 2.0f, minZ);
	m = m3 * m2 * m1;

	///This is the slightly faster way but who cares we do it rarely 
	/***
	m[0] = float(width_)/2.0f;
	m[5] = -float(height_)/2.0f;
	m[10] =  maxZ - minZ;
	m[12] = float(width_)/2.0f;
	m[13] = float(height_)/2.0f;
	m[14] = minZ;
	m[15] = 1.0f; ***/

	return m;
}

Matrix4 Matrix4::orthographic(float xMin_, float xMax_, float yMin_, float yMax_, float zMin_, float zMax_) {
	Matrix4 m;

	Matrix4 m1 = scale(2.0f / (xMax_ - xMin_), 2.0f / (yMax_ - yMin_), -2.0f / (zMax_ - zMin_));
	Matrix4 m2 = translate(-(xMax_ + xMin_) / (xMax_ - xMin_), -(yMax_ + yMin_) / (yMax_ - yMin_), -(zMax_ + zMin_) / (zMax_ - zMin_));
	m = m2 * m1;
	/***
	m[0] = 2.0f / (xMax - xMin);
	m[5] = 2.0f / (yMax - yMin);
	m[10] = -2.0f / (zMax - zMin);
	m[12] = -((xMax + xMin) / (xMax - xMin));
	m[13] = -((yMax + yMin) / (yMax - yMin));
	m[14] = -((zMax + zMin) / (zMax - zMin));
	m[15] = 1.0f;
	***/
	return m;
}

/// The orthographic projection matrix is linear and affine but nothing else so there is has no inverse
/// Therefore, it is labeled singular or non-invertable.
/// I would still like to back transform from screen space to world space though
/// Here's my unOrtho - It undoes what the orthographic matrix creates
/// Multiply screen coordinates by this matrix and you should get x and y world coordinates
Matrix4 Matrix4::unOrtho(const Matrix4& ortho) {
	Matrix4 m;
	m[0] = 1.0f / ortho[0];
	m[5] = 1.0f / ortho[5];
	m[10] = 1.0f / ortho[10];
	m[12] = -ortho[12] * m[0];
	m[13] = -ortho[13] * m[5];
	m[14] = -ortho[14] * m[10];
	m[15] = 1.0f;
	return m;
}

Matrix4 Matrix4::lookAt(float eyeX, float eyeY, float eyeZ,
	float atX, float atY, float atZ,
	float upX, float upY, float upZ) {

	Vec3 at(atX, atY, atZ);
	Vec3 up(upX, upY, upZ);
	Vec3 eye(eyeX, eyeY, eyeZ);

	Matrix4 result;

	Vec3 forward = (at - eye).normalize();
	up.normalize();
	Vec3 side = (forward.cross(up)).normalize();
	up = side.cross(forward);

	result[0] = side.x;
	result[1] = side.y;
	result[2] = side.z;
	result[3] = 0.0;

	result[4] = up.x;
	result[5] = up.y;
	result[6] = up.z;
	result[7] = 0.0;

	result[8] = -forward.x;
	result[9] = -forward.y;
	result[10] = -forward.z;
	result[11] = 0.0;

	result[12] = side.dot(eye);
	result[13] = up.dot(eye);
	result[14] = forward.dot(eye);
	result[15] = 1.0;

	return result;
}

Matrix4 Matrix4::lookAt(const Vec3& eye, const Vec3& at, const Vec3& up) {
	return lookAt(eye.x, eye.y, eye.z, at.x, at.y, at.z, up.x, up.y, up.z);
}



Matrix3& Matrix3::inverse()
{
	float determinant, invDeterminant;
	float tmp[9];

	tmp[0] = m[4] * m[8] - m[5] * m[7];
	tmp[1] = m[7] * m[2] - m[8] * m[1];
	tmp[2] = m[1] * m[5] - m[2] * m[4];
	tmp[3] = m[5] * m[6] - m[3] * m[8];
	tmp[4] = m[0] * m[8] - m[2] * m[6];
	tmp[5] = m[2] * m[3] - m[0] * m[5];
	tmp[6] = m[3] * m[7] - m[4] * m[6];
	tmp[7] = m[6] * m[1] - m[7] * m[0];
	tmp[8] = m[0] * m[4] - m[1] * m[3];

	// check determinant if it is 0
	determinant = m[0] * tmp[0] + m[1] * tmp[3] + m[2] * tmp[6];
	if (fabs(determinant) <= EPSILON)
	{
		return identity(); // cannot inverse, make it idenety matrix
	}

	// divide by the determinant
	invDeterminant = 1.0f / determinant;
	m[0] = invDeterminant * tmp[0];
	m[1] = invDeterminant * tmp[1];
	m[2] = invDeterminant * tmp[2];
	m[3] = invDeterminant * tmp[3];
	m[4] = invDeterminant * tmp[4];
	m[5] = invDeterminant * tmp[5];
	m[6] = invDeterminant * tmp[6];
	m[7] = invDeterminant * tmp[7];
	m[8] = invDeterminant * tmp[8];

	return *this;
}


Matrix2& Matrix2::invert()
{
	float determinant = getDeterminant();
	if (fabs(determinant) <= EPSILON)
	{
		return identity();
	}

	float tmp = m[0];   // copy the first element
	float invDeterminant = 1.0f / determinant;
	m[0] = invDeterminant * m[3];
	m[1] = -invDeterminant * m[1];
	m[2] = -invDeterminant * m[2];
	m[3] = invDeterminant * tmp;

	return *this;
}