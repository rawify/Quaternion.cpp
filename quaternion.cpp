/**
 * Quaternion.cpp v1.0.0 30/08/2023
 *
 * Copyright (c) 2023, Robert Eisele (raw.org)
 * Licensed under the MIT license.
 **/

#include "quaternion.h"

/**
 * Assigns a quaternion to the current quaternion
 */
Quaternion &Quaternion::operator=(const Quaternion &q)
{
	w = q.w;
	x = q.x;
	y = q.y;
	z = q.z;
	return *this;
}

/**
 * Adds two quaternions Q1 and Q2
 */
Quaternion &Quaternion::operator+=(const Quaternion &q)
{
	w += q.w;
	x += q.x;
	y += q.y;
	z += q.z;
	return *this;
}

/**
 * Subtracts a quaternions Q2 from Q1
 */
Quaternion &Quaternion::operator-=(const Quaternion &q)
{
	w -= q.w;
	x -= q.x;
	y -= q.y;
	z -= q.z;
	return *this;
}

/**
 * Scales a quaternion by a scalar
 */
Quaternion &Quaternion::operator*=(float scale)
{
	w *= scale;
	x *= scale;
	y *= scale;
	z *= scale;
	return *this;
}

/**
 * Calculates the Hamilton product of two quaternions
 */
Quaternion &Quaternion::operator*=(const Quaternion &q)
{
	float w1 = w;
	float x1 = x;
	float y1 = y;
	float z1 = z;

	float w2 = q.w;
	float x2 = q.x;
	float y2 = q.y;
	float z2 = q.z;

	w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;
	x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
	y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2;
	z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2;
	return *this;
}

/**
 * Calculates the dot product of two quaternions
 */
float Quaternion::dot(const Quaternion &q) const
{
	return w * q.w + x * q.x + y * q.y + z * q.z;
}

/**
 * Calculates the length/modulus/magnitude or the norm of a quaternion
 */
float Quaternion::norm() const
{
	return sqrtf(w * w + x * x + y * y + z * z);
}

/**
 * Calculates the squared length/modulus/magnitude or the norm of a quaternion
 */
float Quaternion::normSq() const
{
	return w * w + x * x + y * y + z * z;
}

/**
 * Normalizes the quaternion to have |Q| = 1 as long as the norm is not zero
 */
Quaternion &Quaternion::normalize()
{
	float iLen = 1 / norm();
	w *= iLen;
	x *= iLen;
	y *= iLen;
	z *= iLen;
	return *this;
}

/**
 * Calculates the conjugate of a quaternion
 */
const Quaternion Quaternion::conjugate() const
{
	return Quaternion(w, -x, -y, -z);
}

/**
 * Rotates a vector according to the current quaternion, assumes |q|=1
 *
 * @link https://raw.org/proof/vector-rotation-using-quaternions/
 */
void Quaternion::rotateVector(float &vx, float &vy, float &vz)
{
	// t = 2q x v
	float tx = 2. * (y * vz - z * vy);
	float ty = 2. * (z * vx - x * vz);
	float tz = 2. * (x * vy - y * vx);

	// v + w t + q x t
	vx = vx + w * tx + y * tz - z * ty;
	vy = vy + w * ty + z * tx - x * tz;
	vz = vz + w * tz + x * ty - y * tx;
}

/**
 * Creates a quaternion by a rotation given by Euler angles (multiplication order from right to left)
 *
 * If needed, define QUATERNION_EULER_ORDER for another order
 */
static const Quaternion fromEuler(float x, float y, float z)
{
	x = x * 0.5;
	y = y * 0.5;
	z = z * 0.5;

	float cX = cosf(x);
	float cY = cosf(y);
	float cZ = cosf(z);

	float sX = sinf(x);
	float sY = sinf(y);
	float sZ = sinf(z);

#if QUATERNION_EULER_ORDER == QUATERNION_EULER_ZXY
	// axisAngle([0, 0, 1], φ) * axisAngle([1, 0, 0], θ) * axisAngle([0, 1, 0], ψ)
	return Quaternion(
		cX * cY * cZ - sX * sY * sZ,
		sY * cX * cZ - sX * sZ * cY,
		sX * sY * cZ + sZ * cX * cY,
		sX * cY * cZ + sY * sZ * cX);
#elif QUATERNION_EULER_ORDER == QUATERNION_EULER_XYZ // roll around X, pitch around Y, yaw around Z
	// axisAngle([1, 0, 0], φ) * axisAngle([0, 1, 0], θ) * axisAngle([0, 0, 1], ψ)
	return Quaternion(
		cX * cY * cZ - sX * sY * sZ,
		sX * cY * cZ + sY * sZ * cX,
		sY * cX * cZ - sX * sZ * cY,
		sX * sY * cZ + sZ * cX * cY);
#elif QUATERNION_EULER_ORDER == QUATERNION_EULER_YXZ // deviceorientation
	// axisAngle([0, 1, 0], φ) * axisAngle([1, 0, 0], θ) * axisAngle([0, 0, 1], ψ)
	return Quaternion(
		sX * sY * sZ + cX * cY * cZ,
		sX * sZ * cY + sY * cX * cZ,
		sX * cY * cZ - sY * sZ * cX,
		sZ * cX * cY - sX * sY * cZ);
#elif QUATERNION_EULER_ORDER == QUATERNION_EULER_ZYX
	// axisAngle([0, 0, 1], φ) * axisAngle([0, 1, 0], θ) * axisAngle([1, 0, 0], ψ)
	return Quaternion(
		sX * sY * sZ + cX * cY * cZ,
		sZ * cX * cY - sX * sY * cZ,
		sX * sZ * cY + sY * cX * cZ,
		sX * cY * cZ - sY * sZ * cX);
#elif QUATERNION_EULER_ORDER == QUATERNION_EULER_YZX
	// axisAngle([0, 1, 0], φ) * axisAngle([0, 0, 1], θ) * axisAngle([1, 0, 0], ψ)
	return Quaternion(
		cX * cY * cZ - sX * sY * sZ,
		sX * sY * cZ + sZ * cX * cY,
		sX * cY * cZ + sY * sZ * cX,
		sY * cX * cZ - sX * sZ * cY);
#elif QUATERNION_EULER_ORDER == QUATERNION_EULER_XZY
	// axisAngle([1, 0, 0], φ) * axisAngle([0, 0, 1], θ) * axisAngle([0, 1, 0], ψ)
	return Quaternion(
		sX * sY * sZ + cX * cY * cZ,
		sX * cY * cZ - sY * sZ * cX,
		sZ * cX * cY - sX * sY * cZ,
		sX * sZ * cY + sY * cX * cZ);
#elif QUATERNION_EULER_ORDER == QUATERNION_EULER_ZYZ
	// axisAngle([0, 0, 1], φ) * axisAngle([0, 1, 0], θ) * axisAngle([0, 0, 1], ψ)
	return Quaternion(
		cX * cY * cZ - sX * sZ * cY,
		sY * sZ * cX - sX * sY * cZ,
		sX * sY * sZ + sY * cX * cZ,
		sX * cY * cZ + sZ * cX * cY);
#elif QUATERNION_EULER_ORDER == QUATERNION_EULER_ZXZ
	// axisAngle([0, 0, 1], φ) * axisAngle([1, 0, 0], θ) * axisAngle([0, 0, 1], ψ)
	return Quaternion(
		cX * cY * cZ - sX * sZ * cY,
		sX * sY * sZ + sY * cX * cZ,
		sX * sY * cZ - sY * sZ * cX,
		sX * cY * cZ + sZ * cX * cY);
#elif QUATERNION_EULER_ORDER == QUATERNION_EULER_YXY
	// axisAngle([0, 1, 0], φ) * axisAngle([1, 0, 0], θ) * axisAngle([0, 1, 0], ψ)
	return Quaternion(
		cX * cY * cZ - sX * sZ * cY,
		sX * sY * sZ + sY * cX * cZ,
		sX * cY * cZ + sZ * cX * cY,
		sY * sZ * cX - sX * sY * cZ);
#elif QUATERNION_EULER_ORDER == QUATERNION_EULER_YZY
	// axisAngle([0, 1, 0], φ) * axisAngle([0, 0, 1], θ) * axisAngle([0, 1, 0], ψ)
	return Quaternion(
		cX * cY * cZ - sX * sZ * cY,
		sX * sY * cZ - sY * sZ * cX,
		sX * cY * cZ + sZ * cX * cY,
		sX * sY * sZ + sY * cX * cZ);
#elif QUATERNION_EULER_ORDER == QUATERNION_EULER_XYX
	// axisAngle([1, 0, 0], φ) * axisAngle([0, 1, 0], θ) * axisAngle([1, 0, 0], ψ)
	return Quaternion(
		cX * cY * cZ - sX * sZ * cY,
		sX * cY * cZ + sZ * cX * cY,
		sX * sY * sZ + sY * cX * cZ,
		sX * sY * cZ - sY * sZ * cX);
#elif QUATERNION_EULER_ORDER == QUATERNION_EULER_XZX
	// axisAngle([1, 0, 0], φ) * axisAngle([0, 0, 1], θ) * axisAngle([1, 0, 0], ψ)
	return Quaternion(
		cX * cY * cZ - sX * sZ * cY,
		sX * cY * cZ + sZ * cX * cY,
		sY * sZ * cX - sX * sY * cZ,
		sX * sY * sZ + sY * cX * cZ);
#endif
}

/**
 * Creates quaternion by a rotation given as axis-angle orientation
 */
const Quaternion Quaternion::fromAxisAngle(float x, float y, float z, float angle)
{
	Quaternion ret;

	float halfAngle = angle * 0.5;

	float sin_2 = sinf(halfAngle);
	float cos_2 = cosf(halfAngle);

	float sin_norm = sin_2 / sqrtf(x * x + y * y + z * z);

	ret.w = cos_2;
	ret.x = x * sin_norm;
	ret.y = y * sin_norm;
	ret.z = z * sin_norm;

	return ret;
}
