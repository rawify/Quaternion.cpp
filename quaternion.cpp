/**
 * Quaternion.cpp v1.0.0 16/08/2025
 *
 * Copyright (c) 2025, Robert Eisele (raw.org)
 * Licensed under the MIT license.
 **/

#include "quaternion.h"

// ---------- methods ----------
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
	const float w1 = w, x1 = x, y1 = y, z1 = z;
	const float w2 = q.w, x2 = q.x, y2 = q.y, z2 = q.z;

	w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;
	x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
	y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2;
	z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2;
	return *this;
}

/**
 * Calculates the length/modulus/magnitude or the norm of a quaternion
 */
float Quaternion::norm() const
{
	return sqrtf(w * w + x * x + y * y + z * z);
}

/**
 * Normalizes the quaternion to have |Q| = 1 as long as the norm is not zero
 */
Quaternion &Quaternion::normalize()
{
	const float n2 = normSq();
	const float inv = 1.0f / sqrtf(n2);
	w *= inv;
	x *= inv;
	y *= inv;
	z *= inv;
	return *this;
}

/**
 * Rotates a vector according to the current quaternion, assumes |q|=1
 *
 * @link https://raw.org/proof/vector-rotation-using-quaternions/
 */
void Quaternion::rotateVector(float &vx, float &vy, float &vz) const
{
	// t = 2 cross(q.xyz, v)
	const float tx = 2.f * (y * vz - z * vy);
	const float ty = 2.f * (z * vx - x * vz);
	const float tz = 2.f * (x * vy - y * vx);

	// v + w t + cross(q.xyz, t)
	vx = vx + w * tx + y * tz - z * ty;
	vy = vy + w * ty + z * tx - x * tz;
	vz = vz + w * tz + x * ty - y * tx;
}

const Quaternion Quaternion::fromEuler(float x, float y, float z)
{
	x *= 0.5f;
	y *= 0.5f;
	z *= 0.5f;

	const float cX = cosf(x), sX = sinf(x);
	const float cY = cosf(y), sY = sinf(y);
	const float cZ = cosf(z), sZ = sinf(z);

#if QUATERNION_EULER_ORDER == QUATERNION_EULER_ZXY
	return Quaternion(
		cX * cY * cZ - sX * sY * sZ,
		sY * cX * cZ - sX * sZ * cY,
		sX * sY * cZ + sZ * cX * cY,
		sX * cY * cZ + sY * sZ * cX);
#elif QUATERNION_EULER_ORDER == QUATERNION_EULER_XYZ
	return Quaternion(
		cX * cY * cZ - sX * sY * sZ,
		sX * cY * cZ + sY * sZ * cX,
		sY * cX * cZ - sX * sZ * cY,
		sX * sY * cZ + sZ * cX * cY);
#elif QUATERNION_EULER_ORDER == QUATERNION_EULER_YXZ
	return Quaternion(
		sX * sY * sZ + cX * cY * cZ,
		sX * sZ * cY + sY * cX * cZ,
		sX * cY * cZ - sY * sZ * cX,
		sZ * cX * cY - sX * sY * cZ);
#elif QUATERNION_EULER_ORDER == QUATERNION_EULER_ZYX
	return Quaternion(
		sX * sY * sZ + cX * cY * cZ,
		sZ * cX * cY - sX * sY * cZ,
		sX * sZ * cY + sY * cX * cZ,
		sX * cY * cZ - sY * sZ * cX);
#elif QUATERNION_EULER_ORDER == QUATERNION_EULER_YZX
	return Quaternion(
		cX * cY * cZ - sX * sY * sZ,
		sX * sY * cZ + sZ * cX * cY,
		sX * cY * cZ + sY * sZ * cX,
		sY * cX * cZ - sX * sZ * cY);
#elif QUATERNION_EULER_ORDER == QUATERNION_EULER_XZY
	return Quaternion(
		sX * sY * sZ + cX * cY * cZ,
		sX * cY * cZ - sY * sZ * cX,
		sZ * cX * cY - sX * sY * cZ,
		sX * sZ * cY + sY * cX * cZ);
#elif QUATERNION_EULER_ORDER == QUATERNION_EULER_ZYZ
	return Quaternion(
		cX * cY * cZ - sX * sZ * cY,
		sY * sZ * cX - sX * sY * cZ,
		sX * sY * sZ + sY * cX * cZ,
		sX * cY * cZ + sZ * cX * cY);
#elif QUATERNION_EULER_ORDER == QUATERNION_EULER_ZXZ
	return Quaternion(
		cX * cY * cZ - sX * sZ * cY,
		sX * sY * sZ + sY * cX * cZ,
		sX * sY * cZ - sY * sZ * cX,
		sX * cY * cZ + sZ * cX * cY);
#elif QUATERNION_EULER_ORDER == QUATERNION_EULER_YXY
	return Quaternion(
		cX * cY * cZ - sX * sZ * cY,
		sX * sY * sZ + sY * cX * cZ,
		sX * cY * cZ + sZ * cX * cY,
		sY * sZ * cX - sX * sY * cZ);
#elif QUATERNION_EULER_ORDER == QUATERNION_EULER_YZY
	return Quaternion(
		cX * cY * cZ - sX * sZ * cY,
		sX * sY * cZ - sY * sZ * cX,
		sX * cY * cZ + sZ * cX * cY,
		sX * sY * sZ + sY * cX * cZ);
#elif QUATERNION_EULER_ORDER == QUATERNION_EULER_XYX
	return Quaternion(
		cX * cY * cZ - sX * sZ * cY,
		sX * cY * cZ + sZ * cX * cY,
		sX * sY * sZ + sY * cX * cZ,
		sX * sY * cZ - sY * sZ * cX);
#elif QUATERNION_EULER_ORDER == QUATERNION_EULER_XZX
	return Quaternion(
		cX * cY * cZ - sX * sZ * cY,
		sX * cY * cZ + sZ * cX * cY,
		sY * sZ * cX - sX * sY * cZ,
		sX * sY * sZ + sY * cX * cZ);
#else
#error "Unsupported QUATERNION_EULER_ORDER"
#endif
}

/**
 * Creates quaternion by a rotation given as axis-angle orientation
 */
const Quaternion Quaternion::fromAxisAngle(float x, float y, float z, float angle)
{
	const float half = angle * 0.5f;
	const float s = sinf(half), c = cosf(half);

	// Normalize axis safely (zero vector -> identity rotation)
	const float n2 = x * x + y * y + z * z;
	if (n2 <= 0.f)
	{
		return Quaternion(c, 0.f, 0.f, 0.f);
	}
	const float invLen = 1.0f / sqrtf(n2);
	return Quaternion(c, x * invLen * s, y * invLen * s, z * invLen * s);
}
