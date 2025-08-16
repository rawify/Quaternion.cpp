/**
 * Quaternion.cpp v1.0.0 16/08/2025
 *
 * Copyright (c) 2025, Robert Eisele (raw.org)
 * Licensed under the MIT license.
 **/

#ifndef QUATERNION_H
#define QUATERNION_H

#ifdef ARDUINO
#include "Arduino.h"
#else
#include <math.h>
#endif

// ---------- Tunables ----------
#ifndef QUATERNION_FORCE_INLINE
#if defined(_MSC_VER)
#define QUATERNION_FORCE_INLINE __forceinline
#elif defined(__GNUC__) || defined(__clang__)
#define QUATERNION_FORCE_INLINE __attribute__((always_inline)) inline
#else
#define QUATERNION_FORCE_INLINE inline
#endif
#endif

// ---------- Euler Orders ----------
#define QUATERNION_EULER_XYZ 1
#define QUATERNION_EULER_YXZ 2
#define QUATERNION_EULER_ZYX 3
#define QUATERNION_EULER_YZX 4
#define QUATERNION_EULER_XZY 5
#define QUATERNION_EULER_ZYZ 6
#define QUATERNION_EULER_ZXZ 7
#define QUATERNION_EULER_YXY 8
#define QUATERNION_EULER_YZY 9
#define QUATERNION_EULER_XYX 10
#define QUATERNION_EULER_XZX 11

#define QUATERNION_EULER_RPY QUATERNION_EULER_XYZ
#define QUATERNION_EULER_YPR QUATERNION_EULER_ZYX

// Default order that is used is ZYX
#ifndef QUATERNION_EULER_ORDER
#define QUATERNION_EULER_ORDER QUATERNION_EULER_ZYX
#endif

class Quaternion
{
public:
    float w;
    float x;
    float y;
    float z;

    QUATERNION_FORCE_INLINE Quaternion() : w(1.f), x(0.f), y(0.f), z(0.f) {}
    QUATERNION_FORCE_INLINE Quaternion(const Quaternion &q) : w(q.w), x(q.x), y(q.y), z(q.z) {}
    QUATERNION_FORCE_INLINE Quaternion(float _x, float _y, float _z) : w(0.f), x(_x), y(_y), z(_z) {}
    QUATERNION_FORCE_INLINE Quaternion(float _w, float _x, float _y, float _z) : w(_w), x(_x), y(_y), z(_z) {}

    Quaternion &operator=(const Quaternion &rhs);
    Quaternion &operator+=(const Quaternion &q);
    Quaternion &operator-=(const Quaternion &q);
    Quaternion &operator*=(float scale);
    Quaternion &operator*=(const Quaternion &q);

    QUATERNION_FORCE_INLINE const Quaternion operator-() const
    {
        return Quaternion(-w, -x, -y, -z);
    }

    QUATERNION_FORCE_INLINE const Quaternion operator*(const Quaternion &q) const
    {
        return Quaternion(*this) *= q;
    }

    QUATERNION_FORCE_INLINE const Quaternion operator*(float scale) const
    {
        return Quaternion(w * scale, x * scale, y * scale, z * scale);
    }

    QUATERNION_FORCE_INLINE const Quaternion operator+(const Quaternion &q2) const
    {
        return Quaternion(w + q2.w, x + q2.x, y + q2.y, z + q2.z);
    }

    QUATERNION_FORCE_INLINE const Quaternion operator-(const Quaternion &q2) const
    {
        return Quaternion(w - q2.w, x - q2.x, y - q2.y, z - q2.z);
    }

    QUATERNION_FORCE_INLINE bool operator==(const Quaternion &v) const
    {
        return (fabsf(w - v.w) < 0.01f &&
                fabsf(x - v.x) < 0.01f &&
                fabsf(y - v.y) < 0.01f &&
                fabsf(z - v.z) < 0.01f);
    }
    QUATERNION_FORCE_INLINE bool operator!=(const Quaternion &v) const
    {
        return !(*this == v);
    }

    QUATERNION_FORCE_INLINE float dot(const Quaternion &q) const
    {
        return w * q.w + x * q.x + y * q.y + z * q.z;
    }

    QUATERNION_FORCE_INLINE float normSq() const
    {
        return w * w + x * x + y * y + z * z;
    }

    float norm() const;
    Quaternion &normalize(); // no-op on zero

    QUATERNION_FORCE_INLINE const Quaternion conjugate() const
    {
        return Quaternion(w, -x, -y, -z);
    }
    void rotateVector(float &vx, float &vy, float &vz) const; // assumes |q|=1

    static const Quaternion fromEuler(float _x, float _y, float _z);
    static const Quaternion fromAxisAngle(float x, float y, float z, float angle);
};

#endif // QUATERNION_H
