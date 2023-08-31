/**
 * Quaternion.cpp v1.0.0 30/08/2023
 *
 * Copyright (c) 2023, Robert Eisele (raw.org)
 * Licensed under the MIT license.
 **/

#ifndef QUATERNION_H
#define QUATERNION_H

#ifdef ARDUINO
#include "Arduino.h"
#else
#include <math.h>
#endif

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

    /* @TODO
        union {
        struct {
            float x;
            float y;
            float z;
        };
        real_t vector[3] = { 0, 0, 0 };
    };
    */

    Quaternion() : w(1), x(0), y(0), z(0) {}
    Quaternion(const Quaternion &q) : w(q.w), x(q.x), y(q.y), z(q.z) {}
    Quaternion(float _x, float _y, float _z) : w(0), x(_x), y(_y), z(_z) {}
    Quaternion(float _w, float _x, float _y, float _z) : w(_w), x(_x), y(_y), z(_z) {}

    Quaternion &operator=(const Quaternion &rhs);
    Quaternion &operator+=(const Quaternion &q);
    Quaternion &operator-=(const Quaternion &q);
    Quaternion &operator*=(float scale);
    Quaternion &operator*=(const Quaternion &q);

    const Quaternion operator-() const { return Quaternion(-w, -x, -y, -z); }
    const Quaternion operator*(const Quaternion &q) const { return Quaternion(*this) *= q; }
    const Quaternion operator*(float scale) const { return Quaternion(w * scale, x * scale, y * scale, z * scale); }
    const Quaternion operator+(const Quaternion &q2) const
    {
        const Quaternion &q1 = *this;
        return Quaternion(q1.w + q2.w, q1.x + q2.x, q1.y + q2.y, q1.z + q2.z);
    }
    const Quaternion operator-(const Quaternion &q2) const
    {
        const Quaternion &q1 = *this;
        return Quaternion(q1.w - q2.w, q1.x - q2.x, q1.y - q2.y, q1.z - q2.z);
    }

    float dot(const Quaternion &q) const;
    float norm() const;
    float normSq() const;
    Quaternion &normalize();

    const Quaternion conjugate() const;
    void rotateVector(float &vx, float &vy, float &vz);

    static const Quaternion fromEuler(float _x, float _y, float _z);
    static const Quaternion fromAxisAngle(float x, float y, float z, float angle);
};

#endif