# Quaternion.c - ℍ in C++

Quaternion.c is a well tested C++ library for 3D rotations. Quaternions are a compact representation of rotations and can be used everywhere, from the attitude of a dron over to computer games to the rotation of satellites and all by avoiding the [Gimbal lock](https://en.wikipedia.org/wiki/Gimbal_lock). The library is a port of [Quaternion.js](https://github.com/rawify/Quaternion.js)


# Examples

```cpp
#include <quaternion.h>

Quaternion x(1, 2, 3, 4);
x.normalize();
```

Converting Euler angles to a quaternion is as easy as 

```cpp
Quaternion q = Quaternion::fromAxisAngle(1, 0, 0, 0.4);
```

Operators
===

Quaternion q1 + q2
---
Adds two quaternions Q1 and Q2

Quaternion q1 - q2
---
Subtracts a quaternions Q2 from Q1

Quaternion -q
---
Calculates the additive inverse, or simply it negates the quaternion

Quaternion q1 * q2
---
Calculates the Hamilton product of two quaternions. **Note:** This function is not commutative, i.e. order matters!

Quaternion q1 * scale
---
Scales a quaternion by a scalar

Functions
===

Quaternion norm()
---
Calculates the length/modulus/magnitude or the norm of a quaternion

Quaternion normSq()
---
Calculates the squared length/modulus/magnitude or the norm of a quaternion

Quaternion normalize()
---
Normalizes the quaternion to have |Q| = 1 as long as the norm is not zero. Alternative names are the signum, unit or versor

Quaternion dot()
---
Calculates the dot product of two quaternions

Quaternion conjugate()
---
Calculates the conjugate of a quaternion. If the quaternion is normalized, the conjugate is the inverse of the quaternion - but faster.

rotateVector(&x, &y, &z)
---
Rotates a 3 component vector, represented as three arguments passed as reference

Quaternion::fromAxisAngle(x, y, z, angle)
---
Gets a quaternion by a rotation given as an axis (x, y, z) and angle


Quaternion::fromEuler(ϕ, θ, ψ)
---
Euler angles are probably the reason to use quaternions. The definition is mostly sloppy and you can only decide how it was defined based on the matrix representation. A `ZXY` in one definition is the multiplication order read from right to left and in another the execution order from left to right.

So for `fromEuler(ϕ, θ, ψ)` with `QUATERNION_EULER_ZYX`, for example means first rotate around Y by ψ then around X by θ and then around Z by ϕ (`RotZ(ϕ)RotX(θ)RotY(ψ)`).

The order of `fromEuler()` can be defined at compile time by defining `QUATERNION_EULER_ORDER`. The value can be `ZXY, XYZ / RPY, YXZ, ZYX / YPR (default), YZX, XZY, ZYZ, ZXZ, YXY, YZY, XYX, XZX`, using the constants such as `QUATERNION_EULER_ZYX`.

Copyright and licensing
===
Copyright (c) 2023, [Robert Eisele](https://raw.org/)
Licensed under the MIT license.

