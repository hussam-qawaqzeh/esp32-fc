// ============================================================================
// I2Cdev device library code is placed under the MIT license
// Copyright (c) 2011 Jeff Rowberg
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
// ============================================================================

#ifndef _HELPER_3DMATH_H_
#define _HELPER_3DMATH_H_

#include <math.h>

class Quaternion {
    public:
        float w, x, y, z;

        Quaternion() {
            w = 1.0f;
            x = 0.0f;
            y = 0.0f;
            z = 0.0f;
        }

        Quaternion(float nw, float nx, float ny, float nz) {
            w = nw;
            x = nx;
            y = ny;
            z = nz;
        }

        Quaternion getProduct(Quaternion q) {
            // Quaternion multiplication formula:
            // (w1, x1, y1, z1) * (w2, x2, y2, z2) =
            // (w1*w2 - x1*x2 - y1*y2 - z1*z2,
            //  w1*x2 + x1*w2 + y1*z2 - z1*y2,
            //  w1*y2 - x1*z2 + y1*w2 + z1*x2,
            //  w1*z2 + x1*y2 - y1*x2 + z1*w2)
            return Quaternion(
                w*q.w - x*q.x - y*q.y - z*q.z,
                w*q.x + x*q.w + y*q.z - z*q.y,
                w*q.y - x*q.z + y*q.w + z*q.x,
                w*q.z + x*q.y - y*q.x + z*q.w
            );
        }

        Quaternion getConjugate() {
            return Quaternion(w, -x, -y, -z);
        }

        float getMagnitude() {
            return sqrt(w*w + x*x + y*y + z*z);
        }

        void normalize() {
            float m = getMagnitude();
            if (m > 0) {
                w /= m;
                x /= m;
                y /= m;
                z /= m;
            }
        }

        Quaternion getNormalized() {
            Quaternion r(w, x, y, z);
            r.normalize();
            return r;
        }

        void fromAxisAngle(float ax, float ay, float az, float aAngle) {
            float halfAngle = aAngle / 2;
            float s = sin(halfAngle);
            w = cos(halfAngle);
            x = ax * s;
            y = ay * s;
            z = az * s;
        }

        void fromMatrix(float m0, float m1, float m2, float m3, float m4, float m5, float m6, float m7, float m8) {
            float trace = m0 + m4 + m8;
            if (trace > 0) {
                float s = 0.5f / sqrtf(trace + 1.0f);
                w = 0.25f / s;
                x = (m7 - m5) * s;
                y = (m2 - m6) * s;
                z = (m3 - m1) * s;
            } else if (m0 > m4 && m0 > m8) {
                float s = 2.0f * sqrtf(1.0f + m0 - m4 - m8);
                w = (m7 - m5) / s;
                x = 0.25f * s;
                y = (m1 + m3) / s;
                z = (m2 + m6) / s;
            } else if (m4 > m8) {
                float s = 2.0f * sqrtf(1.0f + m4 - m0 - m8);
                w = (m2 - m6) / s;
                x = (m1 + m3) / s;
                y = 0.25f * s;
                z = (m5 + m7) / s;
            } else {
                float s = 2.0f * sqrtf(1.0f + m8 - m0 - m4);
                w = (m3 - m1) / s;
                x = (m2 + m6) / s;
                y = (m5 + m7) / s;
                z = 0.25f * s;
            }
        }
};

class VectorBase {
    public:
        float x, y, z;

        VectorBase() {
            x = 0;
            y = 0;
            z = 0;
        }

        VectorBase(float nx, float ny, float nz) {
            x = nx;
            y = ny;
            z = nz;
        }

        void set(float nx, float ny, float nz) {
            x = nx;
            y = ny;
            z = nz;
        }

        void set(VectorBase *v) {
            x = v->x;
            y = v->y;
            z = v->z;
        }

        float getMagnitude() {
            return sqrt(x*x + y*y + z*z);
        }

        void normalize() {
            float m = getMagnitude();
            if (m > 0) {
                x /= m;
                y /= m;
                z /= m;
            }
        }

        VectorBase getNormalized() {
            VectorBase r(x, y, z);
            r.normalize();
            return r;
        }

        void rotate(Quaternion *q) {
            // http://cache.industry.siemens.com/dl/files/359/109359359/att_865051/V3_DSPC_Quaternion.pdf
            Quaternion p(0, x, y, z);
            Quaternion qInv = q->getConjugate();
            Quaternion pRotated = q->getProduct(p).getProduct(qInv);
            x = pRotated.x;
            y = pRotated.y;
            z = pRotated.z;
        }

        VectorBase getRotated(Quaternion *q) {
            VectorBase r(x, y, z);
            r.rotate(q);
            return r;
        }
};

class Vector3 : public VectorBase {
    public:
        Vector3() : VectorBase() {
        }

        Vector3(float nx, float ny, float nz) : VectorBase(nx, ny, nz) {
        }

        float getDotProduct(VectorBase *v) {
            return x*v->x + y*v->y + z*v->z;
        }

        void getCrossProduct(VectorBase *v) {
            float _x = y*v->z - z*v->y;
            float _y = z*v->x - x*v->z;
            float _z = x*v->y - y*v->x;
            x = _x;
            y = _y;
            z = _z;
        }

        VectorBase getCrossProduct(VectorBase v) {
            return VectorBase(
                y*v.z - z*v.y,
                z*v.x - x*v.z,
                x*v.y - y*v.x
            );
        }
};

typedef float RotationMatrix[3][3];
typedef float RotationMatrixFloat[3][3];

#endif /* _HELPER_3DMATH_H_ */
