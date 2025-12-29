// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class
// 10/7/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     2011-10-07 - initial release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#ifndef _HELPER_3DMATH_H_
#define _HELPER_3DMATH_H_

#include <math.h>

class Quaternion {
public:
    float w;
    float x;
    float y;
    float z;
    
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
        // Quaternion multiplication formula from Wikipedia equation (34)
        // (Q1 * Q2).w = (w1*w2 - x1*x2 - y1*y2 - z1*z2)
        // (Q1 * Q2).x = (w1*x2 + x1*w2 + y1*z2 - z1*y2)
        // (Q1 * Q2).y = (w1*y2 - x1*z2 + y1*w2 + z1*x2)
        // (Q1 * Q2).z = (w1*z2 + x1*y2 - y1*x2 + z1*w2
        return Quaternion(
            w*q.w - x*q.x - y*q.y - z*q.z,  // new w
            w*q.x + x*q.w + y*q.z - z*q.y,  // new x
            w*q.y - x*q.z + y*q.w + z*q.x,  // new y
            w*q.z + x*q.y - y*q.x + z*q.w); // new z
    }
    
    Quaternion getConjugate() {
        return Quaternion(w, -x, -y, -z);
    }
    
    float getMagnitude() {
        return sqrt(w*w + x*x + y*y + z*z);
    }
    
    void normalize() {
        float m = getMagnitude();
        w /= m;
        x /= m;
        y /= m;
        z /= m;
    }
    
    Quaternion getNormalized() {
        Quaternion r(w, x, y, z);
        r.normalize();
        return r;
    }
};

class VectorInt16 {
public:
    int16_t x;
    int16_t y;
    int16_t z;
    
    VectorInt16() {
        x = 0;
        y = 0;
        z = 0;
    }
    
    VectorInt16(int16_t nx, int16_t ny, int16_t nz) {
        x = nx;
        y = ny;
        z = nz;
    }
    
    VectorInt16 getProduct(float scalar) {
        return VectorInt16(x*scalar, y*scalar, z*scalar);
    }
    
    VectorInt16 getProduct(VectorInt16 v) {
        return VectorInt16(x*v.x, y*v.y, z*v.z);
    }
    
    VectorInt16 getSum(VectorInt16 v) {
        return VectorInt16(x+v.x, y+v.y, z+v.z);
    }
};

class VectorFloat {
public:
    float x;
    float y;
    float z;
    
    VectorFloat() {
        x = 0.0f;
        y = 0.0f;
        z = 0.0f;
    }
    
    VectorFloat(float nx, float ny, float nz) {
        x = nx;
        y = ny;
        z = nz;
    }
    
    float getMagnitude() {
        return sqrt(x*x + y*y + z*z);
    }
    
    void normalize() {
        float m = getMagnitude();
        x /= m;
        y /= m;
        z /= m;
    }
    
    VectorFloat getNormalized() {
        VectorFloat r(x, y, z);
        r.normalize();
        return r;
    }
    
    void rotate(Quaternion *q) {
        // http://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 40
        Quaternion qProd = q->getProduct(Quaternion(0, x, y, z));
        qProd = qProd.getProduct(q->getConjugate());
        x = qProd.x;
        y = qProd.y;
        z = qProd.z;
    }
    
    VectorFloat getRotated(Quaternion *q) {
        VectorFloat r(x, y, z);
        r.rotate(q);
        return r;
    }
};

template <typename T> class VectorBase {
public:
    T x;
    T y;
    T z;
    
    VectorBase() {
        x = 0;
        y = 0;
        z = 0;
    }
    
    VectorBase(T nx, T ny, T nz) {
        x = nx;
        y = ny;
        z = nz;
    }
    
    T getMagnitude() {
        return sqrt(x*x + y*y + z*z);
    }
    
    void normalize() {
        T m = getMagnitude();
        x /= m;
        y /= m;
        z /= m;
    }
    
    VectorBase getNormalized() {
        VectorBase r(x, y, z);
        r.normalize();
        return r;
    }
    
    void rotate(Quaternion *q) {
        Quaternion qProd = q->getProduct(Quaternion(0, x, y, z));
        qProd = qProd.getProduct(q->getConjugate());
        x = qProd.x;
        y = qProd.y;
        z = qProd.z;
    }
    
    VectorBase getRotated(Quaternion *q) {
        VectorBase r(x, y, z);
        r.rotate(q);
        return r;
    }
    
    void set(VectorBase v) {
        x = v.x;
        y = v.y;
        z = v.z;
    }
    
    void set(T nx, T ny, T nz) {
        x = nx;
        y = ny;
        z = nz;
    }
};

#endif /* _HELPER_3DMATH_H_ */
