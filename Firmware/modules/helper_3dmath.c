// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class, 3D math helper
// 6/5/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     2012-06-05 - add 3D math helper file to DMP6 example sketch

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

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

#include <helper_3dmath.h>
#include <stdint.h>
#include <math.h>

static Quaternion quat = {1.0f,0.0f,0.0f,0.0f};
static VectorInt16 vect = {0,0,0};
static VectorFloat vectf = {0.0f, 0.0f, 0.0f};

Quaternion getProduct(Quaternion q) {
	// Quaternion multiplication is defined by:
	//     (Q1 * Q2).w = (w1w2 - x1x2 - y1y2 - z1z2)
	//     (Q1 * Q2).x = (w1x2 + x1w2 + y1z2 - z1y2)
	//     (Q1 * Q2).y = (w1y2 - x1z2 + y1w2 + z1x2)
	//     (Q1 * Q2).z = (w1z2 + x1y2 - y1x2 + z1w2
	q.w = quat.w*q.w - quat.x*q.x - quat.y*q.y - quat.z*q.z;
	q.x = quat.w*q.x + quat.x*q.w + quat.y*q.z - quat.z*q.y;
	q.y = quat.w*q.y - quat.x*q.z + quat.y*q.w + quat.z*q.x;
	q.z = quat.w*q.z + quat.x*q.y - quat.y*q.x + quat.z*q.w;

	return q;
}

Quaternion getConjugate(void) {
	quat.w = quat.w;
	quat.x = -quat.x;
	quat.y = -quat.y;
	quat.z = -quat.z; 

	return quat;
}

float getQuaternionMagnitude(Quaternion *q) {
	return sqrt(q->w*q->w + q->x*q->x + q->y*q->y + q->z*q->z);
}

void normalizeQuaternion(Quaternion *q) {
	float m = getQuaternionMagnitude(q);
	q->w /= m;
	q->x /= m;
	q->y /= m;
	q->z /= m;
}

Quaternion getNormalizedQuaternion(void) {
	Quaternion r;
	r.w = quat.w;
	r.x = quat.x;
	r.y = quat.y;
	r.z = quat.z;
	normalizeQuaternion(&r);
	return r;
}
        
void VectorInt16Set(int16_t nx, int16_t ny, int16_t nz) {
	vect.x = nx;
	vect.y = ny;
	vect.z = nz;
}

float getVectorMagnitude(VectorInt16 *v) {
	return sqrt(v->x*v->x + v->y*v->y + v->z*v->z);
}

void normalizeVector(VectorInt16 *v) {
	float m = getVectorMagnitude(v);
	v->x /= m;
	v->y /= m;
	v->z /= m;
}

VectorInt16 getNormalizedVector(void) {
	VectorInt16 r;
	r.x = vect.x; 
	r.y = vect.y;
	r.z = vect.z;
	normalizeVector(&r);
	return r;
}

void rotate(Quaternion *q) {
	// http://www.cprogramming.com/tutorial/3d/quaternions.html
	// http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/index.htm
	// http://content.gpwiki.org/index.php/OpenGL:Tutorials:Using_Quaternions_to_represent_rotation
	// ^ or: http://webcache.googleusercontent.com/search?quat=cache:xgJAp3bDNhQJ:content.gpwiki.org/index.php/OpenGL:Tutorials:Using_Quaternions_to_represent_rotation&hl=en&gl=us&strip=1

	// P_out = quat * P_in * conj(quat)
	// - P_out is the output vector
	// - quat is the orientation quaternion
	// - P_in is the input vector (a*aReal)
	// - conj(quat) is the conjugate of the orientation quaternion (quat=[w,x,y,z], quat*=[w,-x,-y,-z])
	Quaternion p;
	p.w = 0;
	p.x = quat.x;
	p.y = quat.y;
	p.z = quat.z;

	// quaternion multiplication: quat * p, stored back in p
	p = getProduct(p);

	// quaternion multiplication: p * conj(quat), stored back in p
	p = getProduct(getConjugate());

	// p quaternion is now [0, x', y', z']
	quat.x = p.x;
	quat.y = p.y;
	quat.z = p.z;
}

VectorInt16 getRotated(Quaternion *quat) {
	VectorInt16 r;
	r.x = vect.x;
	r.y = vect.y;
	r.z = vect.z;
	rotate(quat);
	return r;
}

void VectorFloatSet(float nx, float ny, float nz) {
	vectf.x = nx;
	vectf.y = ny;
	vectf.z = nz;
}

float getMagnitudeVectorFloat(VectorFloat *vf) {
	return sqrt(vf->x*vf->x + vf->y*vf->y + vf->z*vf->z);
}

void normalizeVectorFloat(VectorFloat *vf) {
	float m = getMagnitudeVectorFloat(vf);
	vf->x /= m;
	vf->y /= m;
	vf->z /= m;
}

VectorFloat getNormalizedVectorFloat(void) {
	VectorFloat r;
	r.x = vectf.x;
	r.y = vectf.y; 
	r.z = vectf.z;
	normalizeVectorFloat(&r);
	return r;
}

void rotateVectorFloat(Quaternion *q) {
	Quaternion p;
	p.w = 0;
	p.x =	vectf.x;
	p.y =	vectf.y;
	p.z = vectf.z;

	// quaternion multiplication: quat * p, stored back in p
	p = getProduct(p);

	// quaternion multiplication: p * conj(quat), stored back in p
	p = getProduct(getConjugate());

	// p quaternion is now [0, x', y', z']
	vectf.x = p.x;
	vectf.y = p.y;
	vectf.z = p.z;
}

VectorFloat getRotatedVectorFloat(Quaternion *q) {
	VectorFloat r;
	r.x = vectf.x;
	r.y =	vectf.y;
	r.z = vectf.z;
	rotate(&quat);
	return r;
}
