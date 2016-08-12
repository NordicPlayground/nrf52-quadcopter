// I2Cdev library collection - MPU9150 I2C device class, 9-axis MotionApps 4.1 implementation
// Based on InvenSense MPU-9150 register map document rev. 2.0, 5/19/2011 (RM-MPU-6000A-00)
// 6/18/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     ... - ongoing debug release

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

#ifndef _MPU9150_9AXIS_MOTIONAPPS41_H_
#define _MPU9150_9AXIS_MOTIONAPPS41_H_

#include <stdbool.h>
#include <stdint.h>
#include <helper_3dmath.h>

uint8_t mpu6500DmpInitialize(void);
bool mpu6500DmpPacketAvailable(void);
uint16_t mpu6500DmpGetFIFOPacketSize(void);
uint8_t mpu6500DmpGetQuaternion1(int32_t *data, const uint8_t* packet);
uint8_t mpu6500DmpGetQuaternion2(int16_t *data, const uint8_t* packet);
uint8_t mpu6500DmpGetQuaternion3(Quaternion *q, const uint8_t* packet);
uint8_t mpu6500DmpGetGravity(VectorFloat *v, Quaternion *q);
uint8_t mpu6500DmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity);

#endif /* _MPU9150_9AXIS_MOTIONAPPS41_H_ */
