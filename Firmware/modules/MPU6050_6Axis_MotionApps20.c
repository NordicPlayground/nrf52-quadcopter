// I2Cdev library collection - MPU6050 I2C device class, 6-axis MotionApps 2.0 implementation
// Based on InvenSense MPU-6050 register map document rev. 2.0, 5/19/2011 (RM-MPU-6000A-00)
// 5/20/2013 by Jeff Rowberg <jeff@rowberg.net>
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

// MotionApps 2.0 DMP implementation, built using the MPU-6050EVB evaluation board
#define MPU6500_INCLUDE_DMP_MOTIONAPPS20

#include "i2cdev.h"
#include "mpu6500.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "helper_3dmath.h"

#include "nrf_log.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "nrf_delay.h"

static uint8_t *dmpPacketBuffer;
static uint16_t dmpPacketSize;

uint8_t mpu6500DmpInitialize(void) {		
    // disable sleep mode
    mpu6500SetSleepEnabled(false);

    // get MPU hardware revision
    NRF_LOG_PRINTF("Selecting user bank 16...\r\n");
    mpu6500SetMemoryBank(0x10, true, true);
    NRF_LOG_PRINTF("Selecting memory byte 6...\r\n");
    mpu6500SetMemoryStartAddress(0x06);
    NRF_LOG_PRINTF("Checking hardware revision...\r\n");
    NRF_LOG_PRINTF("Revision @ user[16][6] = ");
    NRF_LOG_PRINTF("%02x\r\n", mpu6500ReadMemoryByte());
    NRF_LOG_PRINTF("Resetting memory bank selection to 0...\r\n");
    mpu6500SetMemoryBank(0, false, false);

    // check OTP bank valid
    //NRF_LOG_PRINTF("Reading OTP bank valid flag...\r\n");
    //NRF_LOG_PRINTF("OTP bank is ");
    //NRF_LOG_PRINTF("%02x\r\n",mpu6500GetOTPBankValid());

    NRF_LOG_PRINTF("Reading gyro offset values...\r\n");
    int16_t xgOffset = mpu6500GetXGyroOffsetUser();
    int16_t ygOffset = mpu6500GetYGyroOffsetUser();
    int16_t zgOffset = mpu6500GetZGyroOffsetUser();
    NRF_LOG_PRINTF("X gyro offset = ");
    NRF_LOG_PRINTF("%02x\r\n",xgOffset);
    NRF_LOG_PRINTF("Y gyro offset = ");
    NRF_LOG_PRINTF("%02x\r\n",ygOffset);
    NRF_LOG_PRINTF("Z gyro offset = ");
    NRF_LOG_PRINTF("%02x\r\n",zgOffset);

    // setup weird slave stuff (?)
    NRF_LOG_PRINTF("Setting slave 0 address to 0x7F...\r\n");
    mpu6500SetSlaveAddress(0, 0x7F);
    NRF_LOG_PRINTF("Disabling I2C Master mode...\r\n");
    mpu6500SetI2CMasterModeEnabled(false);
    NRF_LOG_PRINTF("Setting slave 0 address to 0x68 (self)...\r\n");
    mpu6500SetSlaveAddress(0, 0x68);
    NRF_LOG_PRINTF("Resetting I2C Master control...\r\n");
    mpu6500ResetI2CMaster();

    // load DMP code into memory banks
    NRF_LOG_PRINTF("Writing DMP code to MPU memory banks (%d bytes)\r\n",MPU6500_DMP_CODE_SIZE);
		
    if (mpu6500WriteProgMemoryBlock(dmpMemory, MPU6500_DMP_CODE_SIZE,0,0,true)) {
        NRF_LOG_PRINTF("Success! DMP code written and verified.\r\n");

        // write DMP configuration
        NRF_LOG_PRINTF("Writing DMP configuration to MPU memory banks (%d bytes in config def)\r\n",MPU6500_DMP_CONFIG_SIZE);
        if (mpu6500WriteProgDMPConfigurationSet(dmpConfig, MPU6500_DMP_CONFIG_SIZE)) {
            NRF_LOG_PRINTF("Success! DMP configuration written and verified.\r\n");

            NRF_LOG_PRINTF("Setting clock source to Z Gyro...\r\n");
            mpu6500SetClockSource(MPU6500_CLOCK_PLL_ZGYRO);

            NRF_LOG_PRINTF("Setting DMP and FIFO_OFLOW interrupts enabled...\r\n");
            mpu6500SetIntEnabled(0x12);

            NRF_LOG_PRINTF("Setting sample rate to 200Hz...\r\n");
            mpu6500SetRate(4); // 1khz / (1 + 4) = 200 Hz

            NRF_LOG_PRINTF("Setting external frame sync to TEMP_OUT_L[0]...\r\n");
            mpu6500SetExternalFrameSync(MPU6500_EXT_SYNC_TEMP_OUT_L);

            NRF_LOG_PRINTF("Setting DLPF bandwidth to 42Hz...\r\n");
            mpu6500SetDLPFMode(MPU6500_DLPF_BW_42);

            NRF_LOG_PRINTF("Setting gyro sensitivity to +/- 2000 deg/sec...\r\n");
            mpu6500SetFullScaleGyroRange(MPU6500_GYRO_FS_2000);

            NRF_LOG_PRINTF("Setting DMP configuration bytes (function unknown)...\r\n");
            mpu6500SetDMPConfig1(0x03);
            mpu6500SetDMPConfig2(0x00);

            //NRF_LOG_PRINTF("Clearing OTP Bank flag...\r\n");
            //mpu6500SetOTPBankValid(false);
/*
			NRF_LOG_PRINTF("Setting X/Y/Z gyro offset to previous values...\r\n");
            mpu6500SetXGyroOffsetUser(xgOffset);
            mpu6500SetYGyroOffsetUser(ygOffset);
            mpu6500SetZGyroOffsetUser(zgOffset);
*/
            NRF_LOG_PRINTF("Writing final memory update 1/7 (function unknown)...\r\n");
            uint8_t dmpUpdate[16], j;
            uint16_t pos = 0;
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = dmpUpdates[pos];
            mpu6500WriteMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1], true);

            NRF_LOG_PRINTF("Writing final memory update 2/7 (function unknown)...");
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = dmpUpdates[pos];
            mpu6500WriteMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1], true);

            NRF_LOG_PRINTF("Resetting FIFO...\r\n");
            mpu6500ResetFIFO();

            NRF_LOG_PRINTF("Reading FIFO count...\r\n");
            uint16_t fifoCount = mpu6500GetFIFOCount();
            uint8_t fifoBuffer[128];

            NRF_LOG_PRINTF("Current FIFO count=%d\r\n",fifoCount);
            mpu6500GetFIFOBytes(fifoBuffer, fifoCount);
						
						/*
            NRF_LOG_PRINTF("Setting motion detection threshold to 2...\r\n");
            mpu6500SetMotionDetectionThreshold(2);

            NRF_LOG_PRINTF("Setting zero-motion detection threshold to 156...\r\n");
            mpu6500SetZeroMotionDetectionThreshold(156);

            NRF_LOG_PRINTF("Setting motion detection duration to 80...\r\n");
            mpu6500SetMotionDetectionDuration(80);

            NRF_LOG_PRINTF("Setting zero-motion detection duration to 0...\r\n");
            mpu6500SetZeroMotionDetectionDuration(0);
						*/
						
            NRF_LOG_PRINTF("Resetting FIFO...\r\n");
            mpu6500ResetFIFO();

            NRF_LOG_PRINTF("Enabling FIFO...\r\n");
            mpu6500SetFIFOEnabled(true);

            NRF_LOG_PRINTF("Enabling DMP...\r\n");
            mpu6500SetDMPEnabled(true);

            NRF_LOG_PRINTF("Resetting DMP...\r\n");
            mpu6500ResetDMP();

            NRF_LOG_PRINTF("Writing final memory update 3/7 (function unknown)...\r\n");
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = dmpUpdates[pos];
            mpu6500WriteMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1], true);

            NRF_LOG_PRINTF("Writing final memory update 4/7 (function unknown)...\r\n");
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = dmpUpdates[pos];
            mpu6500WriteMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1], true);

            NRF_LOG_PRINTF("Writing final memory update 5/7 (function unknown)...\r\n");
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = dmpUpdates[pos];
            mpu6500WriteMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1], true);

            NRF_LOG_PRINTF("Waiting for FIFO count > 2...\r\n");
            while ((fifoCount = mpu6500GetFIFOCount()) < 3);
            NRF_LOG_PRINTF("Current FIFO count=%d\r\n",fifoCount);
            NRF_LOG_PRINTF("Reading FIFO data...\r\n");
            mpu6500GetFIFOBytes(fifoBuffer, fifoCount);

            NRF_LOG_PRINTF("Reading interrupt status...\r\n");
            NRF_LOG_PRINTF("Current interrupt status=%d\r\n",mpu6500GetIntStatus());

            NRF_LOG_PRINTF("Reading final memory update 6/7 (function unknown)...\r\n");
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = dmpUpdates[pos];
            mpu6500ReadMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

            NRF_LOG_PRINTF("Waiting for FIFO count > 2...\r\n");
            while ((fifoCount = mpu6500GetFIFOCount()) < 3);
            NRF_LOG_PRINTF("Current FIFO count=%d\r\n",fifoCount);
            NRF_LOG_PRINTF("Reading FIFO data...\r\n");
            mpu6500GetFIFOBytes(fifoBuffer, fifoCount);

            NRF_LOG_PRINTF("Reading interrupt status...\r\n");
            NRF_LOG_PRINTF("Current interrupt status=%d\r\n",mpu6500GetIntStatus());

            NRF_LOG_PRINTF("Writing final memory update 7/7 (function unknown)...\r\n");
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = dmpUpdates[pos];
            mpu6500WriteMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1], true);

            NRF_LOG_PRINTF("DMP is good to go! Finally.\r\n");

            NRF_LOG_PRINTF("Disabling DMP (you turn it on later)...\r\n");
            mpu6500SetDMPEnabled(false);

            NRF_LOG_PRINTF("Setting up internal 42-byte (default) DMP packet buffer...\r\n");
            dmpPacketSize = 42;
            
						//if ((dmpPacketBuffer = (uint8_t *)malloc(42)) == 0) {
            //    return 3; // TODO: proper error code for no memory
            //}

            NRF_LOG_PRINTF("Resetting FIFO and clearing INT status one last time...\r\n");
            mpu6500ResetFIFO();
            mpu6500GetIntStatus();
        } else {
            NRF_LOG_PRINTF("ERROR! DMP configuration verification failed.\r\n");
            return 2; // configuration block loading failed
        }
    } else {
        NRF_LOG_PRINTF("ERROR! DMP code verification failed.\r\n");
        return 1; // main binary block loading failed
    }
	
    return 0; // success
}

bool mpu6500DmpPacketAvailable(void) {
		return mpu6500GetFIFOCount() >= mpu6500DmpGetFIFOPacketSize();
}

// uint8_t mpu6500DmpSetFIFORate(uint8_t fifoRate);
// uint8_t mpu6500DmpGetFIFORate();
// uint8_t mpu6500DmpGetSampleStepSizeMS();
// uint8_t mpu6500DmpGetSampleFrequency();
// int32_t mpu6500DmpDecodeTemperature(int8_t tempReg);

//uint8_t mpu6500DmpRegisterFIFORateProcess(inv_obj_func func, int16_t priority);
//uint8_t mpu6500DmpUnregisterFIFORateProcess(inv_obj_func func);
//uint8_t mpu6500DmpRunFIFORateProcesses();

// uint8_t mpu6500DmpSendQuaternion(uint_fast16_t accuracy);
// uint8_t mpu6500DmpSendGyro(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t mpu6500DmpSendAccel(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t mpu6500DmpSendLinearAccel(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t mpu6500DmpSendLinearAccelInWorld(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t mpu6500DmpSendControlData(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t mpu6500DmpSendSensorData(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t mpu6500DmpSendExternalSensorData(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t mpu6500DmpSendGravity(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t mpu6500DmpSendPacketNumber(uint_fast16_t accuracy);
// uint8_t mpu6500DmpSendQuantizedAccel(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t mpu6500DmpSendEIS(uint_fast16_t elements, uint_fast16_t accuracy);

uint8_t mpu6500DmpGetAccel1(int32_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    data[0] = (((uint32_t)packet[28] << 24) | ((uint32_t)packet[29] << 16) | ((uint32_t)packet[30] << 8) | packet[31]);
    data[1] = (((uint32_t)packet[32] << 24) | ((uint32_t)packet[33] << 16) | ((uint32_t)packet[34] << 8) | packet[35]);
    data[2] = (((uint32_t)packet[36] << 24) | ((uint32_t)packet[37] << 16) | ((uint32_t)packet[38] << 8) | packet[39]);
    return 0;
}
uint8_t mpu6500DmpGetAccel2(int16_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    data[0] = (packet[28] << 8) | packet[29];
    data[1] = (packet[32] << 8) | packet[33];
    data[2] = (packet[36] << 8) | packet[37];
    return 0;
}
uint8_t mpu6500DmpGetAccel3(VectorInt16 *v, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    v -> x = (packet[28] << 8) | packet[29];
    v -> y = (packet[32] << 8) | packet[33];
    v -> z = (packet[36] << 8) | packet[37];
    return 0;
}
uint8_t mpu6500DmpGetQuaternion1(int32_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    data[0] = (((uint32_t)packet[0] << 24) | ((uint32_t)packet[1] << 16) | ((uint32_t)packet[2] << 8) | packet[3]);
    data[1] = (((uint32_t)packet[4] << 24) | ((uint32_t)packet[5] << 16) | ((uint32_t)packet[6] << 8) | packet[7]);
    data[2] = (((uint32_t)packet[8] << 24) | ((uint32_t)packet[9] << 16) | ((uint32_t)packet[10] << 8) | packet[11]);
    data[3] = (((uint32_t)packet[12] << 24) | ((uint32_t)packet[13] << 16) | ((uint32_t)packet[14] << 8) | packet[15]);
    return 0;
}
uint8_t mpu6500DmpGetQuaternion2(int16_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    data[0] = ((packet[0] << 8) | packet[1]);
    data[1] = ((packet[4] << 8) | packet[5]);
    data[2] = ((packet[8] << 8) | packet[9]);
    data[3] = ((packet[12] << 8) | packet[13]);
    return 0;
}
uint8_t mpu6500DmpGetQuaternion3(Quaternion *q, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    int16_t qI[4];
    uint8_t status = mpu6500DmpGetQuaternion2(qI, packet);
    if (status == 0) {
        q -> w = (float)qI[0] / 16384.0f;
        q -> x = (float)qI[1] / 16384.0f;
        q -> y = (float)qI[2] / 16384.0f;
        q -> z = (float)qI[3] / 16384.0f;
        return 0;
    }
    return status; // int16 return value, indicates error if this line is reached
}
// uint8_t mpu6500DmpGet6AxisQuaternion(long *data, const uint8_t* packet);
// uint8_t mpu6500DmpGetRelativeQuaternion(long *data, const uint8_t* packet);
uint8_t mpu6500DmpGetGyro1(int32_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    data[0] = (((uint32_t)packet[16] << 24) | ((uint32_t)packet[17] << 16) | ((uint32_t)packet[18] << 8) | packet[19]);
    data[1] = (((uint32_t)packet[20] << 24) | ((uint32_t)packet[21] << 16) | ((uint32_t)packet[22] << 8) | packet[23]);
    data[2] = (((uint32_t)packet[24] << 24) | ((uint32_t)packet[25] << 16) | ((uint32_t)packet[26] << 8) | packet[27]);
    return 0;
}

// velg ut fra hvilket format som er ønsket.
uint8_t mpu6500DmpGetGyro2(int16_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    data[0] = (packet[16] << 8) | packet[17];
    data[1] = (packet[20] << 8) | packet[21];
    data[2] = (packet[24] << 8) | packet[25];
    return 0;
}
uint8_t mpu6500DmpGetGyro3(VectorInt16 *v, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    v -> x = (packet[16] << 8) | packet[17];
    v -> y = (packet[20] << 8) | packet[21];
    v -> z = (packet[24] << 8) | packet[25];
    return 0;
}
// uint8_t mpu6500DmpSetLinearAccelFilterCoefficient(float coef);
// uint8_t mpu6500DmpGetLinearAccel(long *data, const uint8_t* packet);
uint8_t mpu6500DmpGetLinearAccel(VectorInt16 *v, VectorInt16 *vRaw, VectorFloat *gravity) {
    // get rid of the gravity component (+1g = +8192 in standard DMP FIFO packet, sensitivity is 2g)
    v -> x = vRaw -> x - gravity -> x*8192;
    v -> y = vRaw -> y - gravity -> y*8192;
    v -> z = vRaw -> z - gravity -> z*8192;
    return 0;
}
// uint8_t mpu6500DmpGetLinearAccelInWorld(long *data, const uint8_t* packet);
uint8_t mpu6500DmpGetLinearAccelInWorld(VectorInt16 *v, VectorInt16 *vReal, Quaternion *q) {
    // rotate measured 3D acceleration vector into original state
    // frame of reference based on orientation quaternion
    memcpy(v, vReal, sizeof(VectorInt16));
		//TODO: Fix this
		//v -> rotate(q);
    return 0;
}
// uint8_t mpu6500DmpGetGyroAndAccelSensor(long *data, const uint8_t* packet);
// uint8_t mpu6500DmpGetGyroSensor(long *data, const uint8_t* packet);
// uint8_t mpu6500DmpGetControlData(long *data, const uint8_t* packet);
// uint8_t mpu6500DmpGetTemperature(long *data, const uint8_t* packet);
// uint8_t mpu6500DmpGetGravity(long *data, const uint8_t* packet);
uint8_t mpu6500DmpGetGravity(VectorFloat *v, Quaternion *q) {
    v -> x = 2 * (q -> x*q -> z - q -> w*q -> y);
    v -> y = 2 * (q -> w*q -> x + q -> y*q -> z);
    v -> z = q -> w*q -> w - q -> x*q -> x - q -> y*q -> y + q -> z*q -> z;
    return 0;
}
// uint8_t mpu6500DmpGetUnquantizedAccel(long *data, const uint8_t* packet);
// uint8_t mpu6500DmpGetQuantizedAccel(long *data, const uint8_t* packet);
// uint8_t mpu6500DmpGetExternalSensorData(long *data, int size, const uint8_t* packet);
// uint8_t mpu6500DmpGetEIS(long *data, const uint8_t* packet);

uint8_t mpu6500DmpGetEuler(float *data, Quaternion *q) {
    data[0] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);   // psi
    data[1] = -asin(2*q -> x*q -> z + 2*q -> w*q -> y);                              // theta
    data[2] = atan2(2*q -> y*q -> z - 2*q -> w*q -> x, 2*q -> w*q -> w + 2*q -> z*q -> z - 1);   // phi
    return 0;
}
uint8_t mpu6500DmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity) {
    // yaw: (about Z axis)
    data[0] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);
    // pitch: (nose up/down, about Y axis)
    data[1] = atan(gravity -> x / sqrt(gravity -> y*gravity -> y + gravity -> z*gravity -> z));
    // roll: (tilt left/right, about X axis)
    data[2] = atan(gravity -> y / sqrt(gravity -> x*gravity -> x + gravity -> z*gravity -> z));
    return 0;
}

// uint8_t mpu6500DmpGetAccelFloat(float *data, const uint8_t* packet);
// uint8_t mpu6500DmpGetQuaternionFloat(float *data, const uint8_t* packet);

uint8_t mpu6500DmpProcessFIFOPacket(const unsigned char *dmpData) {
    /*for (uint8_t k = 0; k < dmpPacketSize; k++) {
        if (dmpData[k] < 0x10) Serial.print("0");
        Serial.print(dmpData[k], HEX);
        Serial.print(" ");
    }
    Serial.print("\n");*/
    //Serial.println((uint16_t)dmpPacketBuffer);
    return 0;
}
uint8_t mpu6500DmpReadAndProcessFIFOPacket(uint8_t numPackets, uint8_t *processed) {
    uint8_t status;
    uint8_t buf[dmpPacketSize];
    for (uint8_t i = 0; i < numPackets; i++) {
        // read packet from FIFO
        mpu6500GetFIFOBytes(buf, dmpPacketSize);
        // process packet
        if ((status = mpu6500DmpProcessFIFOPacket(buf)) > 0) return status;
        
        // increment external process count variable, if supplied
        if (processed != 0) (*processed)++;
    }
    return 0;
}

// uint8_t mpu6500DmpSetFIFOProcessedCallback(void (*func) (void));

// uint8_t mpu6500DmpInitFIFOParam();
// uint8_t mpu6500DmpCloseFIFO();
// uint8_t mpu6500DmpSetGyroDataSource(uint_fast8_t source);
// uint8_t mpu6500DmpDecodeQuantizedAccel();
// uint32_t mpu6500DmpGetGyroSumOfSquare();
// uint32_t mpu6500DmpGetAccelSumOfSquare();
// void mpu6500DmpOverrideQuaternion(long *q);

uint16_t mpu6500DmpGetFIFOPacketSize(void) {
    return dmpPacketSize;
}

