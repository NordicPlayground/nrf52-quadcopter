// I2Cdev library collection - MPU9250 I2C device class, 9-axis MotionApps 4.1 implementation
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

// MotionApps 4.1 DMP implementation, built using the MPU-9150 "MotionFit" board
#define MPU9250_INCLUDE_DMP_MOTIONAPPS41

#include "i2cdev.h"
#include "mpu6500.h"
#include "helper_3dmath.h"
#include "MPU9250_9Axis_MotionApps41.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "nrf_delay.h"

static uint8_t *dmpPacketBuffer;
static uint16_t dmpPacketSize;

static uint8_t devAddr = MPU6500_ADDRESS_AD0_LOW;

uint8_t mpu6500DmpInitialize() 
{
    mpu6500Reset();
    nrf_delay_ms(30);
    mpu6500SetSleepEnabled(false);

    // get MPU hardware revision
    SEGGER_RTT_printf(0,"Selecting user bank 16...\r\n");
    mpu6500SetMemoryBank(0x10, true, true);
    SEGGER_RTT_printf(0,"Selecting memory byte 6...\r\n");
    mpu6500SetMemoryStartAddress(0x06);
    SEGGER_RTT_printf(0,"Checking hardware revision...\r\n");
    SEGGER_RTT_printf(0,"Revision @ user[16][6] = %d \r\n", mpu6500ReadMemoryByte());
    SEGGER_RTT_printf(0,"Resetting memory bank selection to 0...\r\n");
    mpu6500SetMemoryBank(0, false, false);

    // get X/Y/Z gyro offsets
    SEGGER_RTT_printf(0,"Reading gyro offset values...\r\n");
    int16_t xgOffset = mpu6500GetXGyroOffsetUser();
    int16_t ygOffset = mpu6500GetYGyroOffsetUser();
    int16_t zgOffset = mpu6500GetZGyroOffsetUser();
    SEGGER_RTT_printf(0,"X gyro offset = %d\r\n", xgOffset);
    SEGGER_RTT_printf(0,"Y gyro offset = %d\r\n", ygOffset);
    SEGGER_RTT_printf(0,"Z gyro offset = %d\r\n", zgOffset);
    
		uint8_t *buffer;
    i2cdev_readByte(devAddr, MPU6500_RA_USER_CTRL, buffer); // ?
    
    SEGGER_RTT_printf(0,"Enabling interrupt latch, clear on any read, AUX bypass enabled\r\n");
    i2cdev_writeByte(devAddr, MPU6500_RA_INT_PIN_CFG, 0x32);

    // enable MPU AUX I2C bypass mode
    //SEGGER_RTT_printf(0,"Enabling AUX I2C bypass mode...\r\n"));
    //setI2CBypassEnabled(true);

    SEGGER_RTT_printf(0,"Setting magnetometer mode to power-down...\r\n");
    //mag -> setMode(0);
    i2cdev_writeByte(0x0E, 0x0A, 0x00);

    SEGGER_RTT_printf(0,"Setting magnetometer mode to fuse access...\r\n");
    //mag -> setMode(0x0F);
    i2cdev_writeByte(0x0E, 0x0A, 0x0F);

    SEGGER_RTT_printf(0,"Reading mag magnetometer factory calibration...\r\n");
    int8_t asax, asay, asaz;
    //mag -> getAdjustment(&asax, &asay, &asaz);
    i2cdev_readBytes(0x0E, 0x10, 3, buffer);
    asax = (int8_t)buffer[0];
    asay = (int8_t)buffer[1];
    asaz = (int8_t)buffer[2];
    SEGGER_RTT_printf(0,"Adjustment X/Y/Z = %d/%d/%d \r\n",asax,asay,asaz);
    SEGGER_RTT_printf(0,"Setting magnetometer mode to power-down...\r\n");
    //mag -> setMode(0);
    i2cdev_writeByte(0x0E, 0x0A, 0x00);

    // load DMP code into memory banks
    SEGGER_RTT_printf(0,"Writing DMP code to MPU memory banks (%d bytes)\r\n", MPU9250_DMP_CODE_SIZE);

    if (mpu6500WriteProgMemoryBlock(dmpMemory, MPU9250_DMP_CODE_SIZE,0,0,true)) {
        SEGGER_RTT_printf(0,"Success! DMP code written and verified. \r\n");

        SEGGER_RTT_printf(0,"Configuring DMP and related settings...\r\n");

        // write DMP configuration
        SEGGER_RTT_printf(0,"Writing DMP configuration to MPU memory banks (%d bytes in config def)", MPU9250_DMP_CONFIG_SIZE);
			
        if (mpu6500WriteProgDMPConfigurationSet(dmpConfig, MPU9250_DMP_CONFIG_SIZE)) {
            SEGGER_RTT_printf(0,"Success! DMP configuration written and verified.");

            SEGGER_RTT_printf(0,"Setting DMP and FIFO_OFLOW interrupts enabled...\r\n");
            mpu6500SetIntEnabled(0x12);

            SEGGER_RTT_printf(0,"Setting sample rate to 200Hz...\r\n");
            mpu6500SetRate(4); // 1khz / (1 + 4) = 200 Hz

            SEGGER_RTT_printf(0,"Setting clock source to Z Gyro...\r\n");
            mpu6500SetClockSource(MPU6500_CLOCK_PLL_ZGYRO);

            SEGGER_RTT_printf(0,"Setting DLPF bandwidth to 42Hz...\r\n");
            mpu6500SetDLPFMode(MPU6500_DLPF_BW_42);

            SEGGER_RTT_printf(0,"Setting external frame sync to TEMP_OUT_L[0]...\r\n");
            mpu6500SetExternalFrameSync(MPU6500_EXT_SYNC_TEMP_OUT_L);

            SEGGER_RTT_printf(0,"Setting gyro sensitivity to +/- 2000 deg/sec...\r\n");
            mpu6500SetFullScaleGyroRange(MPU6500_GYRO_FS_2000);

            SEGGER_RTT_printf(0,"Setting DMP configuration bytes (function unknown)...\r\n");
            mpu6500SetDMPConfig1(0x03);
            mpu6500SetDMPConfig2(0x00);

            //SEGGER_RTT_printf(0,"Clearing OTP Bank flag...\r\n");
            //setOTPBankValid(false);

            SEGGER_RTT_printf(0,"Setting X/Y/Z gyro offsets to previous values...\r\n");
            mpu6500SetXGyroOffsetUser(xgOffset);
            mpu6500SetYGyroOffsetUser(ygOffset);
            mpu6500SetZGyroOffsetUser(zgOffset);

            SEGGER_RTT_printf(0,"Setting X/Y/Z gyro user offsets to zero...\r\n");
            mpu6500SetXGyroOffsetUser(0);
            mpu6500SetYGyroOffsetUser(0);
            mpu6500SetZGyroOffsetUser(0);

            SEGGER_RTT_printf(0,"Writing final memory update 1/19 (function unknown)...\r\n");
            uint8_t dmpUpdate[16], j;
            uint16_t pos = 0;
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = dmpUpdates[pos];
            mpu6500WriteMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1], true);

            SEGGER_RTT_printf(0,"Writing final memory update 2/19 (function unknown)...\r\n");
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = dmpUpdates[pos];
            mpu6500WriteMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1], true);

            SEGGER_RTT_printf(0,"Resetting FIFO...\r\n");
            mpu6500ResetFIFO();

            SEGGER_RTT_printf(0,"Reading FIFO count...\r\n");
            uint8_t fifoCount = mpu6500GetFIFOCount();

            SEGGER_RTT_printf(0,"Current FIFO count= %d \r\n", fifoCount);
            uint8_t fifoBuffer[128];
            //getFIFOBytes(fifoBuffer, fifoCount);

            SEGGER_RTT_printf(0,"Writing final memory update 3/19 (function unknown)...\r\n");
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = dmpUpdates[pos];
            mpu6500WriteMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1], true);

            SEGGER_RTT_printf(0,"Writing final memory update 4/19 (function unknown)...\r\n");
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = dmpUpdates[pos];
            mpu6500WriteMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1], true);
						
            SEGGER_RTT_printf(0,"Disabling all standby flags...\r\n");
            i2cdev_writeByte(0x68, MPU6500_RA_PWR_MGMT_2, 0x00);

            SEGGER_RTT_printf(0,"Setting accelerometer sensitivity to +/- 2g...\r\n");
            i2cdev_writeByte(0x68, MPU6500_RA_ACCEL_CONFIG, 0x00);
						
						/*
						//There is some changes in registers here, between MPU-6050 and MPU6500
            SEGGER_RTT_printf(0,"Setting motion detection threshold to 2...\r\n");
            setMotionDetectionThreshold(2);

            SEGGER_RTT_printf(0,"Setting zero-motion detection threshold to 156...\r\n");
            setZeroMotionDetectionThreshold(156);

            SEGGER_RTT_printf(0,"Setting motion detection duration to 80...\r\n");
            setMotionDetectionDuration(80);

            SEGGER_RTT_printf(0,"Setting zero-motion detection duration to 0...\r\n");
            setZeroMotionDetectionDuration(0);
						*/
						
            SEGGER_RTT_printf(0,"Setting AK8975 to single measurement mode...\r\n");
            //mag -> setMode(1);
            i2cdev_writeByte(0x0E, 0x0A, 0x01);

            // setup AK8975 (0x0E) as Slave 0 in read mode
            SEGGER_RTT_printf(0,"Setting up AK8975 read slave 0...\r\n");
            i2cdev_writeByte(0x68, MPU6500_RA_I2C_SLV0_ADDR, 0x8E);
            i2cdev_writeByte(0x68, MPU6500_RA_I2C_SLV0_REG,  0x01);
            i2cdev_writeByte(0x68, MPU6500_RA_I2C_SLV0_CTRL, 0xDA);

            // setup AK8975 (0x0E) as Slave 2 in write mode
            SEGGER_RTT_printf(0,"Setting up AK8975 write slave 2...\r\n");
            i2cdev_writeByte(0x68, MPU6500_RA_I2C_SLV2_ADDR, 0x0E);
            i2cdev_writeByte(0x68, MPU6500_RA_I2C_SLV2_REG,  0x0A);
            i2cdev_writeByte(0x68, MPU6500_RA_I2C_SLV2_CTRL, 0x81);
            i2cdev_writeByte(0x68, MPU6500_RA_I2C_SLV2_DO,   0x01);

            // setup I2C timing/delay control
            SEGGER_RTT_printf(0,"Setting up slave access delay...\r\n");
            i2cdev_writeByte(0x68, MPU6500_RA_I2C_SLV4_CTRL, 0x18);
            i2cdev_writeByte(0x68, MPU6500_RA_I2C_MST_DELAY_CTRL, 0x05);

            // enable interrupts
            SEGGER_RTT_printf(0,"Enabling default interrupt behavior/no bypass...\r\n");
            i2cdev_writeByte(0x68, MPU6500_RA_INT_PIN_CFG, 0x00);

            // enable I2C master mode and reset DMP/FIFO
            SEGGER_RTT_printf(0,"Enabling I2C master mode...\r\n");
            i2cdev_writeByte(0x68, MPU6500_RA_USER_CTRL, 0x20);
            SEGGER_RTT_printf(0,"Resetting FIFO...\r\n");
            i2cdev_writeByte(0x68, MPU6500_RA_USER_CTRL, 0x24);
            SEGGER_RTT_printf(0,"Rewriting I2C master mode enabled because...I don't know");
            i2cdev_writeByte(0x68, MPU6500_RA_USER_CTRL, 0x20);
            SEGGER_RTT_printf(0,"Enabling and resetting DMP/FIFO...\r\n");
            i2cdev_writeByte(0x68, MPU6500_RA_USER_CTRL, 0xE8);

            SEGGER_RTT_printf(0,"Writing final memory update 5/19 (function unknown)...\r\n");
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = dmpUpdates[pos];
            mpu6500WriteMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1], true);
            SEGGER_RTT_printf(0,"Writing final memory update 6/19 (function unknown)...\r\n");
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = dmpUpdates[pos];
            mpu6500WriteMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1], true);
            SEGGER_RTT_printf(0,"Writing final memory update 7/19 (function unknown)...\r\n");
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = dmpUpdates[pos];
            mpu6500WriteMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1], true);
            SEGGER_RTT_printf(0,"Writing final memory update 8/19 (function unknown)...\r\n");
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = dmpUpdates[pos];
            mpu6500WriteMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1], true);
            SEGGER_RTT_printf(0,"Writing final memory update 9/19 (function unknown)...\r\n");
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = dmpUpdates[pos];
            mpu6500WriteMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1], true);
            SEGGER_RTT_printf(0,"Writing final memory update 10/19 (function unknown)...\r\n");
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = dmpUpdates[pos];
            mpu6500WriteMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1], true);
            SEGGER_RTT_printf(0,"Writing final memory update 11/19 (function unknown)...\r\n");
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = dmpUpdates[pos];
            mpu6500WriteMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1], true);
            
            SEGGER_RTT_printf(0,"Reading final memory update 12/19 (function unknown)...\r\n");
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = dmpUpdates[pos];
            mpu6500ReadMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);
            #ifdef DEBUG
                SEGGER_RTT_printf(0,"Read bytes: ");
                for (j = 0; j < 4; j++) {
                    printf(dmpUpdate[3 + j], HEX);
                    SEGGER_RTT_printf(0," ");
                }
                SEGGER_RTT_printf(0,"");
            #endif

            SEGGER_RTT_printf(0,"Writing final memory update 13/19 (function unknown)...\r\n");
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = dmpUpdates[pos];
            mpu6500WriteMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1], true);
            SEGGER_RTT_printf(0,"Writing final memory update 14/19 (function unknown)...\r\n");
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = dmpUpdates[pos];
            mpu6500WriteMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1], true);
            SEGGER_RTT_printf(0,"Writing final memory update 15/19 (function unknown)...\r\n");
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = dmpUpdates[pos];
            mpu6500WriteMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1], true);
            SEGGER_RTT_printf(0,"Writing final memory update 16/19 (function unknown)...\r\n");
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = dmpUpdates[pos];
            mpu6500WriteMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1], true);
            SEGGER_RTT_printf(0,"Writing final memory update 17/19 (function unknown)...\r\n");
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = dmpUpdates[pos];
            mpu6500WriteMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1], true);

            SEGGER_RTT_printf(0,"Waiting for FIRO count >= 46...\r\n");
            while ((fifoCount = mpu6500GetFIFOCount()) < 46);
            SEGGER_RTT_printf(0,"Reading FIFO...\r\n");
						mpu6500GetFIFOBytes(fifoBuffer, fifoCount);
						//mpu6500GetFIFOBytes(fifoBuffer, min(fifoCount, 128)); // safeguard only 128 bytes
            SEGGER_RTT_printf(0,"Reading interrupt status...\r\n");
            mpu6500GetIntStatus();

            SEGGER_RTT_printf(0,"Writing final memory update 18/19 (function unknown)...\r\n");
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = dmpUpdates[pos];
            mpu6500WriteMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1], true);

            SEGGER_RTT_printf(0,"Waiting for FIRO count >= 48...\r\n");
            while ((fifoCount = mpu6500GetFIFOCount()) < 48);
            SEGGER_RTT_printf(0,"Reading FIFO...\r\n");
						
						mpu6500GetFIFOBytes(fifoBuffer, fifoCount);
						//mpu6500GetFIFOBytes(fifoBuffer, min(fifoCount, 128)); // safeguard only 128 bytes
            SEGGER_RTT_printf(0,"Reading interrupt status...\r\n");
            mpu6500GetIntStatus();
            SEGGER_RTT_printf(0,"Waiting for FIRO count >= 48...\r\n");
            while ((fifoCount = mpu6500GetFIFOCount()) < 48);
            SEGGER_RTT_printf(0,"Reading FIFO...\r\n");
						mpu6500GetFIFOBytes(fifoBuffer, fifoCount);
						//mpu6500GetFIFOBytes(fifoBuffer, min(fifoCount, 128)); // safeguard only 128 bytes
            SEGGER_RTT_printf(0,"Reading interrupt status...\r\n");
            mpu6500GetIntStatus();

            SEGGER_RTT_printf(0,"Writing final memory update 19/19 (function unknown)...\r\n");
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = dmpUpdates[pos];
            mpu6500WriteMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1], true);

            SEGGER_RTT_printf(0,"Disabling DMP (you turn it on later)...\r\n");
            mpu6500SetDMPEnabled(false);

            SEGGER_RTT_printf(0,"Setting up internal 48-byte (default) DMP packet buffer...\r\n");
            dmpPacketSize = 48;
            /*if ((dmpPacketBuffer = (uint8_t *)malloc(42)) == 0) {
                return 3; // TODO: proper error code for no memory
            }*/

            SEGGER_RTT_printf(0,"Resetting FIFO and clearing INT status one last time...\r\n");
            mpu6500ResetFIFO();
            mpu6500GetIntStatus();
        } else {
            SEGGER_RTT_printf(0,"ERROR! DMP configuration verification failed. \r\n");
            return 2; // configuration block loading failed
        }
    } else {
        SEGGER_RTT_printf(0,"ERROR! DMP code verification failed. \r\n");
        return 1; // main binary block loading failed
    }
    return 0; // success
}

bool mpu6500DmpPacketAvailable() {
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
    data[0] = ((packet[34] << 24) + (packet[35] << 16) + (packet[36] << 8) + packet[37]);
    data[1] = ((packet[38] << 24) + (packet[39] << 16) + (packet[40] << 8) + packet[41]);
    data[2] = ((packet[42] << 24) + (packet[43] << 16) + (packet[44] << 8) + packet[45]);
    return 0;
}
uint8_t mpu6500DmpGetAccel2(int16_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    data[0] = (packet[34] << 8) + packet[35];
    data[1] = (packet[38] << 8) + packet[39];
    data[2] = (packet[42] << 8) + packet[43];
    return 0;
}
uint8_t mpu6500DmpGetAccel3(VectorInt16 *v, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    v -> x = (packet[34] << 8) + packet[35];
    v -> y = (packet[38] << 8) + packet[39];
    v -> z = (packet[42] << 8) + packet[43];
    return 0;
}
uint8_t mpu6500DmpGetQuaternion1(int32_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    data[0] = ((packet[0] << 24) + (packet[1] << 16) + (packet[2] << 8) + packet[3]);
    data[1] = ((packet[4] << 24) + (packet[5] << 16) + (packet[6] << 8) + packet[7]);
    data[2] = ((packet[8] << 24) + (packet[9] << 16) + (packet[10] << 8) + packet[11]);
    data[3] = ((packet[12] << 24) + (packet[13] << 16) + (packet[14] << 8) + packet[15]);
    return 0;
}
uint8_t mpu6500DmpGetQuaternion2(int16_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    data[0] = ((packet[0] << 8) + packet[1]);
    data[1] = ((packet[4] << 8) + packet[5]);
    data[2] = ((packet[8] << 8) + packet[9]);
    data[3] = ((packet[12] << 8) + packet[13]);
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
    data[0] = ((packet[16] << 24) + (packet[17] << 16) + (packet[18] << 8) + packet[19]);
    data[1] = ((packet[20] << 24) + (packet[21] << 16) + (packet[22] << 8) + packet[23]);
    data[2] = ((packet[24] << 24) + (packet[25] << 16) + (packet[26] << 8) + packet[27]);
    return 0;
}
uint8_t mpu6500DmpGetGyro2(int16_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    data[0] = (packet[16] << 8) + packet[17];
    data[1] = (packet[20] << 8) + packet[21];
    data[2] = (packet[24] << 8) + packet[25];
    return 0;
}
uint8_t mpu6500DmpGetMag(int16_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    data[0] = (packet[28] << 8) + packet[29];
    data[1] = (packet[30] << 8) + packet[31];
    data[2] = (packet[32] << 8) + packet[33];
    return 0;
}
// uint8_t mpu6500DmpSetLinearAccelFilterCoefficient(float coef);
// uint8_t mpu6500DmpGetLinearAccel(long *data, const uint8_t* packet);
uint8_t mpu6500DmpGetLinearAccel(VectorInt16 *v, VectorInt16 *vRaw, VectorFloat *gravity) {
    // get rid of the gravity component (+1g = +4096 in standard DMP FIFO packet)
    v -> x = vRaw -> x - gravity -> x*4096;
    v -> y = vRaw -> y - gravity -> y*4096;
    v -> z = vRaw -> z - gravity -> z*4096;
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
        if (processed != 0) *processed++;
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
uint16_t mpu6500DmpGetFIFOPacketSize() {
    return dmpPacketSize;
}
