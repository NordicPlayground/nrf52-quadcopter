/**
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * imu.c - inertial measurement unit
 */
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include "SEGGER_RTT.h"
#include "imu.h"
#include "ak8963.h"
#include "lps25h.h"
#include "ms5611.h"
#include "i2cdev.h"
#include "mpu6500.h"
#include "MPU6050_6Axis_MotionApps20.h"

#include "systick.h"

#include "nrf_delay.h"

#define M_PI 3.1415f

#define IMU_ENABLE_PRESSURE_LPS25H			1
#define IMU_ENABLE_MAG_AK8963						0
//#define IMU_MPU6500_DLPF_256HZ
#define IMU_ENABLE_PRESSURE_MS5611 			1

#define IMU_TAKE_ACCEL_BIAS

#define IMU_GYRO_FS_CFG       MPU6500_GYRO_FS_500
#define IMU_DEG_PER_LSB_CFG   MPU6500_DEG_PER_LSB_2000
#define IMU_ACCEL_FS_CFG      MPU6500_ACCEL_FS_4
#define IMU_G_PER_LSB_CFG     MPU6500_G_PER_LSB_16
#define IMU_1G_RAW            (int16_t)(1.0f / MPU6500_G_PER_LSB_16)

#define IMU_VARIANCE_MAN_TEST_TIMEOUT 1000 			// Timeout in ms
#define IMU_MAN_TEST_LEVEL_MAX        5.0f      // Max degrees off

#define MAG_GAUSS_PER_LSB     666.7f

#define IMU_STARTUP_TIME_MS   1000

#define GYRO_NBR_OF_AXES 3
#define GYRO_X_SIGN      (-1)
#define GYRO_Y_SIGN      (-1)
#define GYRO_Z_SIGN      (-1)
#define GYRO_NBR_OF_AXES            3
#define GYRO_MIN_BIAS_TIMEOUT_MS    (1*1000)

// Variance threshold to take zero bias for gyro
#define GYRO_VARIANCE_BASE        2000
#define GYRO_VARIANCE_THRESHOLD_X (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Y (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Z (GYRO_VARIANCE_BASE)

BiasObj    gyroBias;
#ifdef IMU_TAKE_ACCEL_BIAS
BiasObj    accelBias;
#endif
static int32_t    varianceSampleTime;
static Axis3i16   gyroMpu;
static Axis3i16   accelMpu;
static Axis3i16   accelLPF;
static Axis3i16   accelLPFAligned;
static Axis3i16   mag;
static Axis3i32   accelStoredFilterValues;
static uint8_t    imuAccLpfAttFactor;
static bool isMagPresent;
static bool isBaroPresent;
//static bool isMs5611Present; Declared, never referenced. 1 Warning.

//static bool isMpu6500TestPassed = true; Declared, never referenced. 1 Warning.
static bool isAK8963TestPassed = true;
static bool isLPS25HTestPassed = true;
//static bool isMs5611TestPassed = true; Declared, never referenced. 1 Warning.

// Pre-calculated values for accelerometer alignment
float cosPitch;
float sinPitch;
float cosRoll;
float sinRoll;
/**
 * MPU6500 selt test function. If the chip is moved to much during the self test
 * it will cause the test to fail.
 */
static void imuBiasInit(BiasObj* bias);
static void imuCalculateBiasMean(BiasObj* bias, Axis3i32* meanOut);
static void imuCalculateVarianceAndMean(BiasObj* bias, Axis3i32* varOut, Axis3i32* meanOut);
static bool imuFindBiasValue(BiasObj* bias);
static void imuAddBiasValue(BiasObj* bias, Axis3i16* dVal);
static void imuAccIIRLPFilter(Axis3i16* in, Axis3i16* out,
                              Axis3i32* storedValues, int32_t attenuation);
static void imuAccAlignToGravity(Axis3i16* in, Axis3i16* out);

static bool isInit;
static uint8_t devStatus;

uint8_t imu6Init(void)
{
  if(isInit)
    return devStatus;

  isMagPresent = false;
  isBaroPresent = false;
	
  mpu6500Init();
	
  if (mpu6500TestConnection() == true)
  {
    SEGGER_RTT_printf(0,"MPU9250 I2C connection [OK].\n");
  }
  else
  {
    SEGGER_RTT_printf(0,"MPU9250 I2C connection [FAIL].\n");
  }

  mpu6500Reset();
	//Activate pass-through mode on the MPU9250. Make the nRF52-chip able to speak directly to the auxillary LPS25H-barometer.
	mpu6500SetI2CBypassEnabled(true);
  nrf_delay_ms(50);
	//devStatus = mpu6500DmpInitialize();
	
	//Most, if not all functions are called by mpu6500DmpInitialize. TODO: Check if some of the functions below needs to be called. 
  // Activate MPU6500
  mpu6500SetSleepEnabled(false);
  // Enable temp sensor
  mpu6500SetTempSensorEnabled(true);
  // Disable interrupts
  mpu6500SetIntEnabled(false);
  // Connect the AK8963 to the main I2C bus
  mpu6500SetI2CBypassEnabled(true);
  // Set x-axis gyro as clock source
  mpu6500SetClockSource(MPU6500_CLOCK_PLL_XGYRO);
  // Set gyro full scale range
  mpu6500SetFullScaleGyroRange(IMU_GYRO_FS_CFG);
  // Set accelerometer full scale range
  mpu6500SetFullScaleAccelRange(IMU_ACCEL_FS_CFG);

#ifdef IMU_MPU6500_DLPF_256HZ
  // 256Hz digital low-pass filter only works with little vibrations
  // Set output rate (15): 8000 / (1 + 15) = 500Hz
  mpu6500SetRate(15);
  // Set digital low-pass bandwidth
  mpu6500SetDLPFMode(MPU6500_DLPF_BW_256);
#else
  // To low DLPF bandwidth might cause instability and decrease agility
  // but it works well for handling vibrations and unbalanced propellers
  // Set output rate (1): 1000 / (1 + 1) = 500Hz
  mpu6500SetRate(1);
  // Set digital low-pass bandwidth
  mpu6500SetDLPFMode(MPU6500_DLPF_BW_98);
#endif


#ifdef IMU_ENABLE_MAG_AK8963
  ak8963Init();
  if (ak8963TestConnection() == true)
  {
    isMagPresent = true;
    ak8963SetMode(AK8963_MODE_16BIT | AK8963_MODE_CONT2); // 16bit 100Hz
    SEGGER_RTT_printf(0,"AK8963 I2C connection [OK].\n");
  }
  else
  {
    SEGGER_RTT_printf(0,"AK8963 I2C connection [FAIL].\n");
  }
#endif

#ifdef IMU_ENABLE_PRESSURE_LPS25H
  lps25hInit();
  if (lps25hTestConnection() == true)
  {
    lps25hSetEnabled(true);
    isBaroPresent = true;
    SEGGER_RTT_printf(0,"LPS25H I2C connection [OK].\n");
  }
  else
  {
    //TODO: Should sensor test fail hard if no connection
    SEGGER_RTT_printf(0,"LPS25H I2C connection [FAIL].\n");
  }
#endif
	
#ifdef IMU_ENABLE_PRESSURE_MS5611
  if (ms5611Init() == true)
  {
    //isMs5611Present = true;
    SEGGER_RTT_printf(0,"MS5611 I2C connection [OK].\n");
  }
  else
  {
    SEGGER_RTT_printf(0,"MS5611 I2C connection [FAIL].\n");
  }
#endif

  imuBiasInit(&gyroBias);
#ifdef IMU_TAKE_ACCEL_BIAS
  imuBiasInit(&accelBias);
#endif
  varianceSampleTime = -GYRO_MIN_BIAS_TIMEOUT_MS + 1;
  imuAccLpfAttFactor = IMU_ACC_IIR_LPF_ATT_FACTOR;
	
  cosPitch = cosf(0.0f * (float) M_PI/180.0f);
  sinPitch = sinf(0.0f * (float) M_PI/180.0f);
  cosRoll = cosf(0.0f * (float) M_PI/180.0f);
  sinRoll = sinf(0.0f * (float) M_PI/180.0f);
	
  isInit = true;
	return devStatus;
}

bool imu6Test(void)
{
  bool testStatus = true;

  if (!isInit)
  {
    SEGGER_RTT_printf(0,"Uninitialized\n");
    testStatus = false;
  }

#ifdef IMU_ENABLE_MAG_AK8963
  testStatus &= isMagPresent;
  if (testStatus)
  {
    isAK8963TestPassed = ak8963SelfTest();
    testStatus = isAK8963TestPassed;
  }
#endif

#ifdef IMU_ENABLE_PRESSURE_LPS25H
  testStatus &= isBaroPresent;
  if (testStatus)
  {
    isLPS25HTestPassed = lps25hSelfTest();
    testStatus = isLPS25HTestPassed;
  }
#endif

	/*
	if (testStatus && isMs5611Present)
  {
    isMs5611TestPassed = ms5611SelfTest();
    testStatus = isMs5611TestPassed;
  }
	*/

  return testStatus;
}

bool imu6ManufacturingTest(void)
{
  bool testStatus = false;
  Axis3f gyro; // Gyro axis data in deg/s
  Axis3f acc;  // Accelerometer axis data in mG
  float pitch, roll;
  uint32_t startTick = ms_ticks;

  testStatus = mpu6500SelfTest();

  if (testStatus)
  {
    while (ms_ticks - startTick < IMU_VARIANCE_MAN_TEST_TIMEOUT)
    {
      imu6Read(&gyro, &acc);
      if (gyroBias.isBiasValueFound)
      {
        SEGGER_RTT_printf(0,"Gyro variance test [OK]\n");
        break;
      }
    }

    if (gyroBias.isBiasValueFound)
    {
      // Calculate pitch and roll based on accelerometer. Board must be level
      pitch = tanf(-acc.x/(sqrtf(acc.y*acc.y + acc.z*acc.z))) * 180/(float) M_PI;
      roll = tanf(acc.y/acc.z) * 180/(float) M_PI;

      if ((fabsf(roll) < IMU_MAN_TEST_LEVEL_MAX) && (fabsf(pitch) < IMU_MAN_TEST_LEVEL_MAX))
      {
        SEGGER_RTT_printf(0,"Acc level test [OK]\n");
        testStatus = true;
      }
      else
      {
        SEGGER_RTT_printf(0,"Acc level test Roll:%0.2f, Pitch:%0.2f [FAIL]\n", roll, pitch);
        testStatus = false;
      }
    }
    else
    {
      SEGGER_RTT_printf(0,"Gyro variance test [FAIL]\n");
      testStatus = false;
    }
  }

  return testStatus;
}

void imu6Read(Axis3f* gyroOut, Axis3f* accOut)
{
  mpu6500GetMotion6(&accelMpu.y, &accelMpu.x, &accelMpu.z, &gyroMpu.y, &gyroMpu.x, &gyroMpu.z);
	
  imuAddBiasValue(&gyroBias, &gyroMpu);
#ifdef IMU_TAKE_ACCEL_BIAS
  if (!accelBias.isBiasValueFound)
  {
    imuAddBiasValue(&accelBias, &accelMpu);
  }
#endif
  if (!gyroBias.isBiasValueFound)
  {
    imuFindBiasValue(&gyroBias);
		gyroBias.isBiasValueFound = true;
    if (gyroBias.isBiasValueFound)
    {
      //soundSetEffect(SND_CALIB);
      //ledseqRun(SYS_LED, seq_calibrated);
    }
  }

#ifdef IMU_TAKE_ACCEL_BIAS
  if (gyroBias.isBiasValueFound &&
      !accelBias.isBiasValueFound)
  {
    Axis3i32 mean;

    imuCalculateBiasMean(&accelBias, &mean);
    accelBias.bias.x = mean.x;
    accelBias.bias.y = mean.y;
    accelBias.bias.z = mean.z - IMU_1G_RAW;
    accelBias.isBiasValueFound = true;
  }
#endif


  imuAccIIRLPFilter(&accelMpu, &accelLPF, &accelStoredFilterValues,
                    (int32_t)imuAccLpfAttFactor);

  imuAccAlignToGravity(&accelLPF, &accelLPFAligned);

  // Re-map outputs
  gyroOut->x = -(gyroMpu.x - gyroBias.bias.x) * IMU_DEG_PER_LSB_CFG;
  gyroOut->y = (gyroMpu.y - gyroBias.bias.y) * IMU_DEG_PER_LSB_CFG;
  gyroOut->z = (gyroMpu.z - gyroBias.bias.z) * IMU_DEG_PER_LSB_CFG;
#ifdef IMU_TAKE_ACCEL_BIAS
  accOut->x = (accelLPFAligned.x - accelBias.bias.x) * IMU_G_PER_LSB_CFG;
  accOut->y = (accelLPFAligned.y - accelBias.bias.y) * IMU_G_PER_LSB_CFG;
  accOut->z = (accelLPFAligned.z - accelBias.bias.z) * IMU_G_PER_LSB_CFG;
#else
  accOut->x = -(accelLPFAligned.x) * IMU_G_PER_LSB_CFG;
  accOut->y = (accelLPFAligned.y) * IMU_G_PER_LSB_CFG;
  accOut->z = (accelLPFAligned.z) * IMU_G_PER_LSB_CFG;
#endif

}

bool imu6IsCalibrated(void)
{
  bool status;

  status = gyroBias.isBiasValueFound;
#ifdef IMU_TAKE_ACCEL_BIAS
  status &= accelBias.isBiasValueFound;
#endif

  return status;
}

void imu9Read(Axis3f* gyroOut, Axis3f* accOut, Axis3f* magOut)
{
  imu6Read(gyroOut, accOut);

  if (isMagPresent)
  {
    ak8963GetHeading(&mag.x, &mag.y, &mag.z);
    ak8963GetOverflowStatus();
    magOut->x = (float)mag.x / MAG_GAUSS_PER_LSB;
    magOut->y = (float)mag.y / MAG_GAUSS_PER_LSB;
    magOut->z = (float)mag.z / MAG_GAUSS_PER_LSB;
  }
  else
  {
    magOut->x = 0.0;
    magOut->y = 0.0;
    magOut->z = 0.0;
  }
}

bool imuHasBarometer(void)
{
  return isBaroPresent;
}

bool imuHasMangnetometer(void)
{
  return isMagPresent;
}

static void imuBiasInit(BiasObj* bias)
{
  bias->isBufferFilled = false;
  bias->bufHead = bias->buffer;
}

/**
 * Calculates the variance and mean for the bias buffer.
 */
static void imuCalculateVarianceAndMean(BiasObj* bias, Axis3i32* varOut, Axis3i32* meanOut)
{
  uint32_t i;
  int32_t sum[GYRO_NBR_OF_AXES] = {0};
  int64_t sumSq[GYRO_NBR_OF_AXES] = {0};

  for (i = 0; i < IMU_NBR_OF_BIAS_SAMPLES; i++)
  {
    sum[0] += bias->buffer[i].x;
    sum[1] += bias->buffer[i].y;
    sum[2] += bias->buffer[i].z;
    sumSq[0] += bias->buffer[i].x * bias->buffer[i].x;
    sumSq[1] += bias->buffer[i].y * bias->buffer[i].y;
    sumSq[2] += bias->buffer[i].z * bias->buffer[i].z;
  }

  varOut->x = (sumSq[0] - ((int64_t)sum[0] * sum[0]) / IMU_NBR_OF_BIAS_SAMPLES);
  varOut->y = (sumSq[1] - ((int64_t)sum[1] * sum[1]) / IMU_NBR_OF_BIAS_SAMPLES);
  varOut->z = (sumSq[2] - ((int64_t)sum[2] * sum[2]) / IMU_NBR_OF_BIAS_SAMPLES);

  meanOut->x = sum[0] / IMU_NBR_OF_BIAS_SAMPLES;
  meanOut->y = sum[1] / IMU_NBR_OF_BIAS_SAMPLES;
  meanOut->z = sum[2] / IMU_NBR_OF_BIAS_SAMPLES;

  isInit = true;
}

/**
 * Calculates the mean for the bias buffer.
 */
static void __attribute__((used)) imuCalculateBiasMean(BiasObj* bias, Axis3i32* meanOut)
{
  uint32_t i;
  int32_t sum[GYRO_NBR_OF_AXES] = {0};

  for (i = 0; i < IMU_NBR_OF_BIAS_SAMPLES; i++)
  {
    sum[0] += bias->buffer[i].x;
    sum[1] += bias->buffer[i].y;
    sum[2] += bias->buffer[i].z;
  }

  meanOut->x = sum[0] / IMU_NBR_OF_BIAS_SAMPLES;
  meanOut->y = sum[1] / IMU_NBR_OF_BIAS_SAMPLES;
  meanOut->z = sum[2] / IMU_NBR_OF_BIAS_SAMPLES;

}

/**
 * Adds a new value to the variance buffer and if it is full
 * replaces the oldest one. Thus a circular buffer.
 */
static void imuAddBiasValue(BiasObj* bias, Axis3i16* dVal)
{
  bias->bufHead->x = dVal->x;
  bias->bufHead->y = dVal->y;
  bias->bufHead->z = dVal->z;
  bias->bufHead++;

  if (bias->bufHead >= &bias->buffer[IMU_NBR_OF_BIAS_SAMPLES])
  {
    bias->bufHead = bias->buffer;
    bias->isBufferFilled = true;
  }
}

/**
 * Checks if the variances is below the predefined thresholds.
 * The bias value should have been added before calling this.
 * @param bias  The bias object
 */
static bool imuFindBiasValue(BiasObj* bias)
{
  bool foundBias = false;
	
  if (bias->isBufferFilled)
  {
    Axis3i32 variance;
    Axis3i32 mean;

    imuCalculateVarianceAndMean(bias, &variance, &mean);

    if (variance.x < GYRO_VARIANCE_THRESHOLD_X &&
        variance.y < GYRO_VARIANCE_THRESHOLD_Y &&
        variance.z < GYRO_VARIANCE_THRESHOLD_Z &&
        (varianceSampleTime + GYRO_MIN_BIAS_TIMEOUT_MS < ms_ticks))
    {
      varianceSampleTime = ms_ticks;
      bias->bias.x = mean.x;
      bias->bias.y = mean.y;
      bias->bias.z = mean.z;
      foundBias = true;
      bias->isBiasValueFound = true;
    }
  }
	
  return foundBias;
}

void getBiasValue(BiasObj* gbias, BiasObj* abias)
{
	gbias->bias = gyroBias.bias;
#ifdef IMU_TAKE_ACCEL_BIAS
	abias->bias = accelBias.bias;
#endif
}

static void imuAccIIRLPFilter(Axis3i16* in, Axis3i16* out, Axis3i32* storedValues, int32_t attenuation)
{
  out->x = iirLPFilterSingle(in->x, attenuation, &storedValues->x);
  out->y = iirLPFilterSingle(in->y, attenuation, &storedValues->y);
  out->z = iirLPFilterSingle(in->z, attenuation, &storedValues->z);
}


/**
 * Compensate for a miss-aligned accelerometer. It uses the trim
 * data gathered from the UI and written in the config-block to
 * rotate the accelerometer to be aligned with gravity.
 */
static void imuAccAlignToGravity(Axis3i16* in, Axis3i16* out)
{
  Axis3i16 rx;
  Axis3i16 ry;

  // Rotate around x-axis
  rx.x = in->x;
  rx.y = in->y * cosRoll - in->z * sinRoll;
  rx.z = in->y * sinRoll + in->z * cosRoll;

  // Rotate around y-axis
  ry.x = rx.x * cosPitch - rx.z * sinPitch;
  ry.y = rx.y;
  ry.z = -rx.x * sinPitch + rx.z * cosPitch;

  out->x = ry.x;
  out->y = ry.y;
  out->z = ry.z;
}
