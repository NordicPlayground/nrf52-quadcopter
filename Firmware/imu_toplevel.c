#include "imu_toplevel.h"
#include "include_common.h"

#include <string.h>
#include "imu.h"
#include "nrf_drv_gpiote.h"

#include "i2cdev.h"
#include "mpu6500.h"
#include "lps25h.h"
#include "max17043.h"
#include "ms5611.h"

#include "sensfusion6.h"
#include "quadcopter.h"
#include "helper_3dmath.h"

#include "SEGGER_RTT.h"
#include "MPU6050_6Axis_MotionApps20.h"
#define M_PI 3.1415f
#define OUTPUT_YAW_PITCH_ROLL 0

volatile uint32_t ms_ticks = 0;

//Global variables
Axis3i16 accel16, gyro16, mag16;
int16_t gyro_readings[3];
Axis3f accel, gyro, mag;
//float pressure = 0.0f, temperature = 0.0f, asl = 0.0f;
float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;


// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

/**
 * @brief GPIOTE events handler.
 */
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	mpuInterrupt = true;
}

void imu_get_values(Axis3f *gyro, attitude *attitude){
    // flipping values to match wikipedia convention of YPR
    gyro->y = -(float)gyro_readings[0];
    gyro->x = -(float)gyro_readings[1];
    gyro->z = -(float)gyro_readings[2];
    attitude->pitch = -(ypr[2] * 180/M_PI);
    attitude->roll = (ypr[1] * 180/M_PI);
    attitude->yaw = (ypr[0] * 180/M_PI);
	  //printf("%f %f %f\r", pitch, roll, yaw);  //Prints values
}

/**
 * @brief Configure interrupt for DMP
 */
void attach_interrupt(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);
    
    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
    in_config.pull = NRF_GPIO_PIN_PULLDOWN;

    err_code = nrf_drv_gpiote_in_init(INT_IMU, &in_config, dmpDataReady);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(INT_IMU, true);
}

void imu_init(void){
    NRF_LOG_PRINTF("imu_init\n");    
    
    devStatus = imu6Init();
    //devStatus = mpu6500DmpInitialize();
    //all offsets 0 ?
    /*
    mpu6500SetXGyroOffsetUser(0);
    mpu6500SetYGyroOffsetUser(0);
    mpu6500SetZGyroOffsetUser(0);
    
    mpu6500SetXAccelOffset(0);
    mpu6500SetYAccelOffset(0);
    mpu6500SetZAccelOffset(0);
    */
}

void imu_periodic(void){
    if(!mpuInterrupt && fifoCount < packetSize) { //TODO check logic // do stuff when fifo has data
        //SEGGER_RTT_printf(0,"imu_periodic: not executing\n");
        return;
    }
    #if DOING_FREQUENCY_TESTING
        nrf_gpio_pin_toggle(IMU_INT_PIN);
    #endif
    mpuInterrupt = false;
    mpuIntStatus = mpu6500GetIntStatus();
    fifoCount = mpu6500GetFIFOCount();
    
    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
            // reset so we can continue cleanly
            mpu6500ResetFIFO();
            SEGGER_RTT_printf(0,"FIFO overflow!\r\n");

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
            // wait for correct available data length, should be a VERY short wait
            while (fifoCount < packetSize) fifoCount = mpu6500GetFIFOCount();

            // read a packet from FIFO
            mpu6500GetFIFOBytes(fifoBuffer, packetSize);
            
            // track FIFO count here in case there is > 1 packet available TODO
            // (this lets us immediately read more without waiting for an interrupt)
            fifoCount -= packetSize;
            mpu6500DmpGetQuaternion3(&q, fifoBuffer);
            mpu6500DmpGetGravity(&gravity, &q);
            mpu6500DmpGetYawPitchRoll(ypr, &q, &gravity);
            mpu6500DmpGetGyro2(gyro_readings, fifoBuffer); // assuming values in deg/s
            
            #if OUTPUT_YAW_PITCH_ROLL
                // display angles in degrees
                SEGGER_RTT_printf(0,"ypr\t");
                SEGGER_RTT_printf(0,"%d",(int)(ypr[0] * 180/M_PI));
                SEGGER_RTT_printf(0,"\t");
                SEGGER_RTT_printf(0,"%d",(int)(ypr[1] * 180/M_PI));
                SEGGER_RTT_printf(0,"\t");
                SEGGER_RTT_printf(0,"%d \t ",(int)(ypr[2] * 180/M_PI));
                SEGGER_RTT_printf(0,"gyros\t x: %d \t y: %d \t z: %d\n",gyro_readings[0],gyro_readings[1],gyro_readings[2]);
            #endif
    }
    #if DOING_FREQUENCY_TESTING
        nrf_gpio_pin_toggle(IMU_INT_PIN);
    #endif
}
