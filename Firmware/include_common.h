#ifndef INCLUDE_COMMON_H__
#define INCLUDE_COMMON_H__

#include "nrf_error.h"

#define INVALID_SETPOINT 0xFFFF


#include "app_timer.h"
#include <stdint.h>
#include "nrf_log.h"
#include <stdlib.h>
#include <string.h>
#include "SEGGER_RTT.h"
#include "controller.h"
#include "pid.h"

#include "imu_types.h"
#include "nrf_gpio.h"
#include "pm.h"



//config
#define STAB_FREQUENCY 400 // 200 hz 
#define IMU_UPDATE_DT (float)(1.0 / STAB_FREQUENCY)
#define JOYSTICK_MIDDLEPOINT 127
#define LED1_GREEN 19

#define DOING_FREQUENCY_TESTING 0
    #if DOING_FREQUENCY_TESTING
    #define IMU_INT_PIN 14
    #define RADIO_INT_PIN 15
    #define STABILIZER_PIN 16
    #define RATE_PIN 17
    #warning DOING_FREQUENCY_TESTING
#endif

typedef  float euler_angle;

typedef struct {
    euler_angle roll;
    euler_angle pitch;
    euler_angle yaw;
}attitude;

#undef max
#define max(a,b) ((a) > (b) ? (a) : (b))
#undef min
#define min(a,b) ((a) < (b) ? (a) : (b))


#endif //INCLUDE_COMMON_H__
