#ifndef QUADCOPTER_H
#define QUADCOPTER_H

#include "boards.h"

//nRF52 Quadcopter defines
#define SWITCH		28

#define LED1_R 		20
#define LED1_G 		19
#define LED1_B 		21

#define LED2_R 		28
#define LED2_G 		31
#define LED2_B 		30

#define LED3_R 		25
#define LED3_G 		27
#define LED3_B 		26

#define LED4_R 		23
#define LED4_G 		22
#define LED4_B 		24

#define MOTOR1 		14
#define MOTOR2 		15
#define MOTOR3 		16
#define MOTOR4 		17

#define SDA 			12
#define SCL				13

//BQ24075
#define PM_CHG		2
#define PM_CH_EN	3
#define PM_ISET		4
#define PM_EN2		5
#define PM_EN1		6
#define PM_PGOOD	7

//MAX17043
#define BAT_ALRT	8

//MPU9250
#define INT_IMU		11

void motor_config(void);
void led_config(void);
void led_test(void);
void led_connected(void);
void led_advertising(void);
void led_blink(void);
void led_blink_clear(void);

#endif
