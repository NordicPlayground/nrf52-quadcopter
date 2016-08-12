#ifndef MOTOR_H__
#define MOTOR_H__

#include "include_common.h"

#define MOTOR_PIN1 14 // Front left
#define MOTOR_PIN2 15 // front right
#define MOTOR_PIN3 17 //back right
#define MOTOR_PIN4 16 // back left

#define PWM_TOP 800 // count to 800 / 20khz

typedef struct {
    float motor1;
    float motor2;
    float motor3;
    float motor4;
} motor_values ;


void motor_init(void);
void motor_values_update(motor_values values);


#endif //MOTOR_H__
