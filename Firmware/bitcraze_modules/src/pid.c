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
 * pid.c - implementation of the PID regulator
 */

#include "pid.h"
#include <stdint.h>
#include <string.h>

#include "include_common.h"
#include "command_input.h"

uint8_t default_values[20];

uint8_t* get_default(void){
	
	default_values[1] = PID_ROLL_RATE_KP;
	default_values[2] = PID_ROLL_RATE_KI;
	default_values[3] = PID_ROLL_RATE_KD;
	
	default_values[4] = PID_PITCH_RATE_KP;
	default_values[5] = PID_PITCH_RATE_KI;
	default_values[6] = PID_PITCH_RATE_KD;
	
	default_values[7] = PID_YAW_RATE_KP;
	default_values[8] = PID_YAW_RATE_KI;
	default_values[9] = PID_YAW_RATE_KD;
	
	default_values[10] = PID_ROLL_KP;
	default_values[11] = PID_ROLL_KI;
	default_values[12] = PID_ROLL_KD;
	
	default_values[13] = PID_PITCH_KP;
	default_values[14] = PID_PITCH_KI;
	default_values[15] = PID_PITCH_KD;
	
	default_values[16] = PID_YAW_KP;
	default_values[17] = PID_YAW_KI;
	default_values[18] = PID_YAW_KD;
	
	default_values[19] = 66;
	
	return default_values;
}

void pidInit(PidObject* pid, const float desired, const float kp,
             const float ki, const float kd, const float dt)
{
  pid->error     = 0;
  pid->prevError = 0;
  pid->integ     = 0;
  pid->deriv     = 0;
  pid->desired   = desired;
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->iLimit    = DEFAULT_PID_INTEGRATION_LIMIT;
  pid->iLimitLow = -DEFAULT_PID_INTEGRATION_LIMIT;
  pid->dt        = dt;
}

float pidUpdate(PidObject* pid, const float measured, const bool updateError)
{
    float output;

    if (updateError)
    {
        pid->error = pid->desired - measured;
    }

    pid->integ += pid->error * pid->dt;
    if (pid->integ > pid->iLimit)
    {
        pid->integ = pid->iLimit;
    }
    else if (pid->integ < pid->iLimitLow)
    {
        pid->integ = pid->iLimitLow;
    }

    pid->deriv = (pid->error - pid->prevError) / pid->dt;

    pid->outP = pid->kp * pid->error;
    pid->outI = pid->ki * pid->integ / INTEGRAL_SCALER;
    pid->outD = pid->kd * pid->deriv / DERIVATOR_SCALER;

    output = pid->outP + pid->outI + pid->outD;

    pid->prevError = pid->error;

		//SEGGER_RTT_printf(0,"Reg values: ref %d - mesu %d - error %d \n",(int)pid->desired, (int)measured, (int)pid->error);

    return output;
}

void pidSetIntegralLimit(PidObject* pid, const float limit) {
    pid->iLimit = limit;
}


void pidSetIntegralLimitLow(PidObject* pid, const float limitLow) {
    pid->iLimitLow = limitLow;
}

void pidReset(PidObject* pid)
{
  pid->error     = 0;
  pid->prevError = 0;
  pid->integ     = 0;
  pid->deriv     = 0;
}

void pidSetError(PidObject* pid, const float error)
{
  pid->error = error;
}

void pidSetDesired(PidObject* pid, const float desired)
{
  pid->desired = desired;
}

float pidGetDesired(PidObject* pid)
{
  return pid->desired;
}

bool pidIsActive(PidObject* pid)
{
  bool isActive = true;

  if (pid->kp < 0.0001F && pid->ki < 0.0001F && pid->kd < 0.0001F)
  {
    isActive = false;
  }

  return isActive;
}

void pidSetKp(PidObject* pid, const float kp)
{
  pid->kp = kp;
	//SEGGER_RTT_printf(0,"I have set Kp! %d\t",(int)pid->kp);
}

void pidSetKi(PidObject* pid, const float ki)
{
  pid->ki = ki;
	//SEGGER_RTT_printf(0,"I have set Ki! %d\t",(int)pid->ki);
}

void pidSetKd(PidObject* pid, const float kd)
{
  pid->kd = kd;
	//SEGGER_RTT_printf(0,"I have set Kd! %d\t",(int)pid->ki);
}
void pidSetDt(PidObject* pid, const float dt) {
    pid->dt = dt;
}
