#include "stabilizer.h"
#include "motor.h"
//#include "imu_toplevel.h"
#include "imu.h"
#include "mpu6500.h"
#include "MS5611.h"
#include "lps25h.h"
#include "quadcopter.h"
#include "sensfusion6.h"

APP_TIMER_DEF(stab_timer_id);
#define PRESCALER 0

//printing
#define PRINT_ACTUATOR_OUTPUT           0
#define PRINT_CMD_INPUT                 0
#define PRINT_RECEIVED_IMU_VALUES       0
#define PRINT_ACC_CALIBRATED_VALUES			0
#define PRINT_EULER_ANGLES							0
#define PRINT_RAW_GYRO                  0
#define PRINT_SCALED_GYRO               0
#define PRINT_TASK_EXECUTION            0
#define PRINT_MOTOR_POWER								0
#define PRINT_CONTROL_SETPOINTS					0
#define PRINT_BAROMETER_VALUES					0
#define PRINT_CURRENT_ALTITUDE					0

//config
#define TUNING_MODE_ROLL                0
#define TUNING_MODE_PITCH               0
#define TUNING_MODE_YAW                 0
#define USING_DUMMY_THRUST              0
static bool AUTO_LEVEL_ENABLED				=	true; //Should be true
static bool ALTITUDE_CONTROL_ENABLED  =	false; //Should be false

static bool calibrate_gyro = false;
static bool calibrate_asl = false;


//scheduling
#define SCHED_FREQUENCY 500 //hz
#define SCHED_FREQUENCY_DT (float)(1.0/SCHED_FREQUENCY)
#define SCHED_INTERVAL_MS (1000/SCHED_FREQUENCY)

#define ATTITUDE_CONTROLLER_FREQUENCY (SCHED_FREQUENCY/2) //hz
#define ATTITUDE_DIVIDER (SCHED_FREQUENCY/ATTITUDE_CONTROLLER_FREQUENCY)
#define ATTITUDE_UPDATE_DT 1.0f/((float)ATTITUDE_CONTROLLER_FREQUENCY)

#define RATE_CONTROLLER_FREQUENCY SCHED_FREQUENCY //hz
#define RATE_DIVIDER (SCHED_FREQUENCY/RATE_CONTROLLER_FREQUENCY)

#define LED_FREQUENCY 5
#define LED_TOGGLE_DIVIDER (SCHED_FREQUENCY/LED_FREQUENCY)

#define GRAVITY 	9.80665f

/*static attitude gyro_actual;
static attitude attitude_actual;
static attitude attitude_setpoint; */ //Declared, but never referenced. 3 Warnings.

static float eulerRollActual;   // Measured roll angle in deg
static float eulerPitchActual;  // Measured pitch angle in deg
static float eulerYawActual;    // Measured yaw angle in deg
static float eulerRollDesired;  // Desired roll angle in deg
static float eulerPitchDesired; // Desired pitch angle in deg
static float eulerYawDesired;   // Desired yaw angle in deg
static float rollRateDesired;   // Desired roll rate in deg/s
static float pitchRateDesired;  // Desired pitch rate in deg/s
static float yawRateDesired;    // Desired yaw rate in deg/s
static float altitudeDesired;		// Desired altitude

//static Axis3i16 gyro_16bit; // Gyro axis data in deg/s ********Declared, never referenced. 1 Warning.
static Axis3f gyro; // Gyro axis data in deg/s
static Axis3f acc;  // Accelerometer axis data in mG

float  actuatorThrust;  // Actuator output for thrust base
float  actuatorRoll;    // Actuator output roll compensation
float  actuatorPitch;   // Actuator output pitch compensation
float  actuatorYaw;     // Actuator output yaw compensation

float nominal_actuatorThrust; // Nominal actuator output for thrust base
float thrust_down_value = 0;

float motorPowerM1;  // Motor 1 power output (16bit value used: 0 - 65535)
float motorPowerM2;  // Motor 2 power output (16bit value used: 0 - 65535)
float motorPowerM3;  // Motor 3 power output (16bit value used: 0 - 65535)
float motorPowerM4;  // Motor 4 power output (16bit value used: 0 - 65535)

float asl; 						//Altitude above sea level through the complementary filter.
float comp_filt_coeff = 0.00002f; //Coefficient which determines the ratio between the complementary values used in the filter.

float pressure_ms; 		 // Pressure value received from the ms5611 barometer.
float temperature_ms;	 // Temperature value received from the ms5611 barometer.
float asl_ms;					 // Altitude value above sea level, received from the ms5611 barometer.
float asl_current_ms;		//Current altitude above reference point

float pressure_lps;   // Pressure value received from the lps25h barometer.
float temperature_lps; // Temperature value received from the lps25h barometer.
float asl_lps;				// Altitude value above sea level, received from the lps25h barometer.
float asl_current_lps;				// Altitude value above sea level, received from the lps25h barometer.

float print_pressure;
float print_temperature;
float print_asl;

float print_pressure_lps;
float print_temperature_lps;
float print_asl_lps;

bool get_desired_altitude = true;
bool user_control_quad = true;
bool stop_motors = false;
uint32_t stab_timer_count = 0;

uint16_t get_offset_acc = 0;
uint16_t get_offset_gyro = 0;
uint8_t get_offset_asl = 0;

float gyro_offset_x = 0.0F, gyro_offset_y = 0.0F, gyro_offset_z = 0.0F;

// Scaling from 16 bit to angle/s
float gyro_scaler = 500.0/32768.0;


static void scheduler_execute(void * arg);
//static float constrain(float value, const float minVal, const float maxVal); Declared, never referenced. 1 Warning.
//static float deadband(float value, const float threshold); Declared, never referenced. 1 Warning.
static float limitThrust(float value);
static void distributePower(const float thrust, const float roll,
                            const float pitch, const float yaw);

void stab_init(){
    NRF_LOG("stab_init()\n");
    rollRateDesired = 0;
    pitchRateDesired = 0;
    yawRateDesired = 0;
    controllerInit();

    uint32_t err_code;
    err_code = app_timer_create(&stab_timer_id, APP_TIMER_MODE_REPEATED, scheduler_execute);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_start(stab_timer_id, APP_TIMER_TICKS(SCHED_INTERVAL_MS, PRESCALER) ,NULL);
    APP_ERROR_CHECK(err_code);
}

/*static void imu_get_values_dummy(Axis3f *gyro, attitude *attitude){
    gyro->x = 0;
    gyro->y = 0;
    gyro->z = 0;
    attitude->roll = 0;
    attitude->pitch = 0;
    attitude->yaw = 0;
}*/
void fail_safe_handler(void){
	user_control_quad = false;
	if(thrust_down_value <= actuatorThrust){
		thrust_down_value += (actuatorThrust * 0.001F);
	  SEGGER_RTT_printf(0, "Stopping motors, slow descent. \n");
	}
	
	distributePower((actuatorThrust - thrust_down_value), 0, 0, 0);

}

void fault_stop_motors(void){
	user_control_quad = false;
	stop_motors = true;
	distributePower(0,0,0,0);
	SEGGER_RTT_printf(0, "Stopping motors, immediate descent. \n");
}

void set_user_control(void){
	user_control_quad = true;
}

float cmd_get_throttle_dummy(void){
    return 38000; // finding hover thrust, 38k is good
}

void cmd_set_reg_mode(uint8_t* mode_data){
	if(mode_data[11] == 0)
		AUTO_LEVEL_ENABLED = true;
	else if(mode_data[11] == 1)
		AUTO_LEVEL_ENABLED = false;
  if(mode_data[12] == 0)
    ALTITUDE_CONTROL_ENABLED = false;
  else if(mode_data[12] == 1)
    ALTITUDE_CONTROL_ENABLED = true;
}

void calibrate_current_altitude(void){
	if(get_offset_asl < 255){
		if(get_offset_asl == 0){
			asl_current_ms = 0;
			asl_current_lps = 0;
			SEGGER_RTT_printf(0,"Calibrating.. \n");
		}
		
		ms5611GetData(&pressure_ms, &temperature_ms, &asl_ms);
		lps25hGetData(&pressure_lps, &temperature_lps, &asl_lps);
		asl_current_ms += asl_ms;
		asl_current_lps += asl_lps;
		
		get_offset_asl++;
		

		if(get_offset_asl % 5 == 0)
				led_blink();
	}
	else{
		calibrate_asl = false;
		
		asl_current_ms = asl_current_ms / 255;
		asl_current_lps = asl_current_lps / 255;

		get_offset_asl = 0;
		
		led_blink_clear();
	  SEGGER_RTT_printf(0,"Done! \n");
	}
}

void run_altitude_controller(){
  ms5611GetData(&pressure_ms, &temperature_ms, &asl_ms);
	lps25hGetData(&pressure_lps, &temperature_lps, &asl_lps);
	
	// Complementary filter
	asl = (comp_filt_coeff * (asl_ms - asl_current_ms)) + ((1 - comp_filt_coeff) * (asl_lps - asl_current_lps));
	
  if (get_desired_altitude == true){
    altitudeDesired = asl;
    get_desired_altitude = false;
  }
  controllerCorrectAltitudePID_FLOAT(asl, altitudeDesired);
	#if PRINT_CURRENT_ALTITUDE
		SEGGER_RTT_printf(0,"Current altitude: %d \n", (int)(asl*1000));
	#endif
}

float acc_offset_x = 0.0F, acc_offset_y = 0.0F, acc_offset_z = 0.0F;

void run_attitude_controller(attitude setpoints){
    #if DOING_FREQUENCY_TESTING
        nrf_gpio_pin_toggle(STABILIZER_PIN);
    #endif
    #if PRINT_TASK_EXECUTION
        SEGGER_RTT_printf(0,"run_attitude_controller\n");
    #endif
    eulerRollDesired = setpoints.roll;
    eulerPitchDesired = setpoints.pitch;
    eulerYawDesired = setpoints.yaw;

	  imu6Read(&gyro, &acc);
	
    #if PRINT_RECEIVED_IMU_VALUES
        SEGGER_RTT_printf(0,"imu_get_values: ax: %d\t ay: %d \t az: %d\t gx: %d\t gy: %d \t gz: %d\n",(int)(acc.x*1000), (int)(acc.y*1000),(int)(acc.z*1000),(int)gyro.x,(int)gyro.y,(int)gyro.z);
    #endif
				
		sensfusion6UpdateQ(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z, (ATTITUDE_UPDATE_DT*1.0f));		
		sensfusion6GetEulerRPY(&eulerRollActual, &eulerPitchActual, &eulerYawActual);
				
		#if PRINT_EULER_ANGLES
        SEGGER_RTT_printf(0,"Euler angles: Roll: %d\t Pitch: %d \t Yaw: %d\t \n",(int)eulerRollActual, (int)eulerPitchActual, (int)eulerYawActual);
    #endif
				
    //update attitude PID
    controllerCorrectAttitudePID(eulerRollActual, eulerPitchActual, eulerYawActual,
                                     eulerRollDesired, eulerPitchDesired, eulerYawDesired,
                                     &rollRateDesired, &pitchRateDesired, &yawRateDesired);
																		 
		#if DOING_FREQUENCY_TESTING
        nrf_gpio_pin_toggle(STABILIZER_PIN);
    #endif
}

void run_rate_controller(attitude setpoints){
    #if DOING_FREQUENCY_TESTING
        nrf_gpio_pin_toggle(RATE_PIN);
    #endif
    #if PRINT_TASK_EXECUTION
        SEGGER_RTT_printf(0,"run_rate_controller\n");
    #endif
    int16_t gx;
    int16_t gy;
    int16_t gz;
    mpu6500GetRotation(&gx , &gy, &gz);

    #if PRINT_RAW_GYRO
        SEGGER_RTT_printf(0,"raw gyro: gx: %d \t gy: %d \t gz: %d\t \n",gx,gy,gz);
    #endif

	  //Calibrating initial gyro offset.
		if(get_offset_gyro < 2000 && calibrate_gyro == true){
			if(get_offset_gyro == 0){
				gyro_offset_x = 0;
				gyro_offset_y = 0;
				gyro_offset_z = 0;
			}

			gyro_offset_x += gx;
			gyro_offset_y += gy;
			gyro_offset_z += gz;

			get_offset_gyro++;
		}
		else if(get_offset_gyro == 2000 && calibrate_gyro == true){
			gyro_offset_x = (gyro_offset_x / 2000.0f);
			gyro_offset_y = (gyro_offset_y / 2000.0f);
			gyro_offset_z = (gyro_offset_z / 2000.0f);

			get_offset_gyro = 0;
			calibrate_gyro = false;
		}

		//switching x and y, and inverting signs
		if(calibrate_gyro == true){
		  gyro.y = -(float)gx*gyro_scaler;
			gyro.x = -(float)gy*gyro_scaler;
			gyro.z = -(float)gz*gyro_scaler;
		}
		else{
			gyro.y = -(float)(gx - gyro_offset_x)*gyro_scaler;
			gyro.x = -(float)(gy - gyro_offset_y)*gyro_scaler;
			gyro.z = -(float)(gz - gyro_offset_z)*gyro_scaler;
		}


    #if PRINT_SCALED_GYRO
		SEGGER_RTT_printf(0,"scaled gyro: gx: %d \t gy: %d \t gz: %d \t gyro offset: gx_off: %d \t gy_off: %d \t gz_off: %d \n",(int)gyro.x,(int)gyro.y,(int)gyro.z,(int)gyro_offset_x,(int)gyro_offset_y,(int)gyro_offset_z);
    #endif
    controllerCorrectRatePID_FLOAT(gyro.x, gyro.y, gyro.z,rollRateDesired, pitchRateDesired, yawRateDesired);
    #if DOING_FREQUENCY_TESTING
        nrf_gpio_pin_toggle(RATE_PIN);
    #endif
}

void set_calibration(void){
	calibrate_gyro = true;
	calibrate_asl = true;
}


static void scheduler_execute(void * arg){

    static int loop_counter = 0;
    loop_counter++;
    //SEGGER_RTT_printf(0,"\t loop_counter: %d \t",loop_counter);

	  static attitude setpoints;
    static attitude rate_setpoints;

		//Gets setpoints sent from the user.
	  setpoints = cmd_get_setpoints();
	  rate_setpoints.yaw = setpoints.yaw; //Since we are using only the yaw-rate controller, the setpoint of the yaw-attitude controller becomes our rate setpoint.
		if(!AUTO_LEVEL_ENABLED){
			rate_setpoints.pitch = setpoints.pitch; //If we are in pure rate mode, the setpoints for the attitude controllers becomes the setpoint for the rate controllers.
			rate_setpoints.roll = setpoints.roll;
		}

    #if PRINT_CMD_INPUT
        SEGGER_RTT_printf(0,"throttle sp: %d roll sp: %d , pitch sp: %d yaw sp: %d  ",
        (int)actuatorThrust,(int)setpoints.roll,(int)setpoints.pitch,(int)setpoints.yaw);
    #endif
		
		//Checks if the quad is in attitude mode. If so, performs the normal control loop.
    if (AUTO_LEVEL_ENABLED){			
			if ((loop_counter % ATTITUDE_DIVIDER) == 0){
        run_attitude_controller(setpoints);
        rate_setpoints.roll = rollRateDesired;
        rate_setpoints.pitch = pitchRateDesired;
        yawRateDesired = rate_setpoints.yaw;
			}
			  run_rate_controller(rate_setpoints);
		}
		//Checks if the quad is in rate mode. If so, the program will only execute the rate loop.
		else{
		  rollRateDesired = rate_setpoints.roll;
      pitchRateDesired = rate_setpoints.pitch;
      yawRateDesired = rate_setpoints.yaw;
      run_rate_controller(rate_setpoints);
    }
		//Checks if the user is in altitude mode. If so, the program will also perform the altitude controller loop.
		if (ALTITUDE_CONTROL_ENABLED && loop_counter % 15 == 0 && !calibrate_asl){
			run_altitude_controller();
			#if PRINT_BAROMETER_VALUES
			print_pressure = pressure_ms * 1000;
			print_temperature = temperature_ms * 1000;
			print_asl = asl * 1000;
			
			print_pressure_lps = pressure_lps * 1000;
			print_temperature_lps = temperature_lps * 1000;
			print_asl_lps = asl_lps * 1000;
			
			SEGGER_RTT_printf(0,"MS5611:: Pressure: %d - Temperature: %d  - Altitude (ASL) %d LPS25H:: Pressure: %d - Temperature: %d  - Altitude (ASL) %d \n", (int)print_pressure, (int)print_temperature, (int)print_asl, (int)print_pressure_lps, (int)print_temperature_lps, (int)print_asl_lps);
			#endif
		}
		if (calibrate_asl && loop_counter % 4 == 0){
			calibrate_current_altitude();
		}
    if (!ALTITUDE_CONTROL_ENABLED){
      get_desired_altitude = true; //Check value that enables the quad to set it current altitude as its setpoint, immediately after transition from attitude mode to altitude mode.
    }

    #if PRINT_CONTROL_SETPOINTS
    SEGGER_RTT_printf(0,"Roll (Angle): %d - Pitch (Angle): %d  - Yaw (Angle/s): %d - Throttle: %d \n",(int)setpoints.roll,(int)setpoints.pitch,(int)rate_setpoints.yaw, (int)actuatorThrust);
    #endif

    //Get actuator output for roll, pitch and yaw.
    controllerGetActuatorOutput_FLOAT(&actuatorRoll, &actuatorPitch, &actuatorYaw);
		//If in hover mode, get actuator output for thrust.
		if(ALTITUDE_CONTROL_ENABLED) {
			controllerGetThrustOutput_FLOAT(&actuatorThrust);
			actuatorThrust = actuatorThrust * 100.0f; //Scales the actuatorThrust, delivered from the altitude controller.
		}
		//If not in hover mode, get actuatorThrust from EX-characteristic (call cmd_get_throttle)
		else if (!ALTITUDE_CONTROL_ENABLED) {
			nominal_actuatorThrust = actuatorThrust; //Sets nominal thrust equals previous thrust. Prepares the controller to go from attitude mode to hover mode.
			actuatorThrust = cmd_get_throttle();
		}
		
    #if TUNING_MODE_ROLL
    actuatorPitch = 0;
    actuatorYaw = 0;
    #endif
    #if TUNING_MODE_PITCH
    actuatorYaw = 0;
    actuatorRoll = 0;
    #endif
    #if TUNING_MODE_YAW
    actuatorPitch = 0;
    actuatorRoll = 0;
    #endif
		
    if(PRINT_ACTUATOR_OUTPUT){
			if(!ALTITUDE_CONTROL_ENABLED){
				SEGGER_RTT_printf(0,"Actuator output: roll: %d\t pitch: %d\t yaw: %d\t throttle: %d \t current altitude: %d \n\n",(int)actuatorRoll,(int)actuatorPitch,(int)actuatorYaw, (int)actuatorThrust, (int)asl);
			}
			else{
				SEGGER_RTT_printf(0,"Actuator output: roll: %d\t pitch: %d\t yaw: %d\t throttle: %d \t current altitude: %d \n\n",(int)actuatorRoll,(int)actuatorPitch,(int)actuatorYaw, (int)(actuatorThrust+nominal_actuatorThrust), (int)(asl*1000));
			}
		}

    //Calculate motor output, checks if the user is in control of the quadcopter.
		if (user_control_quad && !stop_motors){
			// Normal run, attitude enabled and altitude disabled.
			if (actuatorThrust > 0 && !ALTITUDE_CONTROL_ENABLED)
				{
					thrust_down_value = 0.5F * actuatorThrust; //Prepares the intial value of the fail safe handler.
					distributePower(actuatorThrust, actuatorRoll, actuatorPitch, actuatorYaw);
				}
			//Hover mode enabled. Checks if nominal actuatorThrust is higher than 0, and if so, distribute hover thrust.	
			else if (nominal_actuatorThrust > 0 && ALTITUDE_CONTROL_ENABLED)
				{
					thrust_down_value = 0.5F * nominal_actuatorThrust; //Prepares the intial value of the fail safe handler.
					distributePower((actuatorThrust + nominal_actuatorThrust), actuatorRoll, actuatorPitch, actuatorYaw); //TODO: Check if max/min value of float becomes an issue with thrust. 
				}
			//If actuatorThrust is 0 in attitude mode, or if nominal actuatorThrust is 0 in hover mode, distribute no power to motors. (Important)
		  else if ((!ALTITUDE_CONTROL_ENABLED && actuatorThrust == 0) || (ALTITUDE_CONTROL_ENABLED && nominal_actuatorThrust == 0))
				{
					distributePower(0, 0, 0, 0);
				}
			//In case something happens that I haven't thought about, reset all PIDs and stop all motors!
			else
				{
					controllerResetAllPID();
					distributePower(0, 0, 0, 0);
				}
		}
		//Extreme case, some part of the application have faulted. Immediate descent.
		else if(stop_motors){
			fault_stop_motors(); 
		}
		//If user is not in control of the quad, call fail_safe_handler. Slow descent.
		else{
			fail_safe_handler();
		}

    //send motor values to pwm generation
    motor_values values;
    values.motor1 = motorPowerM1;
    values.motor2 = motorPowerM2;
    values.motor3 = motorPowerM3;
    values.motor4 = motorPowerM4;
    motor_values_update(values);
		#if PRINT_MOTOR_POWER
    SEGGER_RTT_printf(0, "MotorPower: %d - %d - %d - %d \n",(int)motorPowerM1,(int)motorPowerM2,(int)motorPowerM3,(int)motorPowerM4);
    #endif
    //look for new IMU data. Doing this here to not delay periodic tasks.
    //imu_periodic();
		
		//imu_get_values_dummy(&gyro, &attitude_actual);
}


static void distributePower(const float thrust, const float roll,
                            const float pitch, const float yaw)
{

		float r = roll;
		float p = pitch;
		motorPowerM1 = limitThrust(thrust + r + p - yaw);
		motorPowerM2 = limitThrust(thrust - r + p + yaw);
		motorPowerM3 =  limitThrust(thrust - r - p - yaw);
		motorPowerM4 =  limitThrust(thrust + r - p + yaw);

}

static float limitThrust(float value)
{
  if(value > UINT16_MAX)
  {
    value = UINT16_MAX;
  }
  else if(value < 0)
  {
    value = 0;
  }
  return value;
}

// Constrain value between min and max. Never used, but I let it be.
/*static float constrain(float value, const float minVal, const float maxVal)
{
  return min(maxVal, max(minVal,value));
}*/
