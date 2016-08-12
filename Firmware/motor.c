#include "motor.h"
#include "nrf_drv_pwm.h"
#include "nrf_drv_clock.h"

#define MOTOR_DEBUG_MODE 0
uint8_t toggle = 0;

#define DISABLE_MOTORS DOING_FREQUENCY_TESTING
#if DISABLE_MOTORS
    #warning "Motors are disabled by DISABLE_MOTORS"
#endif

//printing
#define PRINT_MOTOR_OUTPUT 0

void motor_testing(void);
void pwm_event_handler (nrf_drv_pwm_evt_type_t event_type);
void pwm_start(void);
void pwm_init(void);
void pwm_update(void);

//int pwm_scaler = (UINT16_MAX / PWM_TOP );
float pwm_scaler = (65535.0/PWM_TOP);

static nrf_drv_pwm_t m_pwm0 = NRF_DRV_PWM_INSTANCE(0);
static motor_values current_values;
static nrf_pwm_values_common_t pwm_values[] =
    {
        10,20,40,80
    };
    
static  nrf_pwm_sequence_t const pwm_seq =
    {
        .values.p_common = pwm_values,
        .length          = NRF_PWM_VALUES_LENGTH(pwm_values),
        .repeats         = 0,
        .end_delay       = 0
    };

void motor_init(void){
    NRF_LOG("motor_init()\n");
    #if DISABLE_MOTORS
        return;
    #endif
    //motor_testing();
    memset(&current_values,0,sizeof(current_values));
    pwm_init();
    //pwm_start();
}

void motor_values_update(motor_values values){
    // save these values, and update them using the pwm event handler
    current_values = values;
    #if PRINT_MOTOR_OUTPUT 
    NRF_LOG_PRINTF("Motors: %d ## %d ## %d ## %d ",(int)current_values.motor1,(int)current_values.motor2,(int)current_values.motor3,(int)current_values.motor4);
    #endif
    //NRF_LOG_PRINTF("pwm_scaler = %d \n", pwm_scaler);
    //scale to pwm size
    current_values.motor1 = current_values.motor1 / pwm_scaler;
    current_values.motor2 = current_values.motor2 / pwm_scaler;
    current_values.motor3 = current_values.motor3 / pwm_scaler;
    current_values.motor4 = current_values.motor4 / pwm_scaler;
    #if PRINT_MOTOR_OUTPUT 
    NRF_LOG_PRINTF(" Motors After scaling: %d ## %d ## %d ## %d \n",(int)current_values.motor1,(int)current_values.motor2,(int)current_values.motor3,(int)current_values.motor4);
    #endif
    #if DISABLE_MOTORS
        return;
    #endif
    pwm_update();
}    

void pwm_init(void){
    uint32_t err_code;
    nrf_drv_pwm_config_t const config0 =
    {
        .output_pins =
        {
            MOTOR_PIN1,             // channel 0
            MOTOR_PIN2,             // channel 1
            MOTOR_PIN3,             // channel 2
            MOTOR_PIN4,             // channel 3
        },
        .base_clock = NRF_PWM_CLK_16MHz,
        .count_mode = NRF_PWM_MODE_UP,
        .top_value  = PWM_TOP, // reaching top with 40 khz 
        .load_mode  = NRF_PWM_LOAD_INDIVIDUAL,
        .step_mode  = NRF_PWM_STEP_AUTO 
    };
    err_code = nrf_drv_pwm_init(&m_pwm0, &config0, NULL);
    APP_ERROR_CHECK(err_code);
}

void pwm_start(void){
    nrf_drv_pwm_simple_playback(&m_pwm0, &pwm_seq, 1, 0 );
}

void pwm_update(void){
    //NRF_LOG_PRINTF("pwm_update\n");
    #if MOTOR_DEBUG_MODE
        // do not update
        if (toggle == 1){
            pwm_values[3] = 10;
            toggle = 0;
        }
        else if (toggle == 0){
            pwm_values[3] = 80;
            toggle = 1;
        }
        pwm_start();
        return;
    #endif
    // | with 0x8000 to invert pwm signal
    uint16_t temp_value = 0;
    temp_value= (uint16_t)current_values.motor1;
    pwm_values[0] = temp_value | 0x8000 ;
    temp_value= (uint16_t)current_values.motor2;
    pwm_values[1] = temp_value | 0x8000 ;
    temp_value= (uint16_t)current_values.motor3;
    pwm_values[2] = temp_value | 0x8000 ;
    temp_value= (uint16_t)current_values.motor4;
    pwm_values[3] = temp_value | 0x8000 ;
        
    pwm_start();
    
}

// not used
void pwm_event_handler (nrf_drv_pwm_evt_type_t event_type){
    
    if (event_type == NRF_DRV_PWM_EVT_FINISHED){
        pwm_update();
    }
}

void motor_testing(void){
    static nrf_pwm_values_common_t seq_values[] =
    {
        100 | 0x8000, 200 | 0x8000, 300 | 0x8000, 350 | 0x8000
    };
    nrf_pwm_sequence_t const seq =
    {
        .values.p_common = seq_values,
        .length          = NRF_PWM_VALUES_LENGTH(seq_values),
        .repeats         = 0,
        .end_delay       = 0
    };
    nrf_drv_pwm_simple_playback(&m_pwm0, &seq, 1, NRF_DRV_PWM_FLAG_LOOP);
}


uint32_t motor_stop(void){
    uint32_t err_code;
    err_code = nrf_drv_pwm_stop	(&m_pwm0,false);
    return err_code;
}
