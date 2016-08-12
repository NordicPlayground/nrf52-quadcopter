#include "motor.h"
#include "nrf_drv_pwm.h"
#include "nrf_drv_clock.h"

static nrf_drv_pwm_t m_pwm0 = NRF_DRV_PWM_INSTANCE(0);
void motor_testing(void);

void motor_init(void){
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
        .base_clock = NRF_PWM_CLK_125kHz,
        .count_mode = NRF_PWM_MODE_UP,
        .top_value  = 1000,
        .load_mode  = PWM_DECODER_LOAD_Individual,
        .step_mode  = NRF_PWM_STEP_TRIGGERED //run until triggered task is executed (never)
    };
    err_code = nrf_drv_pwm_init(&m_pwm0, &config0, NULL);
    APP_ERROR_CHECK(err_code);
    
    motor_testing();
}

void motor_testing(void){
    static nrf_pwm_values_common_t seq_values[] =
    {
        100,200,300,400
    };
    nrf_pwm_sequence_t const seq =
    {
        .values.p_common = seq_values,
        .length          = NRF_PWM_VALUES_LENGTH(seq_values),
        .repeats         = 0,
        .end_delay       = 0
    };
    nrf_drv_pwm_simple_playback(&m_pwm0, &seq, 1, 0);
}


uint32_t motor_stop(void){
    uint32_t err_code;
    err_code = nrf_drv_pwm_stop	(&m_pwm0,false);
    return err_code;
}