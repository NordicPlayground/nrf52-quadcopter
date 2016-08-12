#include "quadcopter.h"

void motor_config(void) 
{
	nrf_gpio_cfg_output(MOTOR1);
	nrf_gpio_cfg_output(MOTOR2);
	nrf_gpio_cfg_output(MOTOR3);
	nrf_gpio_cfg_output(MOTOR4);

	//Disable all motors
	nrf_gpio_pin_clear(MOTOR1);
	nrf_gpio_pin_clear(MOTOR2);
	nrf_gpio_pin_clear(MOTOR3);
	nrf_gpio_pin_clear(MOTOR4);
}

void led_config(void) 
{
	nrf_gpio_cfg_output(LED1_G);
	nrf_gpio_cfg_output(LED2_G);
	nrf_gpio_cfg_output(LED3_G);
	nrf_gpio_cfg_output(LED4_G);

	nrf_gpio_cfg_output(LED1_R);
	nrf_gpio_cfg_output(LED2_R);
	nrf_gpio_cfg_output(LED3_R);
	nrf_gpio_cfg_output(LED4_R);

	nrf_gpio_cfg_output(LED1_B);
	nrf_gpio_cfg_output(LED2_B);
	nrf_gpio_cfg_output(LED3_B);
	nrf_gpio_cfg_output(LED4_B);

	nrf_gpio_pin_set(LED1_G);
	nrf_gpio_pin_set(LED2_G);
	nrf_gpio_pin_set(LED3_G);
	nrf_gpio_pin_set(LED4_G);

	nrf_gpio_pin_set(LED1_R);
	nrf_gpio_pin_set(LED2_R);
	nrf_gpio_pin_set(LED3_R);
	nrf_gpio_pin_set(LED4_R);

	nrf_gpio_pin_set(LED1_B);
	nrf_gpio_pin_set(LED2_B);
	nrf_gpio_pin_set(LED3_B);
	nrf_gpio_pin_set(LED4_B);
}

void led_connected(void)
{
	led_config();
	
	nrf_gpio_pin_clear(LED1_R);
	nrf_gpio_pin_clear(LED2_G);
}

void led_advertising(void)
{     		
		  nrf_gpio_pin_toggle(LED1_B);
			nrf_gpio_pin_toggle(LED2_B);
}

void led_blink(void)
{
	nrf_gpio_pin_toggle(LED4_R);
	nrf_gpio_pin_toggle(LED4_B);
	nrf_gpio_pin_toggle(LED4_G);
}

void led_blink_clear(void)
{
	nrf_gpio_pin_set(LED4_R);
	nrf_gpio_pin_set(LED4_G);
	nrf_gpio_pin_set(LED4_B);
}
