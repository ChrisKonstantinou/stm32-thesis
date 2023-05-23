#include "pwm_control.h"
#include "pid.h"

PIDController *controler;


uint16_t PWM_GenerateControlSignal(uint16_t input_data)
{
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);



	return 200;
}
