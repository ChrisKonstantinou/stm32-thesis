#include "pwm_control.h"
#include "pid.h"

void PWM_SetPercentage(float percent, TIM_HandleTypeDef* htimer)
{
	ARR_value_tim = htimer->Instance->ARR;
	uint16_t compare_value = (uint16_t)(percent * (float)ARR_value_tim);

	// This works only for timer 1
	htimer->Instance->CCR1 = compare_value;
}

void PWM_SetFrequencyDivider(uint16_t frequency_scale, TIM_HandleTypeDef* htimer)
{
	htimer->Instance->ARR = htimer->Instance->ARR * frequency_scale;
}

uint16_t PWM_GenerateControlSignal(uint16_t input_data)
{
	return 50;
}
