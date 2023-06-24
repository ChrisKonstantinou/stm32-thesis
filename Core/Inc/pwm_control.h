#ifndef __PWM_CONTROL_H
#define __PWM_CONTROL_H

#include "main.h"

uint32_t ARR_value_tim;

uint16_t 	PWM_GenerateControlSignal(uint16_t);
void 		PWM_SetPercentage(float, TIM_HandleTypeDef*);
void 		PWM_SetFrequencyDivider(uint16_t, TIM_HandleTypeDef*);

#endif
