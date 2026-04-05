#include "pwm.h"
#include "main.h"
extern TIM_HandleTypeDef htim1;
/*
 * Mapping:
 *   PA11 -> TIM1_CH4 -> PWMA
 *   PA8  -> TIM1_CH1 -> PWMB
 */

static uint16_t PWM_Clamp(uint16_t duty)
{
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1); //read timer's auto-reload value
    if (duty > arr)
    {
        duty = (uint16_t)arr; // make sure the requested PWM duty is within the range
    }
    return duty;
}

void PWM_Start(void)
{
	// make sure the motors start in a safe stopped state
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);   // PA8  -> PWMB
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);   // PA11 -> PWMA

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0); //set duty cycle to 0
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
}

void PWM_Stop(void)
{
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
}

void PWM_SetA(uint16_t duty)
{
    duty = PWM_Clamp(duty);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, duty);   // PWMA on PA11
}

void PWM_SetB(uint16_t duty)
{
    duty = PWM_Clamp(duty);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty);   // PWMB on PA8
}

uint16_t PWM_GetMaxDuty(void)
{
    return (uint16_t)__HAL_TIM_GET_AUTORELOAD(&htim1);
}
