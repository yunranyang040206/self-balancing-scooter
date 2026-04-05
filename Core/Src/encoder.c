#include "encoder.h"
#include "main.h"

/*
 * Mapping:
 *   Left encoder  -> TIM2 -> PA0 / PA1
 *   Right encoder -> TIM4 -> PB6 / PB7
 *
 * TIM2 is 32-bit on STM32F446RE.
 * TIM4 is 16-bit on STM32F446RE.
 *
 * This version follows your current workflow:
 *   - CubeMX does the timer/pin initialization
 *   - encoder.c only starts encoder mode and reads counts
 */

void Encoder_Init(void)
{
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
}

int32_t Encoder_ReadLeft(void)
{
    int32_t value = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    return value;
}

int16_t Encoder_ReadRight(void)
{
    int16_t value = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    return value;
}

void Encoder_ResetLeft(void)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0);
}

void Encoder_ResetRight(void)
{
    __HAL_TIM_SET_COUNTER(&htim4, 0);
}
