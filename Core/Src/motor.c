//#include "motor.h"
//#include "main.h"
//#include "pwm.h"
//
///*
// * Mapping:
// *   Motor A:
// *     PWMA -> PA11 -> TIM1_CH4
// *     AI1  -> PB13
// *     AI2  -> PB12
// *
// *   Motor B:
// *     PWMB -> PA8  -> TIM1_CH1
// *     BI1  -> PB14
// *     BI2  -> PB15
// */
//
//void Motor_Init(void)
//{
//    PWM_Start();
//    Motor_Stop();
//}
//
//void Motor_A_Forward(uint16_t duty)
//{
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);   // AI1 = 1
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // AI2 = 0
//    PWM_SetA(duty);
//}
//
//void Motor_A_Backward(uint16_t duty)
//{
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); // AI1 = 0
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);   // AI2 = 1
//    PWM_SetA(duty);
//}
//
//void Motor_B_Forward(uint16_t duty)
//{
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);   // BI1 = 1
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); // BI2 = 0
//    PWM_SetB(duty);
//}
//
//void Motor_B_Backward(uint16_t duty)
//{
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); // BI1 = 0
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);   // BI2 = 1
//    PWM_SetB(duty);
//}
//
//void Motor_Stop(void)
//{
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
//
//    PWM_SetA(0);
//    PWM_SetB(0);
//}

#include "motor.h"
#include "main.h"
#include "pwm.h"

void Motor_Init(void)
{
    PWM_Start();
    Motor_Stop();
}

void Motor_A_Forward(uint16_t duty)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    PWM_SetA(duty);
}

void Motor_A_Backward(uint16_t duty)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    PWM_SetA(duty);
}

void Motor_B_Forward(uint16_t duty)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
    PWM_SetB(duty);
}

void Motor_B_Backward(uint16_t duty)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
    PWM_SetB(duty);
}

void Motor_Stop(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);

    PWM_SetA(0);
    PWM_SetB(0);
}
