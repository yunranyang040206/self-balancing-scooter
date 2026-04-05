#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

void Motor_Init(void);
void Motor_A_Forward(uint16_t duty);
void Motor_A_Backward(uint16_t duty);
void Motor_B_Forward(uint16_t duty);
void Motor_B_Backward(uint16_t duty);
void Motor_Stop(void);

#endif
