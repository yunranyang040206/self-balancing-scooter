#ifndef PWM_H
#define PWM_H

#include <stdint.h>

void PWM_Start(void);
void PWM_Stop(void);

void PWM_SetA(uint16_t duty);
void PWM_SetB(uint16_t duty);

uint16_t PWM_GetMaxDuty(void);

#endif
