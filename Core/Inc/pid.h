#ifndef PID_H
#define PID_H

#include <stdint.h>

void Control(void);

extern short gyrox, gyroy, gyroz;
extern short aacx, aacy, aacz;

extern int Encoder_Left;
extern int Encoder_Right;

extern int Vertical_out;
extern int Velocity_out;
extern int Turn_out;
extern int Target_Speed;
extern int Target_turn;
extern int MOTO1;
extern int MOTO2;

extern float Med_Angle;
extern float pitch, roll, yaw;

#endif
