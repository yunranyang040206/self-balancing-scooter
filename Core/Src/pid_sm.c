#include "pid.h"
#include "encoder.h"
#include "motor.h"
#include "main.h"
#include <stdint.h>
#include <math.h>

/* External I2C handle from main.c */
extern I2C_HandleTypeDef hi2c1;

/* Global attitude variables from main.c */
extern float pitch, roll, yaw;

/* Sensor data */
int Encoder_Left = 0, Encoder_Right = 0;
short gyrox = 0, gyroy = 0, gyroz = 0;
short aacx = 0, aacy = 0, aacz = 0;

/* Control intermediate values */
int Vertical_out = 0, Velocity_out = 0, Turn_out = 0;
int Target_Speed = 0, Target_turn = 0;
int MOTO1 = 0, MOTO2 = 0;

/* Balance offset */
float Med_Angle = 1.0f;

/* PID gains */
float Vertical_Kp = -400.0f;
float Vertical_Kd = -0.75f;

float Velocity_Kp = -1.1f;
float Velocity_Ki = -0.00f;

float Turn_Kp = 10.0f;
float Turn_Kd = 0.6f;

float pitch_offset = 0.0f;

float Current_Target_Speed = 0.0f; // The speed currently being used
float Desired_Speed = 0.0f;        // The speed we WANT to reach (0, 30, 80, etc.)
#define RAMP_STEP 0.2f             // How fast to accelerate (adjust this)

uint8_t stop = 0;

#define SPEED_Y         12
#define SPEED_Z         150
#define PWM_MAX         17999
#define MPU_ADDR        0x68
#define MPU_PWR_MGMT_1  0x6B
#define MPU_ACCEL_XOUT_H 0x3B
#define MPU_GYRO_XOUT_H  0x43
#define ACCEL_SCALE     16384.0f
#define GYRO_SCALE      131.0f
#define RAD_TO_DEG      57.2957795f

static int16_t MPU_Read16(uint8_t reg)
{
    uint8_t buf[2];

    if (HAL_I2C_Mem_Read(&hi2c1,
                         MPU_ADDR << 1,
                         reg,
                         I2C_MEMADD_SIZE_8BIT,
                         buf,
                         2,
                         100) != HAL_OK)
    {
        return 0;
    }

    return (int16_t)((buf[0] << 8) | buf[1]);
}

static void MPU_ReadRaw(short *ax, short *ay, short *az,
                        short *gx, short *gy, short *gz)
{
    *ax = MPU_Read16(MPU_ACCEL_XOUT_H);
    *ay = MPU_Read16(MPU_ACCEL_XOUT_H + 2);
    *az = MPU_Read16(MPU_ACCEL_XOUT_H + 4);

    *gx = MPU_Read16(MPU_GYRO_XOUT_H);
    *gy = MPU_Read16(MPU_GYRO_XOUT_H + 2);
    *gz = MPU_Read16(MPU_GYRO_XOUT_H + 4);
}

//static void MPU_UpdateAngles(void)
//{
//    float ax_g, ay_g, az_g;
//
//    MPU_ReadRaw(&aacx, &aacy, &aacz, &gyrox, &gyroy, &gyroz);
//
//    ax_g = (float)aacx / ACCEL_SCALE;
//    ay_g = (float)aacy / ACCEL_SCALE;
//    az_g = (float)aacz / ACCEL_SCALE;
//
//    roll  = atan2f(ay_g, az_g) * RAD_TO_DEG;
////    pitch = atan2f(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g)) * RAD_TO_DEG;
////    pitch = 0.98f * (pitch + gyro_pitch_dps * dt) + 0.02f * accel_pitch;
//    pitch = atan2f(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g)) * RAD_TO_DEG;
//    pitch -= pitch_offset;
//    yaw = 0.0f;
//}

static void MPU_UpdateAngles(void)
{
    static uint32_t last_tick = 0;
    uint32_t now = HAL_GetTick();
    float dt = (last_tick == 0) ? 0.01f : (now - last_tick) * 0.001f;
    last_tick = now;

    float ax_g, ay_g, az_g;
    float accel_pitch;
    float gyro_pitch_dps;

    MPU_ReadRaw(&aacx, &aacy, &aacz, &gyrox, &gyroy, &gyroz);

    ax_g = (float)aacx / ACCEL_SCALE;
    ay_g = (float)aacy / ACCEL_SCALE;
    az_g = (float)aacz / ACCEL_SCALE;

    accel_pitch = atan2f(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g)) * RAD_TO_DEG;

    gyro_pitch_dps = (float)gyroy / GYRO_SCALE;   // may need gyrox or sign flip

    pitch = 0.98f * (pitch + gyro_pitch_dps * dt) + 0.02f * accel_pitch;
    roll  = atan2f(ay_g, az_g) * RAD_TO_DEG;
    yaw = 0.0f;
}

static void LimitPWM(int *m1, int *m2)
{
    if (*m1 > PWM_MAX)  *m1 = PWM_MAX;
    if (*m1 < -PWM_MAX) *m1 = -PWM_MAX;

    if (*m2 > PWM_MAX)  *m2 = PWM_MAX;
    if (*m2 < -PWM_MAX) *m2 = -PWM_MAX;
}

static int ApplyDeadband(int pwm)
{
    if (pwm > 0 && pwm < 1200) return 1200;
    if (pwm < 0 && pwm > -1200) return -1200;
    return pwm;
}

static void ApplyMotorPWM(int m1, int m2)
{
    m1 = ApplyDeadband(m1);
    m2 = ApplyDeadband(m2);

    if (m1 > 0) {
        Motor_A_Forward((uint16_t)m1);
    } else if (m1 < 0) {
        Motor_A_Backward((uint16_t)(-m1));
    }

    if (m2 > 0) {
        Motor_B_Forward((uint16_t)m2);
    } else if (m2 < 0) {
        Motor_B_Backward((uint16_t)(-m2));
    }

    if (m1 == 0 && m2 == 0) {
        Motor_Stop();
    }
}

static void SafetyStop(float angle)
{
    if (angle > 45.0f || angle < -45.0f) {
        Motor_Stop();
        stop = 1;
        Target_Speed = 0;
        Target_turn = 0;
    }
}

int Vertical(float Med, float Angle, float gyro_Y)
{
    return (int)(Vertical_Kp * (Angle - Med) + Vertical_Kd * gyro_Y);
}

int Velocity(int Target, int encoder_L, int encoder_R)
{
    static int Err_LowOut_last = 0;
    static int Encoder_S = 0;
    static float a = 0.7f;

    int Err, Err_LowOut, temp;

    Velocity_Ki = Velocity_Kp / 200.0f;

    Err = (encoder_L + encoder_R) - Target;

    Err_LowOut = (int)((1.0f - a) * Err + a * Err_LowOut_last);
    Err_LowOut_last = Err_LowOut;

    Encoder_S += Err_LowOut;

    if (Encoder_S > 20000) Encoder_S = 20000;
    if (Encoder_S < -20000) Encoder_S = -20000;

    if (stop == 1) {
        Encoder_S = 0;
        stop = 0;
    }

    temp = (int)(Velocity_Kp * Err_LowOut + Velocity_Ki * Encoder_S);
    return temp;
}

int Turn(float gyro_Z, int Target_turn_in)
{
    return (int)(Turn_Kp * Target_turn_in + Turn_Kd * gyro_Z);
}

//
//void Control(void)
//{
//    int PWM_out;
//    float gyro_y_dps, gyro_z_dps;
//    float target_angle;
//    uint32_t now = HAL_GetTick();
//
//    static uint32_t state_start = 0;
//    static uint8_t state = 0;
//    static uint8_t initialized = 0;
//    static uint8_t vel_count = 0;
//
//    if (!initialized) {
//        state_start = now;
//        initialized = 1;
//    }
//
//    switch (state)
//    {
//        case 0:
//            Target_Speed = 5;
//            Target_turn  = 0.1;
//            if (now - state_start >= 3000) {
//                state = 1;
//                state_start = now;
//            }
//            break;
//
//        case 1:
//            Target_Speed = 0;
//            Target_turn  = 0;
//            if (now - state_start >= 3000) {
//                state = 2;
//                state_start = now;
//            }
//            break;
//
//        case 2:
//            Target_Speed = -5;
//            Target_turn  = -0.1;
//            if (now - state_start >= 3000) {
//                state = 3;
//                state_start = now;
//            }
//            break;
//
//        case 3:
//        default:
//            Target_Speed = 0;
//            Target_turn  = 0;
//            if (now - state_start >= 3000) {
//                state = 0;
//                state_start = now;
//            }
//            break;
//    }
//
//    Encoder_Left  = Encoder_ReadLeft();
//    Encoder_Right = -Encoder_ReadRight();
//
//    Encoder_ResetLeft();
//    Encoder_ResetRight();
//
//    MPU_UpdateAngles();
//
//    gyro_y_dps = (float)gyroy / GYRO_SCALE;
//    gyro_z_dps = (float)gyroz / GYRO_SCALE;
//
//    vel_count++;
//    if (vel_count >= 5) {
//        vel_count = 0;
//        Velocity_out = Velocity(Target_Speed, Encoder_Left, Encoder_Right);
//    }
//
//    target_angle = Med_Angle + 0.05f * Velocity_out;
//
//    if (target_angle > Med_Angle + 5.0f) target_angle = Med_Angle + 5.0f;
//    if (target_angle < Med_Angle - 5.0f) target_angle = Med_Angle - 5.0f;
//
//    Vertical_out = Vertical(target_angle, pitch, gyro_y_dps);
//    Turn_out = Turn(gyro_z_dps, Target_turn);
//
//    PWM_out = Vertical_out;
//    MOTO1 = PWM_out - Turn_out;
//    MOTO2 = PWM_out + Turn_out;
//
//    LimitPWM(&MOTO1, &MOTO2);
//
//    if (fabsf(pitch) > 35.0f) {
//        Motor_Stop();
//        stop = 1;
//        MOTO1 = 0;
//        MOTO2 = 0;
//        return;
//    }
//
//    ApplyMotorPWM(MOTO1, MOTO2);
//}

void Control(void)
{
    int PWM_out;
    float gyro_y_dps, gyro_z_dps;
    float target_angle;
    uint32_t now = HAL_GetTick();

    static uint32_t state_start = 0;
    static uint8_t state = 0;
    static uint8_t initialized = 0;
    static uint8_t vel_count = 0;

    if (!initialized) {
        state_start = now;
        initialized = 1;
    }

    /* 1. State Machine: Now sets 'Desired_Speed' instead of 'Target_Speed' */
    switch (state)
    {
        case 0:
            Desired_Speed = 30.0f; // Target speed to ramp towards
            Target_turn  = 0.0f;
            if (now - state_start >= 4000) { state = 1; state_start = now; }
            break;

        case 1:
            Desired_Speed = 0.0f;  // Smoothly return to stop
            Target_turn  = 0.0f;
            if (now - state_start >= 3000) { state = 2; state_start = now; }
            break;

        case 2:
            Desired_Speed = -30.0f; // Smoothly ramp to reverse
            Target_turn  = 0.0f;
            if (now - state_start >= 4000) { state = 3; state_start = now; }
            break;

        case 3:
        default:
            Desired_Speed = 0.0f;
            Target_turn  = 0.0f;
            if (now - state_start >= 3000) { state = 0; state_start = now; }
            break;
    }

    /* 2. Ramping Logic: Current_Target_Speed "chases" Desired_Speed */
    if (Current_Target_Speed < Desired_Speed) {
        Current_Target_Speed += RAMP_STEP;
        if (Current_Target_Speed > Desired_Speed) Current_Target_Speed = Desired_Speed;
    }
    else if (Current_Target_Speed > Desired_Speed) {
        Current_Target_Speed -= RAMP_STEP;
        if (Current_Target_Speed < Desired_Speed) Current_Target_Speed = Desired_Speed;
    }

    /* 3. Sensor Acquisition */
    Encoder_Left  = Encoder_ReadLeft();
    Encoder_Right = -Encoder_ReadRight(); // Double check this sign!
    Encoder_ResetLeft();
    Encoder_ResetRight();

    MPU_UpdateAngles();
    gyro_y_dps = (float)gyroy / GYRO_SCALE;
    gyro_z_dps = (float)gyroz / GYRO_SCALE;

    /* 4. Velocity Loop (Outer Loop - Runs slower) */
    vel_count++;
    if (vel_count >= 8) { // Run every ~40ms if Control() is at 5ms
        vel_count = 0;
        // Use the RAMPED speed here
        Velocity_out = Velocity((int)Current_Target_Speed, Encoder_Left, Encoder_Right);
    }

    /* 5. Calculate Lean Angle (The "Controlled Fall") */
    // Increased multiplier from 0.002 to 0.01 for better "lean" response
    // Increased clamp from 1.0 to 5.0 to allow actual movement
    target_angle = Med_Angle + 0.01f * Velocity_out;
    if (target_angle > Med_Angle + 5.0f) target_angle = Med_Angle + 5.0f;
    if (target_angle < Med_Angle - 5.0f) target_angle = Med_Angle - 5.0f;

    /* 6. Vertical Loop (Inner Loop - Runs every cycle) */
    Vertical_out = Vertical(target_angle, pitch, gyro_y_dps);
    Turn_out = Turn(gyro_z_dps, Target_turn);

    /* 7. Output Mixing and Motor Protection */
    MOTO1 = Vertical_out - Turn_out;
    MOTO2 = Vertical_out + Turn_out;

    LimitPWM(&MOTO1, &MOTO2);

    // Safety: If fallen over, kill motors and reset the Velocity Integrator
    if (fabsf(pitch) > 40.0f) {
        Motor_Stop();
        stop = 1; // This triggers Encoder_S = 0 inside your Velocity() function
        MOTO1 = 0;
        MOTO2 = 0;
    } else {
        ApplyMotorPWM(MOTO1, MOTO2);
    }
}
