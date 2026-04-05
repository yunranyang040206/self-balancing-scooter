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
float Med_Angle = 3.0f;

/* PID gains */
float Vertical_Kp = -80.0f;
float Vertical_Kd = -0.75f;

float Velocity_Kp = -1.1f;
float Velocity_Ki = -0.0055f;

float Turn_Kp = 10.0f;
float Turn_Kd = 0.6f;

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

static void MPU_UpdateAngles(void)
{
    float ax_g, ay_g, az_g;

    MPU_ReadRaw(&aacx, &aacy, &aacz, &gyrox, &gyroy, &gyroz);

    ax_g = (float)aacx / ACCEL_SCALE;
    ay_g = (float)aacy / ACCEL_SCALE;
    az_g = (float)aacz / ACCEL_SCALE;

    roll  = atan2f(ay_g, az_g) * RAD_TO_DEG;
    pitch = atan2f(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g)) * RAD_TO_DEG;
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
    if (pwm > 0 && pwm < 2000) return 2000;
    if (pwm < 0 && pwm > -2000) return -2000;
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

void Control(void)
{
    int PWM_out;
    float gyro_y_dps, gyro_z_dps;

    /* Read encoder counts */
    Encoder_Left  = Encoder_ReadLeft();
    Encoder_Right = -Encoder_ReadRight();   // keep this sign if your right encoder is reversed

    /* Clear encoder counts for next control cycle */
    Encoder_ResetLeft();
    Encoder_ResetRight();

    /* Update IMU angles and raw gyro */
    MPU_UpdateAngles();

    /* Convert gyro raw values to deg/s */
    gyro_y_dps = (float)gyroy / GYRO_SCALE;
    gyro_z_dps = (float)gyroz / GYRO_SCALE;

    /* Desired commands */
    Target_Speed = 0;   // standing still
    Target_turn  = 0;   // no turning

    /* Outer velocity loop:
       if robot rolls away, this shifts target angle slightly */
    Velocity_out = Velocity(Target_Speed, Encoder_Left, Encoder_Right);

    /* Inner balance loop:
       pitch is the main balancing angle */
//    Vertical_out = Vertical(Med_Angle + Velocity_out, pitch, gyro_y_dps);
    float target_angle = Med_Angle + Velocity_out;
    Vertical_out = Vertical(target_angle, pitch, gyro_y_dps);

    /* Turning loop */
    Turn_out = Turn(gyro_z_dps, Target_turn);

    /* Motor mix */
    PWM_out = Vertical_out;
    MOTO1 = PWM_out - Turn_out;
    MOTO2 = PWM_out + Turn_out;

    /* Limit output */
    LimitPWM(&MOTO1, &MOTO2);

    /* Safety stop if fallen */
    if (fabsf(pitch) > 35.0f) {
        Motor_Stop();
        stop = 1;
        MOTO1 = 0;
        MOTO2 = 0;
        return;
    }

    /* Apply motor output */
    ApplyMotorPWM(MOTO1, MOTO2);
}
