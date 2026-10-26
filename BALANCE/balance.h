#ifndef __BALANCE_H
#define __BALANCE_H
#include "sys.h"
#include "main.h"
void Balance_TimerCallback(TIM_HandleTypeDef *htim);
#include "system.h"
#include "ackermann_config.h"
#include "ackermann_control.h"

#define PWMA1 TIM10->CCR1
#define PWMA2 TIM11->CCR1
#define PWMB1 TIM9->CCR1
#define PWMB2 TIM9->CCR2
#define PWMC1 TIM1->CCR1
#define PWMC2 TIM1->CCR2
#define PWMD1 TIM1->CCR3
#define PWMD2 TIM1->CCR4
#define EN PDin(3)
#define Servo_PWM TIM8->CCR4
#define SERVO_INIT ACK_SERVO_CENTER_US

void Set_Pwm(int motor_a, int motor_b, int motor_c, int motor_d, int servo);
void Drive_Motor(float speed_mps, float unused_vy, float steer_rad);
void SpeedPI_Reset(void);
/* Caller serializes with the control ISR. right=0 left, right=1 right. */
void SpeedPI_ResetWheel(uint8_t right);
/* Coherent snapshot of both latest 100 Hz PI calculations. */
void SpeedPI_GetState(AckSpeedPI *left, AckSpeedPI *right);
int Speed_PI_A(float encoder, float target);
int Speed_PI_B(float encoder, float target);
void Get_Velocity_Form_Encoder(void);
typedef struct {
    float left_m, right_m, centre_m, delta_m, calibration_m;
} EncoderOdometry;
/* Signed wheel travel since boot; centre is the rear-axle mean, not world X/Y.
 * delta_m belongs to the latest encoder read (100 Hz), including while stopped.
 * calibration_m is signed centre travel from the independently reset origin. */
void Balance_GetOdometry(EncoderOdometry *out);
/* Reset only the independent calibration origin to the latest sampled centre.
 * Safe during motion; does not consume encoder counts or change control travel. */
void Balance_ResetCalibrationOdometry(void);
u8 Turn_Off(float voltage);
float target_limit_float(float value, float low, float high);
int target_limit_int(int value, int low, int high);
float float_abs(float value);
#endif
