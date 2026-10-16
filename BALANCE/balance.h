#ifndef __BALANCE_H
#define __BALANCE_H
#include "sys.h"
#include "main.h"
void Balance_TimerCallback(TIM_HandleTypeDef *htim);
#include "system.h"
#include "ackermann_config.h"

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
int Incremental_PI_A(float encoder, float target);
int Incremental_PI_B(float encoder, float target);
void Get_Velocity_Form_Encoder(void);
u8 Turn_Off(float voltage);
float target_limit_float(float value, float low, float high);
int target_limit_int(int value, int low, int high);
float float_abs(float value);
#endif
