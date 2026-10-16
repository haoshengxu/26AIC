#ifndef ACKERMANN_CONTROL_H
#define ACKERMANN_CONTROL_H

#include "ackermann_config.h"

typedef struct { float output; float last_error; } AckSpeedPI;
typedef struct {
    float integral;
    float last_error;
    float derivative;
} AckYawPID;

float Ack_Clamp(float value, float low, float high);
float Ack_AngleError(float target_deg, float current_deg);
void Ack_SpeedReset(AckSpeedPI *pi);
float Ack_SpeedUpdate(AckSpeedPI *pi, float target, float measured,
                      float kp, float ki);
void Ack_YawReset(AckYawPID *pid, float initial_error);
float Ack_YawUpdate(AckYawPID *pid, float target_deg, float current_deg);
void Ack_Kinematics(float speed, float steer_rad, float *left, float *right);
int Ack_ServoPulse(float steer_rad);
int Ack_DebugServoPulse(float angle_deg);

#endif
