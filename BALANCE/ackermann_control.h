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
/* V3/V4 rear-axle coordinates in metres: X right, Y forward. */
int Ack_PurePursuit(float x_right_m, float y_forward_m, float *steer_left_rad);
int Ack_PurePursuitGain(float x_right_m, float y_forward_m, float gain,
                        float *steer_left_rad);
int Ack_ServoPulse(float steer_rad);
int Ack_DebugServoPulse(float angle_deg);

#endif
