#include "ackermann_control.h"
#include <math.h>

float Ack_Clamp(float value, float low, float high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

float Ack_AngleError(float target_deg, float current_deg)
{
    float error = target_deg - current_deg;
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;
    return error;
}

void Ack_SpeedReset(AckSpeedPI *pi)
{
    pi->output = 0.0f;
    pi->last_error = 0.0f;
}

float Ack_SpeedUpdate(AckSpeedPI *pi, float target, float measured,
                      float kp, float ki)
{
    float error = target - measured;
    /* Incremental PI: u[k] = u[k-1] + Kp*(e[k]-e[k-1]) + Ki*e[k].
     * Clamp the stored output as well as the PWM: no hidden windup. */
    pi->output = Ack_Clamp(pi->output + kp * (error - pi->last_error) +
                          ki * error, -ACK_MOTOR_PWM_LIMIT, ACK_MOTOR_PWM_LIMIT);
    pi->last_error = error;
    return pi->output;
}

void Ack_YawReset(AckYawPID *pid, float initial_error)
{
    pid->integral = 0.0f;
    pid->last_error = initial_error;
    pid->derivative = 0.0f;
}

float Ack_YawUpdate(AckYawPID *pid, float target_deg, float current_deg)
{
    float error = Ack_AngleError(target_deg, current_deg);
    float candidate = Ack_Clamp(pid->integral + error * ACK_CONTROL_DT_S,
                               -ACK_YAW_INTEGRAL_LIMIT, ACK_YAW_INTEGRAL_LIMIT);
    float derivative = Ack_AngleError(error, pid->last_error) / ACK_CONTROL_DT_S;
    float output;
    pid->derivative += ACK_YAW_D_FILTER_ALPHA * (derivative - pid->derivative);
    pid->last_error = error;
    output = ACK_YAW_KP * error + ACK_YAW_KI * candidate +
             ACK_YAW_KD * pid->derivative;
    /* Integrate only when unsaturated, or when error relieves saturation. */
    if ((output <= ACK_MAX_STEER_RAD && output >= -ACK_MAX_STEER_RAD) ||
        (output > ACK_MAX_STEER_RAD && error < 0.0f) ||
        (output < -ACK_MAX_STEER_RAD && error > 0.0f))
        pid->integral = candidate;
    output = ACK_YAW_KP * error + ACK_YAW_KI * pid->integral +
             ACK_YAW_KD * pid->derivative;
    return Ack_Clamp(ACK_YAW_TO_STEER_SIGN * output,
                     -ACK_MAX_STEER_RAD, ACK_MAX_STEER_RAD);
}

void Ack_Kinematics(float speed, float steer_rad, float *left, float *right)
{
    float factor;
    float peak;
    steer_rad = Ack_Clamp(steer_rad, -ACK_MAX_STEER_RAD, ACK_MAX_STEER_RAD);
    /* Curvature avoids division by tan(0) on a straight trajectory.
     * Positive steer is left: inner left wheel must run more slowly. */
    factor = 0.5f * ACK_TRACK_M * tanf(steer_rad) / ACK_WHEELBASE_M;
    *left = speed * (1.0f - factor);
    *right = speed * (1.0f + factor);
    peak = fabsf(*left) > fabsf(*right) ? fabsf(*left) : fabsf(*right);
    if (peak > ACK_MAX_WHEEL_SPEED_MPS) {
        *left *= ACK_MAX_WHEEL_SPEED_MPS / peak;
        *right *= ACK_MAX_WHEEL_SPEED_MPS / peak;
    }
}

int Ack_ServoPulse(float steer_rad)
{
    float pulse;
    steer_rad = Ack_Clamp(steer_rad, -ACK_MAX_STEER_RAD, ACK_MAX_STEER_RAD);
    pulse = ACK_SERVO_CENTER_US + ACK_SERVO_DIRECTION * ACK_SERVO_US_PER_RAD * steer_rad;
    return (int)(Ack_Clamp(pulse, ACK_SERVO_MIN_US, ACK_SERVO_MAX_US) + 0.5f);
}

int Ack_DebugServoPulse(float angle_deg)
{
    float pulse;
    angle_deg = Ack_Clamp(angle_deg, ACK_DEBUG_MIN_ANGLE_DEG, ACK_DEBUG_MAX_ANGLE_DEG);
    pulse = ACK_SERVO_CENTER_US + ACK_SERVO_DIRECTION * ACK_SERVO_US_PER_RAD *
            angle_deg * 0.01745329252f;
    return (int)(Ack_Clamp(pulse, ACK_DEBUG_SERVO_MIN_US, ACK_DEBUG_SERVO_MAX_US) + 0.5f);
}
