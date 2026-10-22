#ifndef ACKERMANN_CONFIG_H
#define ACKERMANN_CONFIG_H

/* All dimensions are metres. Verify these against the actual chassis. */
#define ACK_WHEELBASE_M             0.145f
#define ACK_TRACK_M                 0.162f
#define ACK_WHEEL_DIAMETER_M         0.065f
#define ACK_GEAR_RATIO              30.0f
#define ACK_ENCODER_LINES           500.0f
#define ACK_CONTROL_DT_S            0.01f
#define ACK_MAX_WHEEL_SPEED_MPS      1.6f
#define ACK_PP_MIN_LOOKAHEAD_M      0.05f
#define ACK_MAX_STEER_RAD            0.35f

/* PC9 / TIM8_CH4: 1 MHz counter, 20 ms period, pulse width in us. */
/* Mechanical straight-ahead verified at A=-2 deg from the 1538 us origin. */
#define ACK_SERVO_CENTER_US          1576
#define ACK_SERVO_MIN_US             1000
#define ACK_SERVO_MAX_US             2000
#define ACK_SERVO_US_PER_RAD         1100.0f
#define ACK_SERVO_DIRECTION          (-1.0f)
/* M4 calibration envelope. Normal driving retains its existing limits. */
#define ACK_DEBUG_MIN_ANGLE_DEG      (-25.0f)
#define ACK_DEBUG_MAX_ANGLE_DEG      29.0f
#define ACK_DEBUG_SERVO_MIN_US       1020
#define ACK_DEBUG_SERVO_MAX_US       2057

/* A = rear left, B = rear right. Positive feedback means forward. */
#define ACK_ENCODER_A_SIGN           1.0f
#define ACK_ENCODER_B_SIGN           (-1.0f)
#define ACK_MOTOR_A_SIGN             1
#define ACK_MOTOR_B_SIGN             1
#define ACK_MOTOR_PWM_TOP            16799
#define ACK_MOTOR_PWM_LIMIT          16700.0f
/* DIANSAI limits sum(error_cm_s) to +/-150 after the calculation.
 * Our speed errors use m/s: equivalent accumulator bound is +/-1.5.
 * Ki is per sample (10 ms here), without an explicit dt multiplier. */
#define ACK_SPEED_INTEGRAL_LIMIT    1.5f
#define ACK_SPEED_KP_MAX            100000.0f
#define ACK_SPEED_KI_MAX            10000.0f
#define ACK_SPEED_A_KP               73897.0f
#define ACK_SPEED_A_KI               4535.0f
#define ACK_SPEED_B_KP               73897.0f
#define ACK_SPEED_B_KI               4535.0f

/* Position PID: degrees -> front steering angle (radians).
 * Ki uses seconds; speed-loop Ki above is the per-10-ms discrete gain.
 * Set YAW_TO_STEER_SIGN=-1 if a left turn decreases reported IMU yaw. */
#define ACK_YAW_KP                   0.020f
#define ACK_YAW_KI                   0.0005f
#define ACK_YAW_KD                   0.0008f
#define ACK_YAW_INTEGRAL_LIMIT       40.0f
#define ACK_YAW_D_FILTER_ALPHA       0.2f
#define ACK_YAW_TO_STEER_SIGN         1.0f
#define ACK_TASK1_SPEED_MPS          0.6f
#define ACK_TASK1_DURATION_MS        5000U
#define ACK_TASK2_SPEED_MPS          0.6f
#define ACK_TASK2_TURN_DEG            90.0f
#define ACK_TASK2_YAW_DIRECTION      1.0f
#define ACK_TASK2_TOLERANCE_DEG      1.2f
#define ACK_TASK2_TIMEOUT_MS         15000U
#define ACK_DEBUG_DEFAULT_SPEED_MPS  0.6f
#define ACK_DEBUG_DEFAULT_ANGLE_DEG  0.0f
#define ACK_MIN_BATTERY_V            10.0f
#define ACK_IMU_TIMEOUT_MS           200U

#endif
