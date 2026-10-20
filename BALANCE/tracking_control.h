#ifndef TRACKING_CONTROL_H
#define TRACKING_CONTROL_H
#include <stdint.h>
#include "tracking_config.h"
#include "vision_command.h"

typedef struct {
    int16_t distance_raw;
    uint16_t sequence;
    uint8_t valid;
    uint32_t frames, received_ms, stop_events, sequence_errors;
} TrackingInput;

typedef struct {
    float last_error_m, derivative_mps;
    uint32_t last_sample_ms, last_frames;
    uint8_t has_sample;
    float route_speed_mps;
} TrackingController;

/* State is the highest-priority active condition; reasons retain all flags. */
enum { TRACK_ROUTE_OFF, TRACK_ROUTE_NORMAL, TRACK_ROUTE_SINGLE,
       TRACK_ROUTE_FALLBACK, TRACK_ROUTE_UNKNOWN, TRACK_ROUTE_HOLD,
       TRACK_ROUTE_SATURATED };
#define TRACK_REASON_SINGLE     0x01U
#define TRACK_REASON_FALLBACK   0x02U
#define TRACK_REASON_UNKNOWN    0x04U
#define TRACK_REASON_HOLD       0x08U
#define TRACK_REASON_SATURATED  0x10U

typedef struct {
    float error_m, derivative_mps, measured_speed_mps, denominator_mps;
    float p_rad, d_rad, stanley_rad, heading_rad;
    float correction_right_rad, steer_left_rad, speed_mps;
    uint8_t valid, heading_used;
    float control_x_m, control_y_m, raw_steer_left_rad;
    float speed_curvature_m_inv, requested_speed_mps;
    uint8_t route_state, route_reasons;
} TrackingOutput;

void Tracking_InputInit(void); /* Before RX interrupts. */
typedef struct {
    float speed_mps, kp, kd, stanley_k, filter_tau_s;
    float turn_x_threshold_cm, turn_speed_gain, turn_min_speed_mps;
    float pp_gain, curve_threshold, curve_speed_gain;
    float single_speed_mps, fallback_speed_mps, unknown_speed_mps;
    float hold_speed_mps, saturated_speed_mps, accel_mps2, decel_mps2;
} TrackingParameters;
/* 100 Hz M3: V4 route-aware speeds; V3 retains the legacy X speed rule. */
uint8_t Tracking_UpdateRoute(TrackingController *controller,
    const VisionTarget *control, const VisionTarget *latest, uint32_t now,
    float measured_speed, const TrackingParameters *params, TrackingOutput *out);
/* M3 target X in metres; returns requested rear-axle centre speed. */
float Tracking_TurnSpeed(float x_right_m, const TrackingParameters *params);
void Tracking_GetParameters(TrackingParameters *out);
void Tracking_DefaultParameters(void);
/* Caller masks interrupts to serialize with the control ISR. */
uint8_t Tracking_SetParameter(const char *key, float value);
/* Called only for CRC/semantically valid V2 packets, including invalid measurement. */
void Tracking_OnFrame(int16_t raw, uint16_t seq, uint8_t valid, uint32_t now);
void Tracking_UartError(void);
void Tracking_GetInput(TrackingInput *out);
uint8_t Tracking_CanRun(const TrackingInput *in, uint32_t now);
void Tracking_Reset(TrackingController *controller, TrackingOutput *out);
/* Measured chassis speed = mean of the two rear-wheel encoder velocities. */
uint8_t Tracking_Update(TrackingController *controller, const TrackingInput *in,
                        uint32_t now, float measured_speed, uint8_t imu_valid,
                        float yaw_deg, TrackingOutput *out);
/* Optional future path-heading source: same world reference/sign as IMU yaw.
 * No caller in this V2 firmware -> heading term is omitted by default.
 * Caller must refresh this reference; IMU validity alone is insufficient. */
void Tracking_SetHeadingReference(float target_yaw_deg, uint8_t valid);
#endif
