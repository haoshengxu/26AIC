#ifndef TRACKING_CONTROL_H
#define TRACKING_CONTROL_H
#include <stdint.h>
#include "tracking_config.h"

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
} TrackingController;

typedef struct {
    float error_m, derivative_mps, measured_speed_mps, denominator_mps;
    float p_rad, d_rad, stanley_rad, heading_rad;
    float correction_right_rad, steer_left_rad, speed_mps;
    uint8_t valid, heading_used;
} TrackingOutput;

void Tracking_InputInit(void); /* Before RX interrupts. */
typedef struct { float speed_mps, kp, kd, stanley_k, filter_tau_s; } TrackingParameters;
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
