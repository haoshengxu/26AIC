#ifndef TRACKING_CONFIG_H
#define TRACKING_CONFIG_H
#include "ackermann_config.h"

/* User-selected VOFA defaults; runtime changes reset to these on reboot. */
#define TRACK_BASE_SPEED_MPS        0.90f
/* Legacy V3 slowdown: threshold cm, gain (m/s)/cm, shared curve floor m/s. */
#define TRACK_TURN_X_THRESHOLD_CM   5.0f
#define TRACK_TURN_SPEED_GAIN       0.040f
#define TRACK_TURN_MIN_SPEED_MPS    0.60f
/* V4 route-aware controller. Curvature uses m^-1, speeds use m/s. */
#define TRACK_PP_GAIN               1.40f /* curve gain */
#define TRACK_PP_STRAIGHT_GAIN      0.95f
#define TRACK_GAIN_CURVE_ENTER      0.82f /* m^-1, independent of speed threshold */
#define TRACK_GAIN_CURVE_EXIT       0.31f /* hysteresis: exit < enter */
#define TRACK_GAIN_RATE_PER_S       2.0f
#define TRACK_CURVE_THRESHOLD       0.82f
#define TRACK_CURVE_SPEED_GAIN      0.16f
#define TRACK_SINGLE_SPEED_MPS      0.50f
#define TRACK_FALLBACK_SPEED_MPS    0.40f
#define TRACK_UNKNOWN_SPEED_MPS     0.50f
#define TRACK_HOLD_SPEED_MPS        0.40f
#define TRACK_SAT_SPEED_MPS         0.40f
#define TRACK_ACCEL_MPS2            0.430f
#define TRACK_START_ACCEL_MPS2      0.80f
#define TRACK_DECEL_MPS2            1.60f
/* Yellow tape: front clearance, braking plan, confirmed-stop dwell. */
#define TRACK_TAPE_DECEL_MPS2       0.80f
#define TRACK_FINISH_APPROACH_SPEED 1.00f
#define TRACK_FINISH_BLIND_SPEED_MPS 0.25f
#define TRACK_FINISH_BLIND_TIME_S    0.30f
#define TRACK_TAPE_FINISH_GAP_CM    0.0f
#define TRACK_TAPE_GAP_CM           7.0f
#define TRACK_TAPE_WAIT_S           0.30f
#define TRACK_TAPE_RECOVER_MS       1000U
#define TRACK_TAPE_RECOVER_SPEED    0.50f
#define TRACK_TAPE_RECOVER_LEFT_RAD 0.1745329252f
#define TRACK_TAPE1_STRAIGHT_S      0.00f
#define TRACK_TAPE1_RIGHT_DEG       10.4f
#define TRACK_TAPE1_SPEED_MPS       0.33f
#define TRACK_TAPE1_MAX_S           2.00f
#define TRACK_TAPE3_STRAIGHT_S      0.70f
#define TRACK_TAPE3_RIGHT_DEG       4.7f
#define TRACK_TAPE3_SPEED_MPS       0.32f
#define TRACK_TAPE3_MAX_S           2.80f
#define TRACK_REAR_TO_FRONT_M       0.227f
#define TRACK_TAPE_CONFIRM_MS       100U
#define TRACK_TAPE_GAP_MAX_MS       100U
#define TRACK_TAPE_DELAY_S          0.10f
#define TRACK_TAPE_STOP_TOLERANCE_M 0.005f /* Stop within 5 mm before planned clearance. */
#define TRACK_ROUTE_FRESH_MS        150U
#define TRACK_KP                    5.45f   /* rad / m */
#define TRACK_KD                    0.30f  /* rad s / m */
#define TRACK_STANLEY_K             0.6f   /* 1 / s */
#define TRACK_MIN_SPEED_MPS         0.10f  /* denominator floor */
#define TRACK_D_FILTER_TAU_S        0.08f  /* 0 = unfiltered finite difference */
#define TRACK_D_LIMIT_MPS           2.0f
#define TRACK_STEER_LIMIT_RAD       ACK_MAX_STEER_RAD
/* Normalized q>0: road right of camera -> car left of road -> e>0.
 * Edge-distance center reference is subtracted without reversing wire sign
 * in Distance_ProcessFrame(). */
#define TRACK_ERROR_SIGN            1.0f
/* Formula uses right-positive correction; chassis uses LEFT-positive steer. */
#define TRACK_TO_CHASSIS_SIGN       (-1.0f)
#define TRACK_HEADING_GAIN          1.0f
#define TRACK_HEADING_MAX_AGE_MS    200U
/* Hold the last valid measurement for up to 1000 ms; display expires at 250 ms. */
#define TRACK_TIMEOUT_MS            1000U
#endif
