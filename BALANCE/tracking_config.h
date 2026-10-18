#ifndef TRACKING_CONFIG_H
#define TRACKING_CONFIG_H
#include "ackermann_config.h"

/* User-selected VOFA defaults; runtime changes reset to these on reboot. */
#define TRACK_BASE_SPEED_MPS        0.25f
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
