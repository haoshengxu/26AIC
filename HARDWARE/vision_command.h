#ifndef VISION_COMMAND_H
#define VISION_COMMAND_H
#include <stdint.h>

#define VISION_FRAME_SIZE 14U
#define VISION_ROUTE_FRAME_SIZE 22U
#define VISION_TAPE_FRAME_SIZE 25U
#define VISION_TARGET_VALID 0x01U
#define VISION_ENDPOINT_VALID 0x02U
#define VISION_CURVATURE_VALID 0x04U
#define VISION_TARGET_FALLBACK 0x08U
#define VISION_RUN  0x01U
#define VISION_LINE 0x02U
#define VISION_STOP 0x04U
#define VISION_TIMEOUT_MS 200U
#define VISION_ARM_WAIT_MS 1000U
#define VISION_MAX_SPEED_MMPS 250
#define VISION_MAX_STEER_MRAD 350

typedef struct {
    uint32_t received_ms, accepted, bytes, crc_errors, format_errors;
    uint32_t range_errors, sequence_errors, uart_errors, stop_events;
    uint16_t sequence;
    int16_t speed_mmps, steer_mrad;
    uint8_t flags, valid;
} VisionCommand;

/* V3/V4/V5: signed wire units are 0.01 cm (multiply by 0.0001 for metres).
 * X right-positive, Y forward-positive; no calibration offset is applied. */
#define VISION_TARGET_TIMEOUT_MS 300U
#define VISION_TARGET_HOLD_MS 1000U
typedef struct {
    uint32_t received_ms, accepted, crc_errors, format_errors, uart_errors;
    uint32_t invalid_frames, invalid_events, timeouts, duplicate_frames, sequence_discontinuities;
    uint16_t sequence;
    int16_t x_centi_cm, y_centi_cm;
    uint8_t has_frame, link_alive, valid;
    /* V4 metadata; zero for V3. Curvature is absolute, in cm^-1 (x100 for m^-1).
     * Use endpoint_valid/curvature_valid: zero curvature can be a valid straight. */
    uint8_t version, flags, mode, fallback, endpoint_valid, curvature_valid;
    int16_t endpoint_x_centi_cm, endpoint_y_centi_cm;
    float endpoint_abs_curvature_cm_inv;
    /* V5 latest snapshot only. Independent of route valid; zero distance is valid.
     * Unsigned 0.01 cm, no front/behind sign. Never use the motion hold for tape. */
    uint8_t tape_detected;
    uint16_t tape_distance_y_centi_cm;
} VisionTarget;

/* Atomic snapshot; also expires stale targets. Invalid/expired coordinates zero. */
void Vision_GetTarget(VisionTarget *out);
/* Last valid V3/V4/V5 target, usable for <1000ms; invalid frames do not refresh it.
 * Metadata here belongs to that held frame, not the latest route. Use
 * Vision_GetTarget for fresh mode/endpoint telemetry, even with invalid target. */
void Vision_GetControlTarget(VisionTarget *out);
void Vision_Init(void);
void Vision_ProcessByte(uint8_t byte);
void Vision_UartError(void);
void Vision_Get(VisionCommand *out);
uint16_t Vision_Crc16(const uint8_t *data, uint16_t length);
uint8_t Vision_CanRun(const VisionCommand *cmd, uint32_t now);
#endif
