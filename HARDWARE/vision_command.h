#ifndef VISION_COMMAND_H
#define VISION_COMMAND_H
#include <stdint.h>

#define VISION_FRAME_SIZE 14U
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

void Vision_Init(void);
void Vision_ProcessByte(uint8_t byte);
void Vision_UartError(void);
void Vision_Get(VisionCommand *out);
uint16_t Vision_Crc16(const uint8_t *data, uint16_t length);
uint8_t Vision_CanRun(const VisionCommand *cmd, uint32_t now);
#endif
