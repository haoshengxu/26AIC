#ifndef DISTANCE_MEASUREMENT_H
#define DISTANCE_MEASUREMENT_H
#include <stdint.h>

#define DISTANCE_TIMEOUT_MS 250U
typedef enum {
    DISTANCE_WAIT = 0,
    DISTANCE_VALID,
    DISTANCE_INVALID,
    DISTANCE_TIMEOUT,
    DISTANCE_UART_ERROR
} DistanceState;

typedef struct {
    int16_t distance_raw; /* 0.01 cm, right positive; zeroed when unusable. */
    uint16_t sequence;
    uint8_t measurement_valid, link_alive, has_frame;
    DistanceState state;
    uint32_t received_ms, last_valid_ms;
    uint32_t accepted, valid_frames, invalid_frames;
    uint32_t crc_errors, format_errors, uart_errors, timeouts;
} DistanceMeasurement;

void Distance_Init(void); /* Before enabling USART1. */
/* Exactly 14 bytes, called only by the shared USART1 stream parser. */
uint8_t Distance_ProcessFrame(const uint8_t *frame, uint32_t now);
void Distance_FormatError(void);
void Distance_UartError(void);
void Distance_Tick(void); /* 10 ms control tick, even without RX. */
void Distance_Get(DistanceMeasurement *out); /* Coherent, freshness-checked snapshot. */
const char *Distance_StateText(DistanceState state);
#endif
