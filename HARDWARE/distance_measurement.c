#include "distance_measurement.h"
#include "tracking_control.h"
#include "vision_command.h"
#include "stm32f4xx_hal.h"
#include <string.h>

static volatile DistanceMeasurement distance_latest;

void Distance_Init(void)
{
    memset((void *)&distance_latest, 0, sizeof(distance_latest));
    distance_latest.state = DISTANCE_WAIT;
    Tracking_InputInit();
}

static void Distance_Expire(uint32_t now)
{
    if (distance_latest.link_alive &&
        (uint32_t)(now - distance_latest.received_ms) > DISTANCE_TIMEOUT_MS) {
        distance_latest.link_alive = 0U;
        distance_latest.measurement_valid = 0U;
        distance_latest.distance_raw = 0;
        distance_latest.state = DISTANCE_TIMEOUT;
        ++distance_latest.timeouts;
    }
}

void Distance_Tick(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    /* Read time after masking: RX may have preempted the timer beforehand. */
    Distance_Expire(HAL_GetTick());
    __set_PRIMASK(mask);
}

void Distance_Get(DistanceMeasurement *out)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    Distance_Expire(HAL_GetTick());
    *out = distance_latest;
    __set_PRIMASK(mask);
}

void Distance_FormatError(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    ++distance_latest.format_errors;
    __set_PRIMASK(mask);
}

void Distance_UartError(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    Tracking_UartError();
    ++distance_latest.uart_errors;
    distance_latest.link_alive = 0U;
    distance_latest.measurement_valid = 0U;
    distance_latest.distance_raw = 0;
    distance_latest.state = DISTANCE_UART_ERROR;
    __set_PRIMASK(mask);
}

uint8_t Distance_ProcessFrame(const uint8_t *frame, uint32_t now)
{
    uint16_t raw = (uint16_t)(frame[10] | ((uint16_t)frame[11] << 8));
    uint16_t crc = (uint16_t)(frame[12] | ((uint16_t)frame[13] << 8));
    int32_t signed_raw = raw < 32768U ? (int32_t)raw : (int32_t)raw - 65536L;
    uint32_t mask;
    if (Vision_Crc16(frame, 12U) != crc) {
        mask = __get_PRIMASK();
        __disable_irq();
        ++distance_latest.crc_errors;
        __set_PRIMASK(mask);
        return 0U;
    }
    if (frame[0] != 0xAAU || frame[1] != 0x55U || frame[2] != 2U ||
        frame[3] != 14U || frame[6] > 1U || frame[7] || frame[8] || frame[9] ||
        (frame[6] == 0U && raw != 0U)) {
        Distance_FormatError();
        return 0U;
    }
    mask = __get_PRIMASK();
    __disable_irq();
    distance_latest.sequence = (uint16_t)(frame[4] | ((uint16_t)frame[5] << 8));
    Tracking_OnFrame((int16_t)signed_raw, distance_latest.sequence, frame[6], now);
    distance_latest.distance_raw = (int16_t)signed_raw;
    distance_latest.measurement_valid = frame[6];
    distance_latest.has_frame = 1U;
    distance_latest.link_alive = 1U;
    distance_latest.received_ms = now;
    ++distance_latest.accepted;
    if (frame[6]) {
        distance_latest.state = DISTANCE_VALID;
        distance_latest.last_valid_ms = now;
        ++distance_latest.valid_frames;
    } else {
        distance_latest.state = DISTANCE_INVALID;
        ++distance_latest.invalid_frames;
    }
    __set_PRIMASK(mask);
    return 1U;
}

const char *Distance_StateText(DistanceState state)
{
    switch (state) {
    case DISTANCE_VALID: return "VALID";
    case DISTANCE_INVALID: return "INVALID";
    case DISTANCE_TIMEOUT: return "TIMEOUT";
    case DISTANCE_UART_ERROR: return "UART!";
    default: return "WAIT";
    }
}
