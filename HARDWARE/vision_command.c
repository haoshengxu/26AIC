#include "vision_command.h"
#include "distance_measurement.h"
#include "stm32f4xx_hal.h"
#include <string.h>

/* AA 55 01 0E seq_lo seq_hi flags 00 v_lo v_hi steer_lo steer_hi crc_lo crc_hi.
 * CRC16/CCITT-FALSE covers bytes 0..11; poly=1021, init=FFFF, no reflection.
 * The UART ISR is the sole writer. Readers briefly mask interrupts to obtain
 * a consistent command and counters. No floats, prints or motor writes here. */
static volatile VisionCommand latest;
static volatile VisionTarget target_latest;
static volatile VisionTarget target_control;
static uint8_t buffer[VISION_FRAME_SIZE], used;

uint16_t Vision_Crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU, i;
    uint8_t bit;
    for (i = 0U; i < length; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (bit = 0U; bit < 8U; ++bit)
            crc = (uint16_t)((crc & 0x8000U) ? (crc << 1) ^ 0x1021U : crc << 1);
    }
    return crc;
}

void Vision_Init(void)
{
    memset((void *)&latest, 0, sizeof(latest));
    memset((void *)&target_latest, 0, sizeof(target_latest));
    memset((void *)&target_control, 0, sizeof(target_control));
    used = 0U;
    Distance_Init();
}

void Vision_Get(VisionCommand *out)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    *out = latest;
    __set_PRIMASK(mask);
}

void Vision_GetTarget(VisionTarget *out)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (target_latest.link_alive &&
        (uint32_t)(HAL_GetTick() - target_latest.received_ms) >= VISION_TARGET_TIMEOUT_MS) {
        target_latest.link_alive = 0U;
        target_latest.valid = 0U;
        target_latest.x_centi_cm = target_latest.y_centi_cm = 0;
        ++target_latest.timeouts;
    }
    *out = target_latest;
    __set_PRIMASK(mask);
}

void Vision_GetControlTarget(VisionTarget *out)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (target_control.valid &&
        (uint32_t)(HAL_GetTick()-target_control.received_ms) >= VISION_TARGET_HOLD_MS) {
        target_control.valid = target_control.link_alive = 0U;
        target_control.x_centi_cm = target_control.y_centi_cm = 0;
        ++target_control.timeouts;
    }
    *out = target_control;
    __set_PRIMASK(mask);
}

static uint8_t ParseTarget(uint32_t now)
{
    uint16_t seq, delta, ux, uy;
    int32_t x, y;
    uint16_t crc = (uint16_t)(buffer[12] | ((uint16_t)buffer[13] << 8));
    if (Vision_Crc16(buffer, 12U) != crc) {
        ++target_latest.crc_errors;
        return 0U;
    }
    ux = (uint16_t)(buffer[8] | ((uint16_t)buffer[9] << 8));
    uy = (uint16_t)(buffer[10] | ((uint16_t)buffer[11] << 8));
    x = ux < 32768U ? (int32_t)ux : (int32_t)ux - 65536L;
    y = uy < 32768U ? (int32_t)uy : (int32_t)uy - 65536L;
    if (buffer[6] > 1U || buffer[7] != 0U ||
        (!buffer[6] && (x != 0 || y != 0)) || (buffer[6] && y <= 0)) {
        ++target_latest.format_errors;
        return 0U;
    }
    seq = (uint16_t)(buffer[4] | ((uint16_t)buffer[5] << 8));
    delta = (uint16_t)(seq - target_latest.sequence);
    if (target_latest.has_frame) {
        if (delta == 0U) ++target_latest.duplicate_frames;
        else if (delta != 1U) ++target_latest.sequence_discontinuities;
    }
    /* Sequence is diagnostic: restart/wrap/duplicates must not lock reception.
     * Invalid frames indicate a live link, but immediately clear the target.
     * V3 never updates V1 commands or V2 tracking input. */
    target_latest.sequence = seq;
    target_latest.x_centi_cm = (int16_t)x;
    target_latest.y_centi_cm = (int16_t)y;
    target_latest.valid = buffer[6];
    target_latest.has_frame = target_latest.link_alive = 1U;
    target_latest.received_ms = now;
    ++target_latest.accepted;
    if (!buffer[6]) { ++target_latest.invalid_frames; ++target_latest.invalid_events; }
    else {
        /* Commit XY and receipt time together. Only valid V3 frames refresh
         * the motion hold, including a valid X=0 and restarted sequence. */
        target_control.x_centi_cm = (int16_t)x;
        target_control.y_centi_cm = (int16_t)y;
        target_control.sequence = seq;
        target_control.received_ms = now;
        target_control.has_frame = target_control.link_alive = target_control.valid = 1U;
        ++target_control.accepted;
    }
    return 1U;
}

uint8_t Vision_CanRun(const VisionCommand *cmd, uint32_t now)
{
    return cmd->valid && (uint32_t)(now - cmd->received_ms) <= VISION_TIMEOUT_MS &&
           (cmd->flags & (VISION_RUN | VISION_LINE | VISION_STOP)) ==
           (VISION_RUN | VISION_LINE);
}

void Vision_UartError(void)
{
    target_control.valid = target_control.link_alive = 0U;
    target_control.x_centi_cm = target_control.y_centi_cm = 0;
    target_latest.valid = target_latest.link_alive = 0U;
    target_latest.x_centi_cm = target_latest.y_centi_cm = 0;
    ++target_latest.uart_errors;
    Distance_UartError();
    ++latest.uart_errors;
    /* Latch a safety event: a subsequent good frame cannot hide a UART fault. */
    ++latest.stop_events;
    latest.valid = 0U;
    used = 0U;
}

static uint8_t Parse(uint32_t now)
{
    uint16_t seq, crc, delta;
    int16_t speed, steer;
    uint8_t flags;
    /* V2 measurements never reach V1 motion/STOP/sequence handling. */
    if (buffer[2] == 3U) return ParseTarget(now);
    if (buffer[2] == 2U) return Distance_ProcessFrame(buffer, now);
    crc = (uint16_t)(buffer[12] | ((uint16_t)buffer[13] << 8));
    if (Vision_Crc16(buffer, 12U) != crc) {
        ++latest.crc_errors;
        return 0U;
    }
    flags = buffer[6];
    if (buffer[0] != 0xAAU || buffer[1] != 0x55U ||
        buffer[2] != 1U || buffer[3] != VISION_FRAME_SIZE ||
        buffer[7] != 0U || (flags & ~0x07U) != 0U) {
        ++latest.format_errors;
        return 0U;
    }
    speed = (int16_t)(buffer[8] | ((uint16_t)buffer[9] << 8));
    steer = (int16_t)(buffer[10] | ((uint16_t)buffer[11] << 8));
    if (speed < 0 || speed > VISION_MAX_SPEED_MMPS ||
        steer < -VISION_MAX_STEER_MRAD || steer > VISION_MAX_STEER_MRAD) {
        ++latest.range_errors;
        return 0U;
    }
    /* Honor a valid STOP/line-loss even if its sequence was duplicated. */
    if ((flags & VISION_STOP) || !(flags & VISION_LINE) || !(flags & VISION_RUN))
        ++latest.stop_events;
    seq = (uint16_t)(buffer[4] | ((uint16_t)buffer[5] << 8));
    delta = (uint16_t)(seq - latest.sequence);
    if (latest.valid && (uint32_t)(now - latest.received_ms) <= VISION_TIMEOUT_MS &&
        (delta == 0U || delta >= 0x8000U)) {
        ++latest.sequence_errors;
        return 0U;
    }
    latest.sequence = seq;
    latest.speed_mmps = speed;
    latest.steer_mrad = steer;
    latest.flags = flags;
    latest.received_ms = now;
    latest.valid = 1U;
    ++latest.accepted;
    return 1U;
}

/* Shared bounded stream state machine for V1 control, V2 distance and V3 XY measurement.
 * Only this ISR writes the 14-byte candidate. No allocation or RX backlog.
 * WAIT_AA / WAIT_55 / WAIT_HEADER / WAIT_FRAME are represented by used.
 * On rejection slide ONE byte, retaining every possible nested header.
 * A complete accepted frame is consumed whole, including payload AA 55. */
void Vision_ProcessByte(uint8_t byte)
{
    uint32_t now = HAL_GetTick();
    ++latest.bytes;
    buffer[used++] = byte; /* used is always < 14 on entry. */
    while (used) {
        if (buffer[0] != 0xAAU) goto slide;
        if (used < 2U) return;
        if (buffer[1] != 0x55U) goto slide;
        if (used < 4U) return;
        if ((buffer[2] != 1U && buffer[2] != 2U && buffer[2] != 3U) || buffer[3] != VISION_FRAME_SIZE) {
            if (buffer[2] == 3U) ++target_latest.format_errors;
            else if (buffer[2] == 2U) Distance_FormatError();
            else ++latest.format_errors;
            goto slide;
        }
        if (used < VISION_FRAME_SIZE) return;
        if (Parse(now)) { used = 0U; return; }
slide:
        --used;
        memmove(buffer, buffer + 1, used);
    }
}
