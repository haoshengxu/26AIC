#include "vision_command.h"
#include "distance_measurement.h"
#include "stm32f4xx_hal.h"
#include <string.h>

/* AA 55 01 0E seq_lo seq_hi flags 00 v_lo v_hi steer_lo steer_hi crc_lo crc_hi.
 * CRC16/CCITT-FALSE covers bytes 0..11; poly=1021, init=FFFF, no reflection.
 * The UART ISR is the sole writer. Readers briefly mask interrupts to obtain
 * a consistent command and counters. V4 copies validated float bits only;
 * there is no floating arithmetic, printing or motor control in this ISR. */
static volatile VisionCommand latest;
static volatile VisionTarget target_latest;
static volatile VisionTarget target_control;
static uint8_t buffer[VISION_HEADING_FRAME_SIZE], used;

static void ClearRoute(volatile VisionTarget *target)
{
    target->valid = target->flags = target->mode = target->fallback = 0U;
    target->endpoint_valid = target->curvature_valid = 0U;
    target->x_centi_cm = target->y_centi_cm = 0;
    target->endpoint_x_centi_cm = target->endpoint_y_centi_cm = 0;
    target->endpoint_abs_curvature_cm_inv = 0.0f;
    target->heading_valid = 0U;
    target->heading_centi_deg = 0;
    target->heading_reference_x_centi_cm = target->heading_reference_y_centi_cm = 0;
    target->tape_detected = 0U;
    target->tape_distance_y_centi_cm = 0U;
}

static int16_t ReadSigned16(uint8_t offset)
{
    uint16_t value = (uint16_t)(buffer[offset] | ((uint16_t)buffer[offset+1U] << 8));
    return (int16_t)(value < 32768U ? (int32_t)value : (int32_t)value-65536L);
}

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
        ClearRoute(&target_latest);
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
        ClearRoute(&target_control);
        ++target_control.timeouts;
    }
    *out = target_control;
    __set_PRIMASK(mask);
}

static uint8_t ParseTarget(uint32_t now)
{
    uint16_t seq, delta;
    int16_t x, y;
    uint8_t v6 = buffer[2] == 6U;
    uint8_t v5 = buffer[2] == 5U || v6;
    uint8_t v4 = buffer[2] == 4U || v5;
    uint8_t flags = buffer[6], mode = buffer[7];
    uint8_t crc_offset = v6 ? 30U : v5 ? 23U : (v4 ? 20U : 12U);
    uint32_t bits = 0U;
    float curvature = 0.0f;
    uint16_t crc = (uint16_t)(buffer[crc_offset] | ((uint16_t)buffer[crc_offset+1U] << 8));
    if (Vision_Crc16(buffer, crc_offset) != crc) {
        ++target_latest.crc_errors;
        return 0U;
    }
    x = ReadSigned16(8U);
    y = ReadSigned16(10U);
    if (v4) {
        if (mode > 3U || (flags & 0xF0U) || (!mode && flags) ||
            ((flags & VISION_CURVATURE_VALID) && !(flags & VISION_ENDPOINT_VALID)) ||
            ((flags & VISION_TARGET_FALLBACK) && !(flags & VISION_TARGET_VALID)) ||
            ((flags & VISION_TARGET_VALID) && y <= 0)) goto bad_format;
        if (flags & VISION_CURVATURE_VALID) {
            bits = (uint32_t)buffer[16] | ((uint32_t)buffer[17]<<8) |
                   ((uint32_t)buffer[18]<<16) | ((uint32_t)buffer[19]<<24);
            /* Validate IEEE754 bits without floating arithmetic in the UART ISR.
             * Reject NaN, infinity and negative nonzero; accept valid +/-zero. */
            if ((bits & 0x7F800000U) == 0x7F800000U ||
                ((bits & 0x80000000U) && (bits & 0x7FFFFFFFU))) goto bad_format;
            memcpy(&curvature, &bits, sizeof(curvature));
        }
    } else if (flags > 1U || mode != 0U ||
        (!flags && (x != 0 || y != 0)) || (flags && y <= 0)) {
        goto bad_format;
    }
    /* V5 requires zero payload for unavailable measurements. Preserve V4's
     * historical ignore-and-clear behaviour for compatibility. */
    if (v5 && (buffer[20] > 1U ||
        (!buffer[20] && (buffer[21] || buffer[22])) ||
        (!(flags & VISION_TARGET_VALID) && (x || y)) ||
        (!(flags & VISION_ENDPOINT_VALID) &&
         (buffer[12] || buffer[13] || buffer[14] || buffer[15])) ||
        (!(flags & VISION_CURVATURE_VALID) &&
         (buffer[16] || buffer[17] || buffer[18] || buffer[19])))) goto bad_format;
    if (v6 && (buffer[23] > 1U ||
        (!buffer[23] && (buffer[24] || buffer[25] || buffer[26] || buffer[27] || buffer[28] || buffer[29])))) goto bad_format;
    seq = (uint16_t)(buffer[4] | ((uint16_t)buffer[5] << 8));
    delta = (uint16_t)(seq - target_latest.sequence);
    if (target_latest.has_frame) {
        if (delta == 0U) ++target_latest.duplicate_frames;
        else if (delta != 1U) ++target_latest.sequence_discontinuities;
    }
    /* Sequence is diagnostic: restart/wrap/duplicates must not lock reception.
     * Invalid frames indicate a live link, but immediately clear the target.
     * V3/V4/V5 never update V1 commands or V2 tracking input. */
    target_latest.sequence = seq;
    ClearRoute(&target_latest);
    target_latest.version = buffer[2];
    target_latest.flags = flags;
    target_latest.mode = v4 ? mode : 0U;
    target_latest.valid = (flags & VISION_TARGET_VALID) != 0U;
    if (target_latest.valid) {
        target_latest.x_centi_cm = x;
        target_latest.y_centi_cm = y;
    }
    if (v4) {
        target_latest.fallback = (flags & VISION_TARGET_FALLBACK) != 0U;
        target_latest.endpoint_valid = (flags & VISION_ENDPOINT_VALID) != 0U;
        target_latest.curvature_valid = (flags & VISION_CURVATURE_VALID) != 0U;
        if (target_latest.endpoint_valid) {
            target_latest.endpoint_x_centi_cm = ReadSigned16(12U);
            target_latest.endpoint_y_centi_cm = ReadSigned16(14U);
        }
        target_latest.endpoint_abs_curvature_cm_inv = curvature;
    }
    if (v5) {
        target_latest.tape_detected = buffer[20];
        target_latest.tape_distance_y_centi_cm = (uint16_t)(buffer[21] | ((uint16_t)buffer[22] << 8));
    }
    if (v6 && buffer[23]) {
        target_latest.heading_valid = 1U;
        target_latest.heading_centi_deg = ReadSigned16(24U);
        target_latest.heading_reference_x_centi_cm = ReadSigned16(26U);
        target_latest.heading_reference_y_centi_cm = ReadSigned16(28U);
    }
    /* Heading is deliberately never copied into the one-second XY hold. */
    target_latest.has_frame = target_latest.link_alive = 1U;
    target_latest.received_ms = now;
    ++target_latest.accepted;
    if (!target_latest.valid) { ++target_latest.invalid_frames; ++target_latest.invalid_events; }
    else {
        /* Commit XY and receipt time together. Only valid V3/V4/V5 targets refresh
         * the motion hold, including a valid X=0 and restarted sequence. */
        target_control.x_centi_cm = (int16_t)x;
        target_control.y_centi_cm = (int16_t)y;
        target_control.sequence = seq;
        target_control.received_ms = now;
        target_control.has_frame = target_control.link_alive = target_control.valid = 1U;
        target_control.version = target_latest.version;
        target_control.flags = target_latest.flags;
        target_control.mode = target_latest.mode;
        target_control.fallback = target_latest.fallback;
        target_control.endpoint_valid = target_latest.endpoint_valid;
        target_control.curvature_valid = target_latest.curvature_valid;
        target_control.endpoint_x_centi_cm = target_latest.endpoint_x_centi_cm;
        target_control.endpoint_y_centi_cm = target_latest.endpoint_y_centi_cm;
        target_control.endpoint_abs_curvature_cm_inv = curvature;
        ++target_control.accepted;
    }
    return 1U;
bad_format:
    ++target_latest.format_errors;
    return 0U;
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
    ClearRoute(&target_control);
    target_latest.valid = target_latest.link_alive = 0U;
    ClearRoute(&target_latest);
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
    if (buffer[2] == 3U || buffer[2] == 4U || buffer[2] == 5U || buffer[2] == 6U) return ParseTarget(now);
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

/* Bounded stream parser: V1/V2/V3 use 14 bytes, V4 uses 22 bytes, V5 uses 25 bytes.
 * Only this ISR writes the candidate. No allocation or RX backlog.
 * WAIT_AA / WAIT_55 / WAIT_HEADER / WAIT_FRAME are represented by used.
 * On rejection slide ONE byte, retaining every possible nested header.
 * A complete accepted frame is consumed whole, including payload AA 55. */
void Vision_ProcessByte(uint8_t byte)
{
    uint32_t now = HAL_GetTick();
    uint8_t expected;
    ++latest.bytes;
    buffer[used++] = byte; /* used is always < 32 on entry. */
    while (used) {
        if (buffer[0] != 0xAAU) goto slide;
        if (used < 2U) return;
        if (buffer[1] != 0x55U) goto slide;
        if (used < 4U) return;
        expected = buffer[2] == 6U ? VISION_HEADING_FRAME_SIZE :
                   buffer[2] == 5U ? VISION_TAPE_FRAME_SIZE :
                   (buffer[2] == 4U ? VISION_ROUTE_FRAME_SIZE : VISION_FRAME_SIZE);
        if ((buffer[2] < 1U || buffer[2] > 6U) || buffer[3] != expected) {
            if (buffer[2] == 3U || buffer[2] == 4U || buffer[2] == 5U || buffer[2] == 6U) ++target_latest.format_errors;
            else if (buffer[2] == 2U) Distance_FormatError();
            else ++latest.format_errors;
            goto slide;
        }
        if (used < expected) return;
        if (Parse(now)) {
            used = (uint8_t)(used-expected);
            memmove(buffer, buffer+expected, used);
            continue;
        }
slide:
        --used;
        memmove(buffer, buffer + 1, used);
    }
}

uint8_t Vision_BuildLaneCommand(uint8_t out[9], uint16_t sequence, uint8_t mode)
{
    uint16_t crc;
    if(out==0 || mode>5U) return 0;
    out[0]=0x55;out[1]=0xAA;out[2]=1;out[3]=9;
    out[4]=(uint8_t)sequence;out[5]=(uint8_t)(sequence>>8);out[6]=mode;
    crc=Vision_Crc16(out,7);out[7]=(uint8_t)crc;out[8]=(uint8_t)(crc>>8);
    return 9;
}

static volatile uint8_t lane_pending, lane_mode;
static uint16_t lane_sequence;
void Vision_RequestLane(uint8_t mode)
{
    if(mode<=5U){lane_mode=mode;lane_pending=1;}
}
uint8_t Vision_TakeLaneCommand(uint8_t out[9])
{
    uint32_t mask=__get_PRIMASK();uint8_t mode;
    __disable_irq();
    if(!lane_pending){__set_PRIMASK(mask);return 0;}
    mode=lane_mode;lane_pending=0;
    __set_PRIMASK(mask);
    return Vision_BuildLaneCommand(out,++lane_sequence,mode);
}
