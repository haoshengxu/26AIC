#include "gyro6.h"
#include "ackermann_config.h"
#include <string.h>

/*
 * Six-axis gyro angle frame (11 bytes):
 * 5A BB RollL RollH PitchL PitchH YawL YawH XX XX SUM
 *
 * Angle is a signed short in the range -180 to 180 degrees:
 * angle = raw / 32768 * 180
 */
static volatile Gyro6_Angle_t s_angle = {0.0f, 0.0f, 0.0f};
static volatile uint8_t s_valid = 0;
static volatile uint32_t s_last_frame_ms = 0U;
static uint8_t s_frame[GYRO6_FRAME_LENGTH];
static uint8_t s_frame_index = 0;

static uint8_t Gyro6_Checksum(const uint8_t *frame)
{
    uint16_t sum = 0;
    uint8_t i;

    for (i = 0; i < GYRO6_FRAME_LENGTH - 1; i++)
    {
        sum += frame[i];
    }

    return (uint8_t)sum;
}

static int16_t Gyro6_ReadInt16(uint8_t low, uint8_t high)
{
    return (int16_t)(((uint16_t)high << 8) | low);
}

void Gyro6_ProcessByte(uint8_t byte)
{
    int16_t roll_raw;
    int16_t pitch_raw;
    int16_t yaw_raw;

    if (s_frame_index == 0)
    {
        if (byte != GYRO6_FRAME_HEADER)
        {
            return;
        }
    }

    s_frame[s_frame_index++] = byte;

    if (s_frame_index < GYRO6_FRAME_LENGTH)
    {
        return;
    }

    if ((s_frame[1] == GYRO6_TYPE_ANGLE) &&
        (Gyro6_Checksum(s_frame) == s_frame[GYRO6_FRAME_LENGTH - 1]))
    {
        roll_raw = Gyro6_ReadInt16(s_frame[2], s_frame[3]);
        pitch_raw = Gyro6_ReadInt16(s_frame[4], s_frame[5]);
        yaw_raw = Gyro6_ReadInt16(s_frame[6], s_frame[7]);

        s_angle.roll = (float)roll_raw * 180.0f / 32768.0f;
        s_angle.pitch = (float)pitch_raw * 180.0f / 32768.0f;
        s_angle.yaw = (float)yaw_raw * 180.0f / 32768.0f;
        s_last_frame_ms = HAL_GetTick();
        s_valid = 1;
        s_frame_index = 0;
        return;
    }

    /* Retain a possible next header after a corrupt/dropped byte. */
    {
        uint8_t i;
        for (i = 1U; i < GYRO6_FRAME_LENGTH; ++i) {
            if (s_frame[i] == GYRO6_FRAME_HEADER) {
                s_frame_index = GYRO6_FRAME_LENGTH - i;
                memmove(s_frame, &s_frame[i], s_frame_index);
                return;
            }
        }
    }
    s_frame_index = 0;
}

float Gyro6_GetYaw(void)
{
    return s_angle.yaw;
}

float Gyro6_GetRoll(void)
{
    return s_angle.roll;
}

float Gyro6_GetPitch(void)
{
    return s_angle.pitch;
}

uint8_t Gyro6_IsValid(void)
{
    return (s_valid && (uint32_t)(HAL_GetTick() - s_last_frame_ms) <= ACK_IMU_TIMEOUT_MS) ? 1U : 0U;
}

