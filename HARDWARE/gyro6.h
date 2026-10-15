#ifndef __GYRO6_H
#define __GYRO6_H

#include "main.h"

/* Six-axis gyro UART protocol. */
#define GYRO6_FRAME_HEADER  0x5A
#define GYRO6_TYPE_ANGLE   0xBB
#define GYRO6_FRAME_LENGTH 11

typedef struct
{
    float roll;
    float pitch;
    float yaw;
} Gyro6_Angle_t;

void Gyro6_ProcessByte(uint8_t byte);
float Gyro6_GetYaw(void);
float Gyro6_GetRoll(void);
float Gyro6_GetPitch(void);
uint8_t Gyro6_IsValid(void);

#endif

