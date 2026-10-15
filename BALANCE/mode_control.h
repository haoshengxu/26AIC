#ifndef __MODE_CONTROL_H
#define __MODE_CONTROL_H
#include "stm32f4xx_hal.h"
#include "tracking_control.h"

#define MODE_USER_KEY_GPIO_PORT GPIOE
#define MODE_USER_KEY_PIN GPIO_PIN_0
#define MODE_USER_KEY_ACTIVE_LEVEL GPIO_PIN_RESET
#define MODE_BUZZER_GPIO_PORT GPIOA
#define MODE_BUZZER_PIN GPIO_PIN_8
#define MODE_BUZZER_ACTIVE_LEVEL GPIO_PIN_SET
#define MODE_BUZZER_INACTIVE_LEVEL GPIO_PIN_RESET

typedef enum {
    MODE_CONTROL_STRAIGHT_5S = 1,
    MODE_CONTROL_TURN_90 = 2,
    MODE_CONTROL_VISION_LINE = 3,
    MODE_CONTROL_DEBUG = 4
} ModeControl_Mode_t;

typedef enum {
    MODE_STATUS_READY = 0,
    MODE_STATUS_RUNNING,
    MODE_STATUS_DONE,
    MODE_STATUS_STOPPED,
    MODE_STATUS_IMU_ERROR,
    MODE_STATUS_POWER_ERROR,
    MODE_STATUS_TIMEOUT,
    MODE_STATUS_WAIT_COMMAND,
    MODE_STATUS_LINK_ERROR,
    MODE_STATUS_REMOTE_STOP,
    MODE_STATUS_CONTROL_ERROR
} ModeControl_Status_t;

void ModeControl_GetTrackingOutput(TrackingOutput *out);
void ModeControl_Init(void);
void ModeControl_ButtonTask(void);
void ModeControl_Tick10ms(void);
uint8_t ModeControl_IsActive(void);
ModeControl_Mode_t ModeControl_GetDisplayMode(void);
ModeControl_Status_t ModeControl_GetStatus(void);
uint32_t ModeControl_GetElapsedMs(void);
float ModeControl_GetTargetYaw(void);
uint8_t ModeControl_DebugCommand(const char *key, float value);
float ModeControl_DebugLeft(void);
float ModeControl_DebugRight(void);
float ModeControl_DebugAngle(void);
#endif
