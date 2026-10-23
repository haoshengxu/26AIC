/* Regression tests execute the production OLED renderer with GPIO mocked.
 * They validate the framebuffer, not an emulated physical OLED controller. */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#define __MAIN_H
#define __SYS_H
#define __SYSTEM_H
#define __DELAY_H
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
static uint32_t gpio_pins[16];
#define PDout(pin) gpio_pins[pin]
typedef struct { float Encoder, Motor_Pwm, Target, Velocity_KP, Velocity_KI; } Motor_parameter;
static Motor_parameter MOTOR_A, MOTOR_B;
static float Voltage = 12.0f;
static int Servo = 1500;
#include "../BALANCE/mode_control.h"
static ModeControl_Status_t fake_status;
static float fake_yaw = 37.0f;
static uint8_t fake_valid = 1U;
void delay_ms(u16 ms) { (void)ms; }
void HAL_Delay(uint32_t ms) { (void)ms; }
static uint32_t fake_ms=1000U;
uint32_t HAL_GetTick(void) { return fake_ms; }
float Get_battery_volt(void) { return 12.0f; }
float Gyro6_GetYaw(void) { return fake_yaw; }
uint8_t Gyro6_IsValid(void) { return fake_valid; }
float ModeControl_GetTargetYaw(void) { return 37.0f; }
uint32_t ModeControl_GetElapsedMs(void) { return 1000U; }
static ModeControl_Mode_t fake_mode=MODE_CONTROL_STRAIGHT_5S;
ModeControl_Mode_t ModeControl_GetDisplayMode(void) { return fake_mode; }
ModeControl_Status_t ModeControl_GetStatus(void) { return fake_status; }
#include "../HARDWARE/distance_measurement.h"
static DistanceMeasurement fake_distance;
void Distance_Get(DistanceMeasurement *d) { *d=fake_distance; }
const char *Distance_StateText(DistanceState s) { (void)s; return "WAIT"; }
void Tracking_GetInput(TrackingInput *p) { memset(p,0,sizeof(*p)); }
uint8_t Tracking_CanRun(const TrackingInput *p,uint32_t now) { (void)p;(void)now;return 0; }
void ModeControl_GetTrackingOutput(TrackingOutput *p) { memset(p,0,sizeof(*p)); }
void Tracking_GetParameters(TrackingParameters *p) { p->kp=TRACK_KP;p->kd=TRACK_KD;p->stanley_k=TRACK_STANLEY_K;p->speed_mps=TRACK_BASE_SPEED_MPS;p->filter_tau_s=TRACK_D_FILTER_TAU_S; }
#include "../HARDWARE/vision_command.h"
void Vision_GetTarget(VisionTarget *p) { memset(p,0,sizeof(*p)); }
void Vision_GetControlTarget(VisionTarget *p) { memset(p,0,sizeof(*p)); }
#include "../HARDWARE/oled.c"
#include "../BALANCE/show.c"

static unsigned int assertions;
static unsigned int case_id;
static void finish(unsigned int line)
{
    volatile uint32_t *mailbox = (volatile uint32_t *)0x2001FFF0U;
    mailbox[0] = assertions; mailbox[1] = line;
    mailbox[2] = case_id; mailbox[3] = 0xACCE5501U;
    __asm { BKPT 0 }
    for (;;) { }
}
#define REQUIRE(expr) do { ++assertions; if (!(expr)) finish(__LINE__); } while (0)
static unsigned int pixels(unsigned int x0, unsigned int y0, unsigned int width, unsigned int height)
{
    unsigned int x, y, count = 0;
    for (x = x0; x < x0 + width; ++x)
        for (y = y0; y < y0 + height; ++y)
            if (OLED_GRAM[x][7U - y / 8U] & (1U << (7U - y % 8U))) ++count;
    return count;
}

void test_entry(void)
{
    unsigned int i;
    u8 snapshot[128][8];
    case_id = 1;
    MOTOR_A.Target = 0.3f; MOTOR_A.Encoder = 0.29f;
    MOTOR_B.Target = 0.3f; MOTOR_B.Encoder = 0.31f;
    OLED_Clear();
    oled_show();
    /* This fails with the previous driver/layout: the final padded line
     * wraps past the bottom and clears the screen, then draws spaces. */
    REQUIRE(pixels(0, 0, 128, 64) > 0U);
    for (i = 0; i < 5; ++i) REQUIRE(pixels(0, i * 12, 126, 12) > 0U);
    REQUIRE(pixels(0, 60, 128, 4) == 0U);
    fake_mode=MODE_CONTROL_DEBUG;
    fake_distance.measurement_valid=1;
    fake_distance.distance_raw=-32768;
    fake_distance.has_frame=1;
    fake_distance.accepted=123;
    for(fake_ms=1000;fake_ms<=4000;fake_ms+=3000) {
        OLED_Clear(); oled_show();
        for(i=0;i<5;i++) REQUIRE(pixels(0,i*12,126,12)>0);
        REQUIRE(pixels(0,60,128,4)==0);
    }
    fake_mode=MODE_CONTROL_VISION_LINE;
    for(fake_ms=1000;fake_ms<=4000;fake_ms+=3000) {
        OLED_Clear(); oled_show();
        for(i=0;i<5;i++) REQUIRE(pixels(0,i*12,126,12)>0);
        REQUIRE(pixels(0,60,128,4)==0);
    }
    memcpy(snapshot, OLED_GRAM, sizeof(snapshot));
    fake_mode=MODE_CONTROL_VISION_NO_BLIND;
    OLED_Clear();oled_show();
    REQUIRE(memcmp(snapshot,OLED_GRAM,sizeof(snapshot))!=0);
    fake_mode=MODE_CONTROL_STRAIGHT_5S;

    case_id = 2;
    OLED_Clear();
    OLED_ShowString(0, 0, (const u8 *)"12345678901234567890X");
    REQUIRE(pixels(120, 0, 6, 12) > 0U);
    REQUIRE(pixels(0, 12, 128, 52) == 0U);
    OLED_Clear();
    OLED_DrawPoint(0, 0, 1);
    OLED_ShowString(120, 48, (const u8 *)"ABC");
    REQUIRE(pixels(0, 0, 1, 1) == 1U);
    REQUIRE(pixels(120, 48, 6, 12) > 0U);
    memcpy(snapshot, OLED_GRAM, sizeof(snapshot));
    OLED_ShowString(0, 60, (const u8 *)"OFFSCREEN");
    OLED_ShowString(127, 0, (const u8 *)"OFFSCREEN");
    REQUIRE(memcmp(snapshot, OLED_GRAM, sizeof(snapshot)) == 0);

    case_id = 3;
    OLED_Clear();
    fake_yaw = -179.9f; fake_status = MODE_STATUS_RUNNING;
    oled_show();
    fake_yaw = 179.9f; fake_status = MODE_STATUS_IMU_ERROR; fake_valid = 0U;
    oled_show();
    for (i = 0; i < 5; ++i) REQUIRE(pixels(0, i * 12, 126, 12) > 0U);
    REQUIRE(pixels(126, 0, 2, 64) == 0U);
    REQUIRE(pixels(0, 60, 128, 4) == 0U);
    finish(0U);
}
