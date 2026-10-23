#include "show.h"
#include "gyro6.h"
#include "mode_control.h"
#include "vision_command.h"
#include "distance_measurement.h"
#include <stdio.h>

static void ShowLine(u8 y, char *text)
{
    unsigned int length = (unsigned int)strlen(text);
    while (length < OLED_TEXT_COLUMNS) text[length++] = ' ';
    text[OLED_TEXT_COLUMNS] = '\0';
    OLED_ShowString(0, y, (const u8 *)text);
}

static const char *StatusText(void)
{
    switch (ModeControl_GetStatus()) {
    case MODE_STATUS_RUNNING: return "RUN";
    case MODE_STATUS_DONE: return "DONE";
    case MODE_STATUS_STOPPED: return "STOP";
    case MODE_STATUS_IMU_ERROR: return "IMU!";
    case MODE_STATUS_POWER_ERROR: return "PWR!";
    case MODE_STATUS_TIMEOUT: return "TIME!";
    case MODE_STATUS_WAIT_COMMAND: return "WAIT";
    case MODE_STATUS_LINK_ERROR: return "LINK!";
    case MODE_STATUS_REMOTE_STOP: return "HOST";
    case MODE_STATUS_CONTROL_ERROR: return "CTRL!";
    default: return "READY";
    }
}

static void ShowDistance(void)
{
    DistanceMeasurement d;
    char line[32];
    int32_t magnitude;
    uint32_t age;
    Distance_Get(&d);
    snprintf(line, sizeof(line), "M4 %-5s %-7s", StatusText(), Distance_StateText(d.state));
    ShowLine(0U, line);
    if (d.measurement_valid) {
        magnitude = d.distance_raw;
        if (magnitude < 0) magnitude = -magnitude; /* Safe for -32768. */
        snprintf(line, sizeof(line), "DEV:%c%ld.%02ld cm",
                 d.distance_raw < 0 ? '-' : '+', (long)(magnitude / 100),
                 (long)(magnitude % 100));
    } else {
        snprintf(line, sizeof(line), "DEV:--- cm");
    }
    ShowLine(OLED_TEXT_HEIGHT, line);
    snprintf(line, sizeof(line), "DIR:%s", !d.measurement_valid ? "---" :
             d.distance_raw < 0 ? "LEFT" : d.distance_raw > 0 ? "RIGHT" : "CENTER");
    ShowLine(2U * OLED_TEXT_HEIGHT, line);
    if (d.has_frame) {
        age = (uint32_t)(HAL_GetTick() - d.received_ms);
        if (age > 9999U) age = 9999U;
        snprintf(line, sizeof(line), "SEQ:%u AGE:%lums", (unsigned int)d.sequence,
                 (unsigned long)age);
    } else {
        snprintf(line, sizeof(line), "SEQ:--- AGE:---");
    }
    ShowLine(3U * OLED_TEXT_HEIGHT, line);
    snprintf(line, sizeof(line), "OK:%lu CRC:%lu", (unsigned long)(d.accepted % 1000000U),
             (unsigned long)(d.crc_errors % 1000000U));
    ShowLine(4U * OLED_TEXT_HEIGHT, line);
    OLED_Refresh_Gram();
}

/* Fixed-point text avoids linking printf floating-point formatting. */
static void TrackingNumber(char *dst, float value, unsigned int digits)
{
    int32_t scaled;
    uint32_t scale = digits == 4U ? 10000U : digits == 3U ? 1000U : 100U;
    uint32_t magnitude;
    scaled = (int32_t)(value * (float)scale + (value < 0.0f ? -0.5f : 0.5f));
    magnitude = (uint32_t)(scaled < 0 ? -scaled : scaled);
    snprintf(dst, 16U, "%c%lu.%0*lu", scaled < 0 ? '-' : '+',
             (unsigned long)(magnitude / scale), (int)digits,
             (unsigned long)(magnitude % scale));
}

static void ShowTracking(void)
{
    VisionTarget target;
    TrackingOutput out;
    char line[48], a[16], b[16];
    Vision_GetControlTarget(&target);
    ModeControl_GetTrackingOutput(&out);
    snprintf(line, sizeof(line), "M%u PP %-5s V:%u", (unsigned int)ModeControl_GetDisplayMode(), StatusText(), (unsigned int)target.valid);
    ShowLine(0U, line);
    TrackingNumber(a, target.x_centi_cm*0.0001f, 3U);
    TrackingNumber(b, target.y_centi_cm*0.0001f, 3U);
    snprintf(line, sizeof(line), "X:%s Y:%s m", a, b);
    ShowLine(OLED_TEXT_HEIGHT, line);
    TrackingNumber(a, out.steer_left_rad*57.2957795f, 2U);
    snprintf(line, sizeof(line), "ANG:%s S:%d", a, Servo);
    ShowLine(2U*OLED_TEXT_HEIGHT, line);
    snprintf(line, sizeof(line), "L:%+4d R:%+4d mm/s", (int)(MOTOR_A.Target*1000), (int)(MOTOR_B.Target*1000));
    ShowLine(3U*OLED_TEXT_HEIGHT, line);
    snprintf(line, sizeof(line), "SEQ:%u LINK:%u", (unsigned int)target.sequence, (unsigned int)target.link_alive);
    ShowLine(4U*OLED_TEXT_HEIGHT, line);
    OLED_Refresh_Gram();
}

void oled_show(void)
{
    char line[32];
    int yaw = (int)(Gyro6_GetYaw() * 10.0f);
    int target = (int)(ModeControl_GetTargetYaw() * 10.0f);
    unsigned int elapsed = (unsigned int)(ModeControl_GetElapsedMs() / 100U);
    unsigned int battery = Voltage > 0.0f ? (unsigned int)(Voltage * 100.0f) : 0U;
    if (ModeControl_GetDisplayMode() == MODE_CONTROL_DEBUG && (HAL_GetTick() / 3000U) % 2U == 0U) {
        ShowDistance();
        return;
    }
    if (ModeControl_GetDisplayMode() == MODE_CONTROL_VISION_LINE ||
        ModeControl_GetDisplayMode() == MODE_CONTROL_VISION_NO_BLIND) {
        ShowTracking();
        return;
    }
    /* Five non-overlapping 6x12 text rows: y=0,12,24,36,48.
     * Never use six 10-pixel rows with a 12-pixel font. */
    snprintf(line, sizeof(line), "M%u %-5s T:%2u.%us  ",
             (unsigned int)ModeControl_GetDisplayMode(), StatusText(), elapsed / 10U, elapsed % 10U);
    ShowLine(0, line);
    snprintf(line, sizeof(line), "Y:%c%03d.%d R:%c%03d.%d%c", yaw < 0 ? '-' : '+',
             abs(yaw) / 10, abs(yaw) % 10, target < 0 ? '-' : '+',
             abs(target) / 10, abs(target) % 10, Gyro6_IsValid() ? ' ' : '!');
    ShowLine(OLED_TEXT_HEIGHT, line);
    snprintf(line, sizeof(line), "L:%+4d/%+4d mm/s", (int)(MOTOR_A.Target * 1000.0f),
             (int)(MOTOR_A.Encoder * 1000.0f));
    ShowLine(2U * OLED_TEXT_HEIGHT, line);
    snprintf(line, sizeof(line), "R:%+4d/%+4d mm/s", (int)(MOTOR_B.Target * 1000.0f),
             (int)(MOTOR_B.Encoder * 1000.0f));
    ShowLine(3U * OLED_TEXT_HEIGHT, line);
    snprintf(line, sizeof(line), "S:%4dus %2u.%02uV   ", Servo, battery / 100U, battery % 100U);
    ShowLine(4U * OLED_TEXT_HEIGHT, line);
    OLED_Refresh_Gram();
}

void show_task(void)
{
    static uint32_t last_display;
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now-last_display) < 50U) return;
    last_display = now;
    /* The control ISR never blocks on ADC or OLED operations. */
    Voltage = Get_battery_volt();
    oled_show();

}
