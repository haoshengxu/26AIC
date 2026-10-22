#include "mode_control.h"
#include "balance.h"
#include "gyro6.h"
#include "ackermann_control.h"
#include "vision_command.h"
#include "distance_measurement.h"
#include <string.h>

/* Timing and gestures follow 81V1/car-mspm0-three-mode/Hardware/key.c:
 * single = start/stop, double = stop and select next task, hold = stop.
 * GPIO polarity is adapted to this board's active-low PE0 USER button. */
#define KEY_DEBOUNCE_MS 30U
#define KEY_DOUBLE_MIN_MS 50U
#define KEY_DOUBLE_MAX_MS 300U
#define KEY_LONG_MS 500U
#define FINISH_BEEP_MS 500U
#define SWITCH_BEEP_MS 50U
#define ERROR_BEEP_MS 200U

typedef enum { KEY_NONE, KEY_SINGLE, KEY_DOUBLE, KEY_LONG } KeyEvent;
static volatile ModeControl_Mode_t s_mode = MODE_CONTROL_STRAIGHT_5S;
static volatile ModeControl_Status_t s_status = MODE_STATUS_READY;
static volatile uint32_t s_elapsed_ms;
static volatile float s_target_yaw;
static uint32_t s_start_ms;
static float s_last_yaw;
static float s_turn_progress;
static AckYawPID s_yaw_pid;
static uint32_t s_buzzer_start;
static uint32_t s_buzzer_duration;
static GPIO_PinState s_raw_key;
static GPIO_PinState s_stable_key;
static uint32_t s_raw_change_ms;
static uint32_t s_press_ms;
static uint32_t s_release_ms;
static uint8_t s_pending_click;
static uint8_t s_consumed_press;
static uint32_t s_arm_accepted, s_arm_stop_events;
static float s_debug_left = ACK_DEBUG_DEFAULT_SPEED_MPS;
static float s_debug_right = ACK_DEBUG_DEFAULT_SPEED_MPS;
static float s_debug_angle = ACK_DEBUG_DEFAULT_ANGLE_DEG;
static TrackingController s_tracking;
static TrackingOutput s_tracking_output;

static void StartBuzzer(uint32_t duration)
{
    s_buzzer_start = HAL_GetTick();
    s_buzzer_duration = duration;
    HAL_GPIO_WritePin(MODE_BUZZER_GPIO_PORT, MODE_BUZZER_PIN,
                       duration ? MODE_BUZZER_ACTIVE_LEVEL : MODE_BUZZER_INACTIVE_LEVEL);
}

static void StopTask(ModeControl_Status_t status)
{
    Tracking_Reset(&s_tracking, &s_tracking_output);
    Flag_Stop = 1U;
    s_status = status;
    Drive_Motor(0.0f, 0.0f, 0.0f);
    SpeedPI_Reset();
    Ack_YawReset(&s_yaw_pid, 0.0f);
    Set_Pwm(0, 0, 0, 0, SERVO_INIT);
}

static void StartTask(void)
{
    float yaw;
    if (EN == 0 || Voltage < ACK_MIN_BATTERY_V || Check != 0) {
        StopTask(MODE_STATUS_POWER_ERROR);
        StartBuzzer(ERROR_BEEP_MS);
        return;
    }
    if (s_mode != MODE_CONTROL_VISION_LINE && s_mode != MODE_CONTROL_DEBUG && !Gyro6_IsValid()) {
        StopTask(MODE_STATUS_IMU_ERROR);
        StartBuzzer(ERROR_BEEP_MS);
        return;
    }
    if (s_mode == MODE_CONTROL_VISION_LINE) {
        VisionCommand cmd;
        VisionTarget target;
        uint32_t mask = __get_PRIMASK();
        __disable_irq();
        Vision_Get(&cmd);
        Vision_GetControlTarget(&target);
        /* Require a V3/V4 target received after local arming. */
        s_arm_accepted = target.accepted;
        s_arm_stop_events = cmd.stop_events;
        __set_PRIMASK(mask);
        Tracking_Reset(&s_tracking, &s_tracking_output);
        s_start_ms = HAL_GetTick();
        s_elapsed_ms = 0U;
        SpeedPI_Reset();
        Drive_Motor(0.0f, 0.0f, 0.0f);
        StartBuzzer(0U);
        Flag_Stop = 0U;
        s_status = MODE_STATUS_WAIT_COMMAND;
        return;
    }
    /* Capture the current heading; do not reset/calibrate the IMU. */
    yaw = Gyro6_GetYaw();
    s_target_yaw = yaw;
    if (s_mode == MODE_CONTROL_TURN_90)
        s_target_yaw = Ack_AngleError(yaw + ACK_TASK2_YAW_DIRECTION * ACK_TASK2_TURN_DEG, 0.0f);
    s_last_yaw = yaw;
    s_turn_progress = 0.0f;
    s_elapsed_ms = 0U;
    s_start_ms = HAL_GetTick();
    SpeedPI_Reset();
    Ack_YawReset(&s_yaw_pid, Ack_AngleError(s_target_yaw, yaw));
    StartBuzzer(0U);
    Flag_Stop = 0U;
    s_status = MODE_STATUS_RUNNING;
}

static KeyEvent ScanKey(uint32_t now)
{
    GPIO_PinState raw = HAL_GPIO_ReadPin(MODE_USER_KEY_GPIO_PORT, MODE_USER_KEY_PIN);
    KeyEvent event = KEY_NONE;
    if (raw != s_raw_key) {
        s_raw_key = raw;
        s_raw_change_ms = now;
    }
    if (s_stable_key != s_raw_key && (uint32_t)(now - s_raw_change_ms) >= KEY_DEBOUNCE_MS) {
        s_stable_key = s_raw_key;
        if (s_stable_key == MODE_USER_KEY_ACTIVE_LEVEL) {
            uint32_t gap = s_raw_change_ms - s_release_ms;
            s_press_ms = now;
            s_consumed_press = 0U;
            if (s_pending_click && gap >= KEY_DOUBLE_MIN_MS && gap <= KEY_DOUBLE_MAX_MS) {
                s_pending_click = 0U;
                s_consumed_press = 1U;
                event = KEY_DOUBLE;
            } else if (s_pending_click && gap > KEY_DOUBLE_MAX_MS) {
                s_pending_click = 0U;
                event = KEY_SINGLE;
            } else if (s_pending_click) {
                /* Too close to count as a second click: ignore this press. */
                s_consumed_press = 1U;
            }
        } else if (!s_consumed_press) {
            s_pending_click = 1U;
            s_release_ms = now;
        }
    }
    if (s_stable_key == MODE_USER_KEY_ACTIVE_LEVEL && !s_consumed_press &&
        (uint32_t)(now - s_press_ms) >= KEY_LONG_MS) {
        s_consumed_press = 1U;
        s_pending_click = 0U;
        return KEY_LONG;
    }
    if (s_pending_click && s_stable_key != MODE_USER_KEY_ACTIVE_LEVEL &&
        s_raw_key != MODE_USER_KEY_ACTIVE_LEVEL &&
        (uint32_t)(now - s_release_ms) > KEY_DOUBLE_MAX_MS) {
        s_pending_click = 0U;
        event = KEY_SINGLE;
    }
    return event;
}

void ModeControl_ButtonTask(void)
{
    KeyEvent event = ScanKey(HAL_GetTick());
    if (event == KEY_SINGLE) {
        if (ModeControl_IsActive()) StopTask(MODE_STATUS_STOPPED);
        else StartTask();
    } else if (event == KEY_DOUBLE) {
        StopTask(MODE_STATUS_READY);
        s_mode = s_mode == MODE_CONTROL_STRAIGHT_5S ? MODE_CONTROL_TURN_90 :
                 s_mode == MODE_CONTROL_TURN_90 ? MODE_CONTROL_VISION_LINE :
                 s_mode == MODE_CONTROL_VISION_LINE ? MODE_CONTROL_DEBUG :
                 MODE_CONTROL_STRAIGHT_5S;
        s_elapsed_ms = 0U;
        StartBuzzer(SWITCH_BEEP_MS);
    } else if (event == KEY_LONG) {
        StopTask(MODE_STATUS_STOPPED);
    }
}

void ModeControl_Init(void)
{
    s_mode = MODE_CONTROL_STRAIGHT_5S;
    s_debug_left = s_debug_right = ACK_DEBUG_DEFAULT_SPEED_MPS;
    s_debug_angle = ACK_DEBUG_DEFAULT_ANGLE_DEG;
    s_elapsed_ms = 0U;
    s_target_yaw = 0.0f;
    MOTOR_A.Velocity_KP = ACK_SPEED_A_KP;
    MOTOR_A.Velocity_KI = ACK_SPEED_A_KI;
    MOTOR_B.Velocity_KP = ACK_SPEED_B_KP;
    MOTOR_B.Velocity_KI = ACK_SPEED_B_KI;
    StopTask(MODE_STATUS_READY);
    StartBuzzer(0U);
    s_raw_key = HAL_GPIO_ReadPin(MODE_USER_KEY_GPIO_PORT, MODE_USER_KEY_PIN);
    s_stable_key = s_raw_key;
    s_raw_change_ms = HAL_GetTick();
    s_press_ms = s_raw_change_ms;
    s_release_ms = s_raw_change_ms;
    s_pending_click = 0U;
    /* A key held at power-on must be released before it can start a task. */
    s_consumed_press = s_raw_key == MODE_USER_KEY_ACTIVE_LEVEL ? 1U : 0U;
}

void ModeControl_Tick10ms(void)
{
    uint32_t now = HAL_GetTick();
    float yaw, steer, speed;
    Distance_Tick();
    /* Freeze display time at stop/finish, including manual interruption. */
    if (ModeControl_IsActive()) s_elapsed_ms = now - s_start_ms;
    ModeControl_ButtonTask();
    if (s_buzzer_duration && (uint32_t)(now - s_buzzer_start) >= s_buzzer_duration)
        StartBuzzer(0U);
    if (!ModeControl_IsActive()) return;
    if (Turn_Off(Voltage)) {
        StopTask(MODE_STATUS_POWER_ERROR);
        StartBuzzer(ERROR_BEEP_MS);
        return;
    }
    if (s_mode != MODE_CONTROL_VISION_LINE && s_mode != MODE_CONTROL_DEBUG && !Gyro6_IsValid()) {
        StopTask(MODE_STATUS_IMU_ERROR);
        StartBuzzer(ERROR_BEEP_MS);
        return;
    }
    if (s_mode == MODE_CONTROL_DEBUG) {
        /* Independent bench targets: no yaw PID or kinematic redistribution. */
        MOTOR_A.Target = Ack_Clamp(s_debug_left, -ACK_MAX_WHEEL_SPEED_MPS, ACK_MAX_WHEEL_SPEED_MPS);
        MOTOR_B.Target = Ack_Clamp(s_debug_right, -ACK_MAX_WHEEL_SPEED_MPS, ACK_MAX_WHEEL_SPEED_MPS);
        MOTOR_C.Target = MOTOR_D.Target = 0.0f;
        Servo = Ack_DebugServoPulse(s_debug_angle);
        return;
    }
    yaw = Gyro6_GetYaw();
    if (s_mode == MODE_CONTROL_VISION_LINE) {
        VisionCommand cmd;
        VisionTarget target, latest_route;
        TrackingParameters params;
        uint32_t mask = __get_PRIMASK();
        __disable_irq();
        Vision_Get(&cmd);
        Vision_GetControlTarget(&target);
        Vision_GetTarget(&latest_route);
        Tracking_GetParameters(&params);
        __set_PRIMASK(mask);
        if (cmd.stop_events != s_arm_stop_events) {
            StopTask(MODE_STATUS_REMOTE_STOP);
            return;
        }
        if (s_status == MODE_STATUS_WAIT_COMMAND && target.accepted == s_arm_accepted) {
            if ((uint32_t)(now - s_start_ms) > VISION_ARM_WAIT_MS)
                StopTask(MODE_STATUS_LINK_ERROR);
            return;
        }
        if ((!target.valid || !target.link_alive) &&
            s_tracking.tape_state != TAPE_WAIT && s_tracking.tape_state != TAPE_FAULT &&
            s_tracking.tape_state != TAPE_RECOVER && s_tracking.tape_state != TAPE_FINISH &&
            !(latest_route.version == 5U && latest_route.link_alive && latest_route.tape_detected &&
              s_tracking.tape_state != TAPE_PASS &&
              (uint32_t)(now-latest_route.received_ms)<TRACK_TAPE_GAP_MAX_MS)) {
            StopTask(MODE_STATUS_LINK_ERROR);
            StartBuzzer(ERROR_BEEP_MS);
            return;
        }
        if (!Tracking_UpdateRoute(&s_tracking, &target, &latest_route, now,
                0.5f*(MOTOR_A.Encoder+MOTOR_B.Encoder), &params, &s_tracking_output)) {
            StopTask(MODE_STATUS_CONTROL_ERROR);
            StartBuzzer(ERROR_BEEP_MS);
            return;
        }
        s_status = MODE_STATUS_RUNNING;
        /* Drive_Motor -> Ack_Kinematics + Ack_ServoPulse; balance.c keeps
         * the positional wheel-speed PI and PWM protection. */
        Drive_Motor(s_tracking_output.speed_mps, 0.0f, s_tracking_output.steer_left_rad);
        return;
    }
    if (s_mode == MODE_CONTROL_STRAIGHT_5S) {
        if (s_elapsed_ms >= ACK_TASK1_DURATION_MS) {
            StopTask(MODE_STATUS_DONE);
            StartBuzzer(FINISH_BEEP_MS);
            return;
        }
        speed = ACK_TASK1_SPEED_MPS;
    } else {
        /* Unwrap incremental yaw so +179 -> -179 is a +2 degree change. */
        s_turn_progress += Ack_AngleError(yaw, s_last_yaw);
        s_last_yaw = yaw;
        if (ACK_TASK2_YAW_DIRECTION * s_turn_progress >=
            ACK_TASK2_TURN_DEG - ACK_TASK2_TOLERANCE_DEG) {
            StopTask(MODE_STATUS_DONE);
            StartBuzzer(FINISH_BEEP_MS);
            return;
        }
        if (s_elapsed_ms >= ACK_TASK2_TIMEOUT_MS) {
            StopTask(MODE_STATUS_TIMEOUT);
            StartBuzzer(ERROR_BEEP_MS);
            return;
        }
        speed = ACK_TASK2_SPEED_MPS;
    }
    steer = Ack_YawUpdate(&s_yaw_pid, s_target_yaw, yaw);
    Drive_Motor(speed, 0.0f, steer);
}

void ModeControl_GetTrackingOutput(TrackingOutput *out)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    *out = s_tracking_output;
    __set_PRIMASK(mask);
}

uint8_t ModeControl_IsActive(void) {
    return (s_status == MODE_STATUS_RUNNING || s_status == MODE_STATUS_WAIT_COMMAND) ? 1U : 0U;
}
ModeControl_Mode_t ModeControl_GetDisplayMode(void) { return s_mode; }
ModeControl_Status_t ModeControl_GetStatus(void) { return s_status; }
uint32_t ModeControl_GetElapsedMs(void) { return s_elapsed_ms; }
float ModeControl_GetTargetYaw(void) { return s_target_yaw; }

float ModeControl_DebugLeft(void) { return s_debug_left; }
float ModeControl_DebugRight(void) { return s_debug_right; }
float ModeControl_DebugAngle(void) { return s_debug_angle; }

uint8_t ModeControl_DebugCommand(const char *key, float value)
{
    if (!strcmp(key,"TRESET")) {
        if (value != 1.0f || (ModeControl_IsActive() && s_tracking.tape_state >= TAPE_APPROACH)) return 0;
        Tracking_DefaultParameters();
        Tracking_Reset(&s_tracking, &s_tracking_output);
        return 1;
    }
    if (!strcmp(key,"TKP") || !strcmp(key,"TKD") ||
        !strcmp(key,"TKS") || !strcmp(key,"TDF")) return 0;
    if (key[0] == 'T') {
        uint8_t ok = Tracking_SetParameter(key,value);
        if (ok && !strcmp(key,"TDF")) Tracking_Reset(&s_tracking,&s_tracking_output);
        return ok;
    }
    /* Range comparisons reject NaN and infinity too. */
    if (!strcmp(key, "M")) {
        if (!(value == 1.0f || value == 2.0f || value == 3.0f || value == 4.0f)) return 0;
        StopTask(MODE_STATUS_READY);
        s_mode = (ModeControl_Mode_t)(int)value;
        s_elapsed_ms = 0;
        return 1;
    }
    if (!strcmp(key, "RUN")) {
        if (value == 0.0f) { StopTask(MODE_STATUS_STOPPED); return 1; }
        if (value != 1.0f || (s_mode != MODE_CONTROL_DEBUG && s_mode != MODE_CONTROL_VISION_LINE)) return 0;
        if (!ModeControl_IsActive()) StartTask();
        return ModeControl_IsActive();
    }
    /* Wheel PI tuning is restricted to bench mode; gains remain active in all modes. */
    if (!strcmp(key,"LKP") || !strcmp(key,"LKI") ||
        !strcmp(key,"RKP") || !strcmp(key,"RKI")) {
        float max_gain = key[2]=='P' ? ACK_SPEED_KP_MAX : ACK_SPEED_KI_MAX;
        if (s_mode != MODE_CONTROL_DEBUG || !(value >= 0.0f && value <= max_gain)) return 0;
        if (!strcmp(key,"LKP")) MOTOR_A.Velocity_KP=value;
        else if (!strcmp(key,"LKI")) MOTOR_A.Velocity_KI=value;
        else if (!strcmp(key,"RKP")) MOTOR_B.Velocity_KP=value;
        else MOTOR_B.Velocity_KI=value;
        /* Match DIANSAI tuning: clear this loop's stored integral after a gain change. */
        SpeedPI_ResetWheel(key[0]=='R');
        return 1;
    }
    if (!strcmp(key, "A")) {
        if (!(value >= ACK_DEBUG_MIN_ANGLE_DEG && value <= ACK_DEBUG_MAX_ANGLE_DEG)) return 0;
        s_debug_angle = value;
        return 1;
    }
    if (!strcmp(key, "L") || !strcmp(key, "R") || !strcmp(key, "V")) {
        if (!(value >= -ACK_MAX_WHEEL_SPEED_MPS && value <= ACK_MAX_WHEEL_SPEED_MPS)) return 0;
        if (key[0] != 'R') s_debug_left = value;
        if (key[0] != 'L') s_debug_right = value;
        return 1;
    }
    return 0;
}
