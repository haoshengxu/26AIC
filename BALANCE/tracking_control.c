#include "tracking_control.h"
#include "ackermann_control.h"
#include "stm32f4xx_hal.h"
#include <float.h>
#include <math.h>
#include <string.h>

static volatile TrackingInput tracking_input;
static volatile float tracking_path_yaw;
static volatile uint32_t tracking_path_ms;
static volatile uint8_t tracking_path_valid;
static TrackingParameters parameters = {TRACK_BASE_SPEED_MPS, TRACK_KP,
    TRACK_KD, TRACK_STANLEY_K, TRACK_D_FILTER_TAU_S,
    TRACK_TURN_X_THRESHOLD_CM, TRACK_TURN_SPEED_GAIN, TRACK_TURN_MIN_SPEED_MPS};

void Tracking_DefaultParameters(void)
{
    parameters.speed_mps = TRACK_BASE_SPEED_MPS;
    parameters.kp = TRACK_KP;
    parameters.kd = TRACK_KD;
    parameters.stanley_k = TRACK_STANLEY_K;
    parameters.filter_tau_s = TRACK_D_FILTER_TAU_S;
    parameters.turn_x_threshold_cm = TRACK_TURN_X_THRESHOLD_CM;
    parameters.turn_speed_gain = TRACK_TURN_SPEED_GAIN;
    parameters.turn_min_speed_mps = TRACK_TURN_MIN_SPEED_MPS;
}

void Tracking_GetParameters(TrackingParameters *out)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    *out = parameters;
    __set_PRIMASK(mask);
}

uint8_t Tracking_SetParameter(const char *key, float value)
{
    if (!strcmp(key,"TV") && value >= 0.01f && value <= ACK_MAX_WHEEL_SPEED_MPS)
        parameters.speed_mps = value;
    else if (!strcmp(key,"TXTH") && value >= 0.0f && value <= 327.68f)
        parameters.turn_x_threshold_cm = value;
    else if (!strcmp(key,"TXK") && value >= 0.0f && value <= 1.0f)
        parameters.turn_speed_gain = value;
    else if (!strcmp(key,"TVMIN") && value >= 0.01f && value <= ACK_MAX_WHEEL_SPEED_MPS)
        parameters.turn_min_speed_mps = value;
    else if (!strcmp(key,"TKP") && value >= 0.0f && value <= 10.0f)
        parameters.kp = value;
    else if (!strcmp(key,"TKD") && value >= 0.0f && value <= 2.0f)
        parameters.kd = value;
    else if (!strcmp(key,"TKS") && value >= 0.0f && value <= 10.0f)
        parameters.stanley_k = value;
    else if (!strcmp(key,"TDF") && value >= 0.0f && value <= 1.0f)
        parameters.filter_tau_s = value;
    else return 0U;
    return 1U;
}

float Tracking_TurnSpeed(float x_right_m, const TrackingParameters *p)
{
    float excess_cm = fabsf(x_right_m)*100.0f-p->turn_x_threshold_cm;
    float floor = p->turn_min_speed_mps < p->speed_mps ? p->turn_min_speed_mps : p->speed_mps;
    if (excess_cm <= 0.0f) return p->speed_mps;
    return Ack_Clamp(p->speed_mps-p->turn_speed_gain*excess_cm, floor, p->speed_mps);
}

static uint8_t Tracking_Finite(float value)
{
    return value == value && value <= FLT_MAX && value >= -FLT_MAX;
}

void Tracking_InputInit(void)
{
    Tracking_DefaultParameters();
    memset((void *)&tracking_input, 0, sizeof(tracking_input));
    tracking_path_valid = 0U;
}

void Tracking_OnFrame(int16_t raw, uint16_t seq, uint8_t valid, uint32_t now)
{
    uint16_t delta;
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (!valid) {
        /* A line-loss report is not a new usable measurement. Keep the last
         * valid sample and its timestamp; continued loss cannot renew it. */
        __set_PRIMASK(mask);
        return;
    }
    delta = (uint16_t)(seq - tracking_input.sequence);
    if (tracking_input.valid && (uint32_t)(now - tracking_input.received_ms) < TRACK_TIMEOUT_MS &&
        (delta == 0U || delta >= 0x8000U)) {
        ++tracking_input.sequence_errors;
        __set_PRIMASK(mask);
        return; /* Replayed/reordered values cannot alter steering or refresh motion. */
    }
    tracking_input.distance_raw = raw;
    tracking_input.sequence = seq;
    tracking_input.received_ms = now;
    tracking_input.valid = 1U;
    ++tracking_input.frames;
    __set_PRIMASK(mask);
}

void Tracking_UartError(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    ++tracking_input.stop_events;
    tracking_input.valid = 0U;
    tracking_input.distance_raw = 0;
    __set_PRIMASK(mask);
}

void Tracking_GetInput(TrackingInput *out)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    *out = tracking_input;
    __set_PRIMASK(mask);
}

uint8_t Tracking_CanRun(const TrackingInput *in, uint32_t now)
{
    return in->valid && (uint32_t)(now - in->received_ms) < TRACK_TIMEOUT_MS;
}

void Tracking_Reset(TrackingController *controller, TrackingOutput *out)
{
    memset(controller, 0, sizeof(*controller));
    memset(out, 0, sizeof(*out));
}

void Tracking_SetHeadingReference(float target_yaw_deg, uint8_t valid)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    /* A reference is an angle in the IMU's [-180,180] coordinate system. */
    tracking_path_valid = valid && Tracking_Finite(target_yaw_deg) &&
                          target_yaw_deg >= -180.0f && target_yaw_deg <= 180.0f;
    tracking_path_yaw = target_yaw_deg;
    tracking_path_ms = HAL_GetTick();
    __set_PRIMASK(mask);
}

static float Tracking_Heading(uint8_t imu_valid, float yaw_deg, uint8_t *used)
{
    float target;
    uint8_t valid;
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    target = tracking_path_yaw;
    valid = tracking_path_valid &&
        (uint32_t)(HAL_GetTick() - tracking_path_ms) <= TRACK_HEADING_MAX_AGE_MS;
    __set_PRIMASK(mask);
    *used = valid && imu_valid && Tracking_Finite(yaw_deg) &&
            yaw_deg >= -180.0f && yaw_deg <= 180.0f;
    if (!*used) return 0.0f;
    /* IMU-to-left turn convention -> right-positive formula correction. */
    return -ACK_YAW_TO_STEER_SIGN * TRACK_HEADING_GAIN *
           Ack_AngleError(target, yaw_deg) * 0.01745329252f;
}

uint8_t Tracking_Update(TrackingController *controller, const TrackingInput *in,
                        uint32_t now, float measured_speed, uint8_t imu_valid,
                        float yaw_deg, TrackingOutput *out)
{
    float error, dt, slope, alpha, correction;
    uint32_t elapsed;
    memset(out, 0, sizeof(*out));
    if (!Tracking_CanRun(in, now) || !Tracking_Finite(measured_speed) ||
        !(parameters.speed_mps > 0.0f && parameters.speed_mps <= ACK_MAX_WHEEL_SPEED_MPS) ||
        !(TRACK_MIN_SPEED_MPS > 0.0f) || !(parameters.filter_tau_s >= 0.0f) ||
        !(TRACK_D_LIMIT_MPS > 0.0f) ||
        !(TRACK_STEER_LIMIT_RAD > 0.0f && TRACK_STEER_LIMIT_RAD <= ACK_MAX_STEER_RAD))
        return 0U;
    error = TRACK_ERROR_SIGN * (float)in->distance_raw * 0.0001f;
    if (!controller->has_sample) {
        controller->derivative_mps = 0.0f;
        controller->has_sample = 1U;
    } else if (in->frames != controller->last_frames) {
        elapsed = (uint32_t)(in->received_ms - controller->last_sample_ms);
        if (elapsed > TRACK_TIMEOUT_MS) controller->derivative_mps = 0.0f;
        else if (elapsed > 0U) {
            dt = elapsed * 0.001f;
            slope = Ack_Clamp((error - controller->last_error_m) / dt,
                              -TRACK_D_LIMIT_MPS, TRACK_D_LIMIT_MPS);
            alpha = dt / (parameters.filter_tau_s + dt);
            controller->derivative_mps += alpha * (slope - controller->derivative_mps);
        }
    }
    if (in->frames != controller->last_frames || controller->last_frames == 0U) {
        controller->last_sample_ms = in->received_ms;
        controller->last_error_m = error;
        controller->last_frames = in->frames;
    }
    out->error_m = error;
    out->derivative_mps = controller->derivative_mps;
    out->measured_speed_mps = measured_speed;
    out->denominator_mps = fmaxf(fabsf(measured_speed), TRACK_MIN_SPEED_MPS);
    out->p_rad = parameters.kp * error;
    out->d_rad = parameters.kd * out->derivative_mps;
    out->stanley_rad = atanf(parameters.stanley_k * error / out->denominator_mps);
    out->heading_rad = Tracking_Heading(imu_valid, yaw_deg, &out->heading_used);
    correction = out->p_rad + out->d_rad + out->stanley_rad + out->heading_rad;
    if (!Tracking_Finite(correction)) return 0U;
    out->correction_right_rad = Ack_Clamp(correction, -TRACK_STEER_LIMIT_RAD, TRACK_STEER_LIMIT_RAD);
    out->steer_left_rad = TRACK_TO_CHASSIS_SIGN * out->correction_right_rad;
    out->speed_mps = parameters.speed_mps;
    out->valid = 1U;
    return 1U;
}
