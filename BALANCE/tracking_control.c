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
    TRACK_TURN_X_THRESHOLD_CM, TRACK_TURN_SPEED_GAIN, TRACK_TURN_MIN_SPEED_MPS,
    TRACK_PP_GAIN, TRACK_CURVE_THRESHOLD, TRACK_CURVE_SPEED_GAIN,
    TRACK_SINGLE_SPEED_MPS, TRACK_FALLBACK_SPEED_MPS, TRACK_UNKNOWN_SPEED_MPS,
    TRACK_HOLD_SPEED_MPS, TRACK_SAT_SPEED_MPS, TRACK_ACCEL_MPS2, TRACK_DECEL_MPS2,
    TRACK_PP_STRAIGHT_GAIN, TRACK_GAIN_CURVE_ENTER, TRACK_GAIN_CURVE_EXIT};

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
    parameters.pp_gain = TRACK_PP_GAIN;
    parameters.pp_straight_gain = TRACK_PP_STRAIGHT_GAIN;
    parameters.gain_curve_enter = TRACK_GAIN_CURVE_ENTER;
    parameters.gain_curve_exit = TRACK_GAIN_CURVE_EXIT;
    parameters.curve_threshold = TRACK_CURVE_THRESHOLD;
    parameters.curve_speed_gain = TRACK_CURVE_SPEED_GAIN;
    parameters.single_speed_mps = TRACK_SINGLE_SPEED_MPS;
    parameters.fallback_speed_mps = TRACK_FALLBACK_SPEED_MPS;
    parameters.unknown_speed_mps = TRACK_UNKNOWN_SPEED_MPS;
    parameters.hold_speed_mps = TRACK_HOLD_SPEED_MPS;
    parameters.saturated_speed_mps = TRACK_SAT_SPEED_MPS;
    parameters.accel_mps2 = TRACK_ACCEL_MPS2;
    parameters.decel_mps2 = TRACK_DECEL_MPS2;
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
    else if (!strcmp(key,"TPG") && value >= 0.5f && value <= 1.5f)
        { parameters.pp_gain = value; parameters.pp_straight_gain = value; }
    else if (!strcmp(key,"TPGC") && value >= 0.5f && value <= 1.5f)
        parameters.pp_gain = value;
    else if (!strcmp(key,"TPGS") && value >= 0.5f && value <= 1.5f)
        parameters.pp_straight_gain = value;
    else if (!strcmp(key,"TGEN") && value > parameters.gain_curve_exit && value <= 20.0f)
        parameters.gain_curve_enter = value;
    else if (!strcmp(key,"TGEX") && value >= 0.0f && value < parameters.gain_curve_enter)
        parameters.gain_curve_exit = value;
    else if (!strcmp(key,"TCTH") && value >= 0.0f && value <= 20.0f)
        parameters.curve_threshold = value;
    else if (!strcmp(key,"TCK") && value >= 0.0f && value <= 1.0f)
        parameters.curve_speed_gain = value;
    else if (!strcmp(key,"TSINGLE") && value >= 0.01f && value <= ACK_MAX_WHEEL_SPEED_MPS)
        parameters.single_speed_mps = value;
    else if (!strcmp(key,"TFALL") && value >= 0.01f && value <= ACK_MAX_WHEEL_SPEED_MPS)
        parameters.fallback_speed_mps = value;
    else if (!strcmp(key,"TUNKNOWN") && value >= 0.01f && value <= ACK_MAX_WHEEL_SPEED_MPS)
        parameters.unknown_speed_mps = value;
    else if (!strcmp(key,"THOLD") && value >= 0.01f && value <= ACK_MAX_WHEEL_SPEED_MPS)
        parameters.hold_speed_mps = value;
    else if (!strcmp(key,"TSAT") && value >= 0.01f && value <= ACK_MAX_WHEEL_SPEED_MPS)
        parameters.saturated_speed_mps = value;
    else if (!strcmp(key,"TACC") && value >= 0.05f && value <= 3.0f)
        parameters.accel_mps2 = value;
    else if (!strcmp(key,"TDEC") && value >= 0.05f && value <= 5.0f)
        parameters.decel_mps2 = value;
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

uint8_t Tracking_UpdateRoute(TrackingController *controller,
    const VisionTarget *control, const VisionTarget *latest, uint32_t now,
    float measured_speed, const TrackingParameters *p, TrackingOutput *out)
{
    float x, y, d2, curvature, excess, floor, requested, step;
    uint8_t fresh, v4, curve_known;
    float gain;
    memset(out, 0, sizeof(*out));
    if (!control->valid || !control->link_alive ||
        (uint32_t)(now-control->received_ms) >= VISION_TARGET_HOLD_MS ||
        !Tracking_Finite(measured_speed)) return 0U;
    x = control->x_centi_cm*0.0001f;
    y = control->y_centi_cm*0.0001f;
    if (!Ack_PurePursuitGain(x,y,p->pp_gain,&out->steer_left_rad)) return 0U;
    d2 = x*x+y*y;
    curvature = -2.0f*x/d2; /* left positive; geometry, before gain */
    fresh = latest->link_alive && latest->valid &&
        (uint32_t)(now-latest->received_ms) < TRACK_ROUTE_FRESH_MS &&
        (uint32_t)(now-control->received_ms) < TRACK_ROUTE_FRESH_MS;
    v4 = control->version == 4U;
    curve_known = v4 && latest->version == 4U && latest->link_alive &&
        (uint32_t)(now-latest->received_ms) < TRACK_ROUTE_FRESH_MS &&
        latest->endpoint_valid && latest->endpoint_y_centi_cm > 0 && latest->curvature_valid;
    if (!controller->gain_initialized) {
        controller->active_pp_gain = p->pp_gain;
        controller->gain_curve = 1U; /* Unknown startup uses the calibrated curve gain. */
        controller->gain_initialized = 1U;
    }
    gain = p->pp_gain;
    out->gain_state = 4U;
    if (v4) {
        out->gain_state = !fresh ? 3U : !curve_known ? 2U : controller->gain_curve;
        if (fresh && curve_known) {
            float road = fminf(latest->endpoint_abs_curvature_cm_inv,1.0f)*100.0f;
            /* Road curvature, not gain-scaled pursuit error: avoids feedback switching. */
            if (road >= p->gain_curve_enter) controller->gain_curve = 1U;
            else if (road <= p->gain_curve_exit) controller->gain_curve = 0U;
            out->gain_state = controller->gain_curve;
            gain = controller->gain_curve ? p->pp_gain : p->pp_straight_gain;
            controller->active_pp_gain += Ack_Clamp(gain-controller->active_pp_gain,
                -TRACK_GAIN_RATE_PER_S*ACK_CONTROL_DT_S, TRACK_GAIN_RATE_PER_S*ACK_CONTROL_DT_S);
        }
        gain = controller->active_pp_gain;
    } else controller->active_pp_gain = gain;
    if (!Ack_PurePursuitGain(x,y,gain,&out->steer_left_rad)) return 0U;
    out->active_pp_gain = gain;
    out->raw_steer_left_rad = atanf(ACK_WHEELBASE_M*gain*curvature);
    out->control_x_m = out->error_m = x;
    out->control_y_m = y;
    out->correction_right_rad = -out->steer_left_rad;
    out->measured_speed_mps = measured_speed;
    out->speed_curvature_m_inv = fabsf(gain*curvature);
    out->route_state = TRACK_ROUTE_NORMAL;
    if (v4) {
        /* Endpoint absolute curvature anticipates speed and selects gain. It has neither
         * direction nor current axle curvature, so never adds steering. Bound
         * before unit conversion to keep even extreme finite wire values safe. */
        if (curve_known) {
            float road = fminf(latest->endpoint_abs_curvature_cm_inv,1.0f)*100.0f;
            out->speed_curvature_m_inv = fmaxf(out->speed_curvature_m_inv,road);
        }
        floor = fminf(p->turn_min_speed_mps,p->speed_mps);
        excess = fmaxf(out->speed_curvature_m_inv-p->curve_threshold,0.0f);
        requested = Ack_Clamp(p->speed_mps-p->curve_speed_gain*excess,floor,p->speed_mps);
        if (control->mode == 2U || control->mode == 3U) {
            out->route_reasons |= TRACK_REASON_SINGLE;
            out->route_state = TRACK_ROUTE_SINGLE;
            requested = fminf(requested,p->single_speed_mps);
        }
        if (!curve_known) {
            out->route_reasons |= TRACK_REASON_UNKNOWN;
            out->route_state = TRACK_ROUTE_UNKNOWN;
            requested = fminf(requested,p->unknown_speed_mps);
        }
        if (control->fallback) {
            out->route_reasons |= TRACK_REASON_FALLBACK;
            out->route_state = TRACK_ROUTE_FALLBACK;
            requested = fminf(requested,p->fallback_speed_mps);
        }
        if (fabsf(out->raw_steer_left_rad) >= ACK_MAX_STEER_RAD) {
            out->route_reasons |= TRACK_REASON_SATURATED;
            out->route_state = TRACK_ROUTE_SATURATED;
            requested = fminf(requested,p->saturated_speed_mps);
        }
        if (!fresh) {
            out->route_reasons |= TRACK_REASON_HOLD;
            out->route_state = TRACK_ROUTE_HOLD;
            requested = fminf(requested,p->hold_speed_mps);
            /* Never accelerate on stale/invalid vision, even during startup. */
            requested = fminf(requested,controller->route_speed_mps);
        }
        out->requested_speed_mps = requested;
        step = (requested > controller->route_speed_mps ? p->accel_mps2 : p->decel_mps2)*ACK_CONTROL_DT_S;
        controller->route_speed_mps += Ack_Clamp(requested-controller->route_speed_mps,-step,step);
        out->speed_mps = Ack_Clamp(controller->route_speed_mps,0.0f,p->speed_mps);
        controller->route_speed_mps = out->speed_mps;
    } else {
        /* Backward-compatible V3 tuning, including existing TXTH/TXK commands. */
        out->requested_speed_mps = out->speed_mps = Tracking_TurnSpeed(x,p);
        controller->route_speed_mps = out->speed_mps;
        if (!fresh) {
            out->route_reasons |= TRACK_REASON_HOLD;
            out->route_state = TRACK_ROUTE_HOLD;
        }
    }
    out->valid = 1U;
    return 1U;
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
