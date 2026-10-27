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
    TRACK_PP_STRAIGHT_GAIN, TRACK_GAIN_CURVE_ENTER, TRACK_GAIN_CURVE_EXIT,
    TRACK_START_ACCEL_MPS2, TRACK_TAPE_DECEL_MPS2, TRACK_TAPE_GAP_CM, TRACK_TAPE_WAIT_S,
    10.0f, TRACK_TAPE_RECOVER_SPEED, TRACK_TAPE_RECOVER_MS*0.001f, TRACK_TAPE_FINISH_GAP_CM, TRACK_TAPE_FINISH_GAP_M5_CM, TRACK_FINISH_APPROACH_SPEED, 0.0f, 3.0f, TRACK_FINISH_BLIND_SPEED_MPS, TRACK_FINISH_BLIND_TIME_S,
    TRACK_TAPE1_STRAIGHT_S, TRACK_TAPE1_RIGHT_DEG, TRACK_TAPE1_SPEED_MPS, TRACK_TAPE1_MIN_S, TRACK_TAPE1_MAX_S,
    TRACK_TAPE3_STRAIGHT_S, TRACK_TAPE3_RIGHT_DEG, TRACK_TAPE3_SPEED_MPS, TRACK_TAPE3_MIN_S, TRACK_TAPE3_MAX_S,
    TRACK_TAPE1_STRAIGHT_CM, TRACK_TAPE3_STRAIGHT_CM,
    TRACK_EXIT_SLOW_DISTANCE_CM, TRACK_EXIT_SLOW_SPEED_MPS,
    TRACK_EXIT_SLOW_END_CM, TRACK_EXIT_SLOW3_START_CM, TRACK_EXIT_SLOW3_END_CM,
    TRACK_FINISH_SLOW_START_CM};

void Tracking_DefaultParameters(void)
{
    parameters.local_heading_gain=0.0f;
    parameters.local_heading_limit_deg=3.0f;
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
    parameters.start_accel_mps2 = TRACK_START_ACCEL_MPS2;
    parameters.decel_mps2 = TRACK_DECEL_MPS2;
    parameters.tape_decel_mps2 = TRACK_TAPE_DECEL_MPS2;
    parameters.tape_gap_cm = TRACK_TAPE_GAP_CM;
    parameters.tape_finish_gap_cm=TRACK_TAPE_FINISH_GAP_CM;
    parameters.tape_finish_gap_m5_cm=TRACK_TAPE_FINISH_GAP_M5_CM;
    parameters.finish_approach_speed_mps=TRACK_FINISH_APPROACH_SPEED;
    parameters.finish_blind_speed_mps=TRACK_FINISH_BLIND_SPEED_MPS;
    parameters.finish_blind_time_s=TRACK_FINISH_BLIND_TIME_S;
    parameters.tape_wait_s = TRACK_TAPE_WAIT_S;
    parameters.tape_recover_angle_deg=10.0f;
    parameters.tape_recover_speed_mps=TRACK_TAPE_RECOVER_SPEED;
    parameters.tape_recover_time_s=TRACK_TAPE_RECOVER_MS*0.001f;
    parameters.tape1_straight_s = TRACK_TAPE1_STRAIGHT_S;
    parameters.tape1_right_deg = TRACK_TAPE1_RIGHT_DEG;
    parameters.tape1_speed_mps = TRACK_TAPE1_SPEED_MPS;
    parameters.tape1_min_s = TRACK_TAPE1_MIN_S;
    parameters.tape1_max_s = TRACK_TAPE1_MAX_S;
    parameters.tape3_straight_s = TRACK_TAPE3_STRAIGHT_S;
    parameters.tape3_right_deg = TRACK_TAPE3_RIGHT_DEG;
    parameters.tape3_speed_mps = TRACK_TAPE3_SPEED_MPS;
    parameters.tape3_min_s = TRACK_TAPE3_MIN_S;
    parameters.tape3_max_s = TRACK_TAPE3_MAX_S;
    parameters.tape1_straight_cm = TRACK_TAPE1_STRAIGHT_CM;
    parameters.tape3_straight_cm = TRACK_TAPE3_STRAIGHT_CM;
    parameters.exit_slow_distance_cm = TRACK_EXIT_SLOW_DISTANCE_CM;
    parameters.exit_slow_speed_mps = TRACK_EXIT_SLOW_SPEED_MPS;
    parameters.exit_slow_end_cm = TRACK_EXIT_SLOW_END_CM;
    parameters.exit_slow3_start_cm = TRACK_EXIT_SLOW3_START_CM;
    parameters.exit_slow3_end_cm = TRACK_EXIT_SLOW3_END_CM;
    parameters.finish_slow_start_cm = TRACK_FINISH_SLOW_START_CM;
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
    if ((!strcmp(key,"TEDIST") || !strcmp(key,"TE1S")) && value >= 0.0f && value <= 10000.0f)
        parameters.exit_slow_distance_cm=value;
    else if (!strcmp(key,"TE1E") && value >= 0.0f && value <= 10000.0f)
        parameters.exit_slow_end_cm=value;
    else if (!strcmp(key,"TE3S") && value >= 0.0f && value <= 10000.0f)
        parameters.exit_slow3_start_cm=value;
    else if (!strcmp(key,"TE3E") && value >= 0.0f && value <= 10000.0f)
        parameters.exit_slow3_end_cm=value;
    else if (!strcmp(key,"TEV") && value >= 0.05f && value <= ACK_MAX_WHEEL_SPEED_MPS)
        parameters.exit_slow_speed_mps=value;
    else if (!strcmp(key,"T1BD") && value >= 0.0f && value <= 200.0f)
        parameters.tape1_straight_cm=value;
    else if (!strcmp(key,"T3BD") && value >= 0.0f && value <= 200.0f)
        parameters.tape3_straight_cm=value;
    /* Legacy BT commands/readbacks retained; straight control now uses BD only. */
    else if (!strcmp(key,"T1BT") && value >= 0.0f && value <= 5.0f)
        parameters.tape1_straight_s=value;
    else if (!strcmp(key,"T3BT") && value >= 0.0f && value <= 5.0f)
        parameters.tape3_straight_s=value;
    else if (!strcmp(key,"T1BA") && value >= 0.0f && value <= 20.0f)
        parameters.tape1_right_deg=value;
    else if (!strcmp(key,"T3BA") && value >= 0.0f && value <= 20.0f)
        parameters.tape3_right_deg=value;
    else if (!strcmp(key,"T1BV") && value >= 0.05f && value <= ACK_MAX_WHEEL_SPEED_MPS)
        parameters.tape1_speed_mps=value;
    else if (!strcmp(key,"T3BV") && value >= 0.05f && value <= ACK_MAX_WHEEL_SPEED_MPS)
        parameters.tape3_speed_mps=value;
    else if (!strcmp(key,"T1BMIN") && value >= 0.0f && value <= 5.0f)
        parameters.tape1_min_s=value;
    else if (!strcmp(key,"T3BMIN") && value >= 0.0f && value <= 5.0f)
        parameters.tape3_min_s=value;
    else if (!strcmp(key,"T1BMAX") && value >= 0.1f && value <= 5.0f)
        parameters.tape1_max_s=value;
    else if (!strcmp(key,"T3BMAX") && value >= 0.1f && value <= 5.0f)
        parameters.tape3_max_s=value;
    else if (!strcmp(key,"THG") && value >= 0.0f && value <= 1.0f)
        parameters.local_heading_gain=value;
    else if (!strcmp(key,"THMAX") && value >= 0.0f && value <= 10.0f)
        parameters.local_heading_limit_deg=value;
    else if (!strcmp(key,"TFSTART") && value >= 0.0f && value <= 10000.0f)
        parameters.finish_slow_start_cm=value;
    else if (!strcmp(key,"TFV") && value >= 0.05f && value <= ACK_MAX_WHEEL_SPEED_MPS)
        parameters.finish_approach_speed_mps=value;
    else if (!strcmp(key,"TFBV") && value >= 0.0f && value <= ACK_MAX_WHEEL_SPEED_MPS)
        parameters.finish_blind_speed_mps=value;
    else if (!strcmp(key,"TFBT") && value >= 0.0f && value <= 5.0f)
        parameters.finish_blind_time_s=value;
    else if ((!strcmp(key,"TFDIST3") || !strcmp(key,"TFDIST")) && value >= 0.0f && value <= 100.0f)
        parameters.tape_finish_gap_cm=value;
    else if (!strcmp(key,"TFDIST5") && value >= 0.0f && value <= 100.0f)
        parameters.tape_finish_gap_m5_cm=value;
    else if (!strcmp(key,"TBA") && value >= -20.0f && value <= 20.0f)
        parameters.tape_recover_angle_deg=value;
    else if (!strcmp(key,"TBV") && value >= 0.05f && value <= ACK_MAX_WHEEL_SPEED_MPS)
        parameters.tape_recover_speed_mps=value;
    else if (!strcmp(key,"TBT") && value >= 0.1f && value <= 5.0f)
        parameters.tape_recover_time_s=value;
    else if (!strcmp(key,"TTDEC") && value >= 0.05f && value <= 2.0f)
        parameters.tape_decel_mps2 = value;
    else if (!strcmp(key,"TTDIST") && value >= 2.0f && value <= 100.0f)
        parameters.tape_gap_cm = value;
    else if (!strcmp(key,"TTWAIT") && value >= 0.1f && value <= 30.0f)
        parameters.tape_wait_s = value;
    else if (!strcmp(key,"TV") && value >= 0.01f && value <= ACK_MAX_WHEEL_SPEED_MPS)
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
    else if (!strcmp(key,"TSTART") && value >= 0.05f && value <= 3.0f)
        parameters.start_accel_mps2 = value;
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

/* Confirm repeated fresh detections, then retain the stop through brief losses. */
static void TapeLimit(TrackingController *c, const VisionTarget *v, uint32_t now,
                      float measured, const TrackingParameters *p, TrackingOutput *o)
{
    uint8_t fresh=(v->version==5U || v->version==6U) && v->link_alive &&
        (uint32_t)(now-v->received_ms)<TRACK_TAPE_GAP_MAX_MS;
    uint8_t seen=fresh && v->tape_detected;
    float remaining, a, delay, speed;
    if(c->tape_state==TAPE_PASS) {
        if(Tracking_Finite(measured))
            c->tape_resume_travel_m+=fmaxf(measured,0.0f)*ACK_CONTROL_DT_S;
        if(fresh && !v->tape_detected) {
            if(!c->tape_clear_started){c->tape_clear_started=1;c->tape_clear_ms=now;}
            if((uint32_t)(now-c->tape_clear_ms)>=500U && c->tape_resume_travel_m>=0.50f)
                c->tape_state=TAPE_SEARCH;
        } else c->tape_clear_started=0;
    } else if(c->tape_state==TAPE_SEARCH || c->tape_state==TAPE_CONFIRM) {
        if(!seen) { c->tape_state=TAPE_SEARCH;c->tape_seen_count=0; }
        else {
            if(c->tape_state==TAPE_SEARCH) {
                c->tape_state=TAPE_CONFIRM;c->tape_first_ms=v->received_ms;
                c->tape_last_ms=v->received_ms;c->tape_seen_count=1;
            } else if(v->received_ms!=c->tape_last_ms) {
                c->tape_last_ms=v->received_ms;++c->tape_seen_count;
            }
            c->tape_distance_m=v->tape_distance_y_centi_cm*0.0001f;
            if(c->tape_seen_count>=3U &&
               (uint32_t)(v->received_ms-c->tape_first_ms)>=TRACK_TAPE_CONFIRM_MS)
            {
                if(c->tape_count<4U) ++c->tape_count;
                c->tape_entry_active=0;
                c->tape_state=TAPE_APPROACH;
            }
        }
    } else if(c->tape_state==TAPE_APPROACH || c->tape_state==TAPE_BRAKE) {
        /* After confirmation, odometry also bridges loss in the camera near blind zone.
         * Fresh tape measurements may only bring the latched stop closer. */
        if(Tracking_Finite(measured))
            c->tape_distance_m=fmaxf(0,c->tape_distance_m-fmaxf(measured,0)*ACK_CONTROL_DT_S);
        if(seen && v->received_ms!=c->tape_last_ms) {
            c->tape_last_ms=v->received_ms;
            c->tape_distance_m=fminf(c->tape_distance_m,v->tape_distance_y_centi_cm*0.0001f);
        }
    }
    if(c->tape_state==TAPE_APPROACH || c->tape_state==TAPE_BRAKE) {
        remaining=c->tape_distance_m-TRACK_REAR_TO_FRONT_M-(c->tape_count>=4U ?
            (c->finish_approach_limit ? p->tape_finish_gap_m5_cm : p->tape_finish_gap_cm) : p->tape_gap_cm)*0.01f;
        a=p->tape_decel_mps2;delay=TRACK_TAPE_DELAY_S;
        /* Solve v*delay + v*v/(2*a) <= remaining. */
        o->tape_speed_limit=sqrtf(a*a*delay*delay+2*a*fmaxf(remaining,0))-a*delay;
        if(c->tape_count>=4U && p->finish_blind_time_s>0.0f && p->finish_blind_speed_mps>0.0f)
            o->tape_speed_limit=fmaxf(o->tape_speed_limit,p->finish_blind_speed_mps);
        speed=fmaxf(c->route_speed_mps,fmaxf(measured,0));
        c->tape_state=remaining<=speed*delay+speed*speed/(2*a) ? TAPE_BRAKE : TAPE_APPROACH;
        if(remaining<=TRACK_TAPE_STOP_TOLERANCE_M) {
            if(c->tape_count>=4U && p->finish_blind_time_s>0.0f && p->finish_blind_speed_mps>0.0f) { c->tape_state=TAPE_FINISH_BLIND;c->tape_recover_ms=now; }
            else { c->tape_state=c->tape_count>=4U ? TAPE_FINISH : TAPE_WAIT;c->tape_dwell_started=0;c->route_speed_mps=0; }
        }
    }
    o->tape_front_cm=(c->tape_distance_m-TRACK_REAR_TO_FRONT_M)*100.0f;
    o->tape_state=c->tape_state;
    o->tape_count=c->tape_count;
}

static uint8_t TapeLaneEntry(const TrackingController *c)
{
    return c->tape_lane_switch && (c->tape_count==1U || c->tape_count==3U);
}

static void TapeEntryBegin(TrackingController *c,const TrackingParameters *p)
{
    uint8_t first=c->tape_count==1U;
    c->tape_entry_active=1;c->tape_entry_visual=0;
    c->tape_entry_blending=0;c->tape_entry_last_steer_rad=0.0f;
    c->tape_entry_straight_s=first ? p->tape1_straight_s : p->tape3_straight_s;
    c->tape_entry_right_deg=first ? p->tape1_right_deg : p->tape3_right_deg;
    c->tape_entry_speed_mps=first ? p->tape1_speed_mps : p->tape3_speed_mps;
    c->tape_entry_min_s=first ? p->tape1_min_s : p->tape3_min_s;
    c->tape_entry_max_s=first ? p->tape1_max_s : p->tape3_max_s;
    c->tape_entry_straight_m=0.01f*(first ? p->tape1_straight_cm : p->tape3_straight_cm);
    c->tape_entry_travel_m=0.0f;
    c->tape_entry_straight_complete=c->tape_entry_straight_m<=0.0f;
}

void Tracking_AdvanceOdometry(TrackingController *c,float delta_m)
{
    if(!c->tape_entry_active || c->tape_entry_straight_complete ||
       !Tracking_Finite(delta_m)) return;
    /* Net signed travel: reverse motion must be made up before entry completes. */
    c->tape_entry_travel_m+=delta_m;
    if(c->tape_entry_travel_m>=c->tape_entry_straight_m)
        c->tape_entry_straight_complete=1;
}

/* Any fresh valid target received after departure start counts toward takeover. */
static uint8_t TapeEntryReliable(const TrackingController *c,const VisionTarget *v,uint32_t now)
{
    return v->valid && v->link_alive &&
        (uint32_t)(now-v->received_ms)<TRACK_TAPE_GAP_MAX_MS &&
        (int32_t)(v->received_ms-c->tape_recover_ms)>0;
}

static void TapeEntryCount(TrackingController *c,const VisionTarget *v,uint8_t reliable)
{
    if(!reliable) {
        c->tape_reliable_count=0;
        if((int32_t)(v->received_ms-c->tape_reliable_ms)>0)
            c->tape_reliable_ms=v->received_ms;
    } else if((int32_t)(v->received_ms-c->tape_reliable_ms)>0) {
        if((uint32_t)(v->received_ms-c->tape_reliable_ms)>=TRACK_TAPE_GAP_MAX_MS)
            c->tape_reliable_count=0;
        if(c->tape_reliable_count<3U) ++c->tape_reliable_count;
        c->tape_reliable_ms=v->received_ms;
    }
}

/* Return to the same departure guidance on loss; the original deadline and
 * straight phase never restart. Late recovery remains stopped until 3 frames. */
static uint8_t TapeEntry(TrackingController *c,const VisionTarget *v,uint32_t now,
                         const TrackingParameters *p,TrackingOutput *o)
{
    float elapsed;
    uint8_t reliable;
    if(!TapeLaneEntry(c)) return 0;
    if(!c->tape_entry_active) {
        if(c->tape_state!=TAPE_RECOVER && c->tape_state!=TAPE_FAULT) return 0;
        TapeEntryBegin(c,p);
    }
    elapsed=(uint32_t)(now-c->tape_recover_ms)*0.001f;
    reliable=TapeEntryReliable(c,v,now);
    o->tape_recover_remaining_s=fmaxf(c->tape_entry_max_s-elapsed,0.0f);
    /* I112 remains reserved at zero; seconds cannot represent distance remaining. */
    o->tape_entry_travel_m=c->tape_entry_travel_m;
    o->tape_entry_remaining_m=c->tape_entry_straight_complete ? 0.0f :
        fmaxf(c->tape_entry_straight_m-c->tape_entry_travel_m,0.0f);
    if(c->tape_entry_visual && reliable) {
        o->tape_entry_phase=3;
        if(elapsed>=c->tape_entry_max_s) {
            c->tape_entry_active=0;o->tape_entry_phase=0;
        }
        return 0;
    }
    if(c->tape_entry_visual) {
        c->tape_entry_visual=0;c->tape_reliable_count=0;
        c->tape_reliable_ms=v->received_ms;
        c->tape_state=TAPE_RECOVER;c->local_heading_rad=0.0f;
        c->tape_entry_blending=0;
        c->tape_entry_right_deg=-c->tape_entry_last_steer_rad*57.2957795f;
    }
    TapeEntryCount(c,v,reliable);
    /* At the deadline, stop first. A later group of frames may resume vision. */
    if(c->tape_state!=TAPE_FAULT && elapsed>=c->tape_entry_max_s) {
        c->tape_state=TAPE_FAULT;c->tape_reliable_count=0;
        c->tape_reliable_ms=now;
    }
    if(c->tape_reliable_count>=3U && c->tape_entry_straight_complete && elapsed>=c->tape_entry_min_s) {
        c->tape_state=TAPE_PASS;c->launch_complete=0;c->tape_entry_visual=1;
        c->tape_entry_blending=1;c->tape_entry_blend_ms=now;
        c->tape_entry_blend_from_rad=c->tape_entry_last_steer_rad;
        if(elapsed>=c->tape_entry_max_s) c->tape_entry_active=0;
        o->tape_entry_phase=c->tape_entry_active ? 3 : 0;
        return 0;
    }
    o->valid=1;
    if(c->tape_state==TAPE_FAULT) {
        c->route_speed_mps=0;o->tape_entry_phase=4;
        c->tape_entry_last_steer_rad=0.0f;
    } else {
        o->tape_entry_phase=c->tape_entry_straight_complete ? 2 : 1;
        o->speed_mps=o->requested_speed_mps=c->route_speed_mps=c->tape_entry_speed_mps;
        o->steer_left_rad=o->raw_steer_left_rad=o->tape_entry_phase==1 ? 0.0f :
            -c->tape_entry_right_deg*0.01745329252f;
        o->correction_right_rad=-o->steer_left_rad;
        c->tape_entry_last_steer_rad=o->steer_left_rad;
    }
    o->tape_state=c->tape_state;
    return 1;
}

static uint8_t TapeWait(TrackingController *c,const VisionTarget *v,uint32_t now,
                       float measured,const TrackingParameters *p,TrackingOutput *o)
{
    if(c->tape_state==TAPE_FINISH_BLIND) {
        float elapsed=(uint32_t)(now-c->tape_recover_ms)*0.001f;
        o->valid=1;o->tape_front_cm=(c->tape_distance_m-TRACK_REAR_TO_FRONT_M)*100.0f;
        if(elapsed>=p->finish_blind_time_s || p->finish_blind_speed_mps<=0.0f) { c->tape_state=TAPE_FINISH;c->route_speed_mps=0; }
        else { o->speed_mps=o->requested_speed_mps=c->route_speed_mps=p->finish_blind_speed_mps;o->tape_recover_remaining_s=p->finish_blind_time_s-elapsed; }
        o->tape_state=c->tape_state;return 1;
    }
    if(c->tape_state!=TAPE_WAIT && c->tape_state!=TAPE_FAULT && c->tape_state!=TAPE_FINISH) return 0;
    o->valid=1;o->tape_state=c->tape_state;
    o->tape_front_cm=(c->tape_distance_m-TRACK_REAR_TO_FRONT_M)*100.0f;
    c->route_speed_mps=0;
    if(c->tape_state==TAPE_FINISH) return 1;
    if(c->tape_state==TAPE_FAULT) {
        if(TapeLaneEntry(c)) return 0;
        if(v->valid && v->link_alive &&
           (uint32_t)(now-v->received_ms)<TRACK_TAPE_GAP_MAX_MS &&
           (int32_t)(v->received_ms-c->tape_reliable_ms)>0) {
            c->tape_state=TAPE_PASS;c->launch_complete=0;
            c->tape_resume_travel_m=0;c->tape_clear_started=0;
            return 0;
        }
        return 1;
    }
    if(!Tracking_Finite(measured) || fabsf(measured)>0.02f) c->tape_dwell_started=0;
    else if(!c->tape_dwell_started){c->tape_dwell_started=1;c->tape_stop_ms=now;}
    o->tape_wait_remaining_s=c->tape_dwell_started ?
        fmaxf(p->tape_wait_s-(uint32_t)(now-c->tape_stop_ms)*0.001f,0) : p->tape_wait_s;
    if(c->tape_dwell_started && o->tape_wait_remaining_s<=0) {
        if(TapeLaneEntry(c)) {
            c->tape_lane_event=0;
            TapeEntryBegin(c,p);
        }
        c->tape_state=TAPE_RECOVER;c->tape_recover_ms=now;
        c->tape_reliable_ms=v->received_ms;c->tape_reliable_count=0;
        c->tape_resume_travel_m=0;c->tape_clear_started=0;
        c->launch_complete=0;
        return 0;
    }
    return 1;
}

/* Recovery eligibility only requires a valid, unexpired target.
 * Three successive received frames reject isolated detections and cached targets. */
static uint8_t TapeRecover(TrackingController *c, const VisionTarget *v,
                          uint32_t now, const TrackingParameters *p, TrackingOutput *o)
{
    uint8_t reliable;
    float elapsed, minimum, duration;
    if(c->tape_state!=TAPE_RECOVER || TapeLaneEntry(c)) return 0;
    minimum=(!c->tape_skip_recover && (c->tape_count==1U || c->tape_count==3U)) ? 1.0f : 0.0f;
    duration=fmaxf(p->tape_recover_time_s,minimum);
    elapsed=(uint32_t)(now-c->tape_recover_ms)*0.001f;
    reliable=v->valid && (uint32_t)(now-v->received_ms)<TRACK_TAPE_GAP_MAX_MS;
    if(c->tape_skip_recover && reliable) {
        c->tape_state=TAPE_PASS;c->launch_complete=0;
        return 0;
    }
    if(!reliable) c->tape_reliable_count=0;
    else if(v->received_ms!=c->tape_reliable_ms) {
        if((uint32_t)(v->received_ms-c->tape_reliable_ms)>=TRACK_TAPE_GAP_MAX_MS)
            c->tape_reliable_count=0;
        if(c->tape_reliable_count<3U) ++c->tape_reliable_count;
        c->tape_reliable_ms=v->received_ms;
    }
    if(c->tape_reliable_count>=3U && elapsed>=minimum &&
       (elapsed<duration || (minimum>0 && elapsed==duration))) {
        c->tape_state=TAPE_PASS;c->launch_complete=1;
        return 0;
    }
    o->valid=1;
    o->tape_recover_remaining_s=fmaxf(duration-elapsed,0);
    if(elapsed>=duration) {
        c->tape_state=TAPE_FAULT;c->route_speed_mps=0;
        c->tape_reliable_ms=v->received_ms;
    } else {
        o->speed_mps=o->requested_speed_mps=c->tape_skip_recover ?
            fminf(p->hold_speed_mps,p->speed_mps) : p->tape_recover_speed_mps;
        o->steer_left_rad=o->raw_steer_left_rad=c->tape_skip_recover ?
            0.0f :
            p->tape_recover_angle_deg*0.01745329252f;
        o->correction_right_rad=-o->steer_left_rad;
        c->route_speed_mps=o->speed_mps;
    }
    o->tape_state=c->tape_state;
    return 1;
}

static void ExitSlowUpdate(TrackingController *c,const TrackingParameters *p)
{
    uint8_t first_valid=p->exit_slow_distance_cm<p->exit_slow_end_cm;
    uint8_t third_valid=p->exit_slow3_start_cm<p->exit_slow3_end_cm;
    c->exit_slow_active=0;c->exit_slow_interval=0;
    if(!c->tape_lane_switch) return; /* M3 only. */
    if(c->tape_count>=2U) c->exit_slow_done|=1U;
    if(c->tape_count>=4U) c->exit_slow_done|=2U;
    if(!Tracking_Finite(c->exit_calibration_m)) return;
    /* Invalid pairs are inactive while the two independent sliders are edited. */
    if(first_valid && c->exit_calibration_m>=p->exit_slow_end_cm*0.01f) c->exit_slow_done|=1U;
    if(third_valid && c->exit_calibration_m>=p->exit_slow3_end_cm*0.01f) c->exit_slow_done|=2U;
    if(c->tape_count==1U && !(c->exit_slow_done&1U) && first_valid &&
       c->exit_calibration_m>=p->exit_slow_distance_cm*0.01f)
        c->exit_slow_interval=1;
    if(c->tape_count==3U && !(c->exit_slow_done&2U) && third_valid &&
       c->exit_calibration_m>=p->exit_slow3_start_cm*0.01f)
        c->exit_slow_interval=3;
    c->exit_slow_active=c->exit_slow_interval!=0;
}

/* Mileage shares ORESET/I123 in both vision modes and is checked every tick. */
static uint8_t FinishSlowActive(const TrackingController *c,const TrackingParameters *p)
{
    return Tracking_Finite(c->exit_calibration_m) &&
        c->exit_calibration_m>=p->finish_slow_start_cm*0.01f;
}

static uint8_t Tracking_UpdateRouteCore(TrackingController *controller,
    const VisionTarget *control, const VisionTarget *latest, uint32_t now,
    float measured_speed, const TrackingParameters *p, TrackingOutput *out)
{
    float x, y, d2, curvature, excess, floor, requested, step;
    uint8_t fresh, v4, curve_known;
    float gain;
    memset(out, 0, sizeof(*out));
    if (controller->tape_state==TAPE_WAIT || controller->tape_state==TAPE_RECOVER ||
        controller->tape_state==TAPE_FAULT || controller->tape_state==TAPE_FINISH)
        controller->local_heading_rad=0.0f;
    TapeLimit(controller,latest,now,measured_speed,p,out);
    ExitSlowUpdate(controller,p); /* A newly confirmed next tape ends the cap now. */
    if (TapeWait(controller,latest,now,measured_speed,p,out)) return 1U;
    if (TapeEntry(controller,latest,now,p,out)) return 1U;
    if (TapeRecover(controller,latest,now,p,out)) return 1U;
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
    v4 = (control->version == 4U || control->version == 5U || control->version == 6U);
    curve_known = v4 && (latest->version == 4U || latest->version == 5U || latest->version == 6U) && latest->link_alive &&
        (uint32_t)(now-latest->received_ms) < TRACK_ROUTE_FRESH_MS &&
        latest->endpoint_valid && latest->endpoint_y_centi_cm > 0 && latest->curvature_valid;
    if (!controller->gain_initialized) {
        controller->active_pp_gain = p->pp_straight_gain;
        controller->gain_curve = 0U; /* Curves require three new qualifying frames. */
        controller->gain_initialized = 1U;
    }
    gain = p->pp_gain;
    out->gain_state = 4U;
    if (v4) {
        out->gain_state = !fresh ? 3U : !curve_known ? 2U : controller->gain_curve;
        if (fresh && curve_known) {
            float road = fminf(latest->endpoint_abs_curvature_cm_inv,1.0f)*100.0f;
            /* Road curvature, not gain-scaled pursuit error: avoids feedback switching. */
            if (!controller->gain_frame_seen || latest->accepted!=controller->gain_last_accepted) {
                if (controller->gain_frame_seen &&
                    (uint32_t)(latest->accepted-controller->gain_last_accepted)!=1U)
                    controller->gain_enter_count=0; /* Unobserved frames cannot confirm entry. */
                controller->gain_frame_seen=1;
                controller->gain_last_accepted=latest->accepted;
                if (road >= p->gain_curve_enter) {
                    if (controller->gain_enter_count<3U) ++controller->gain_enter_count;
                    if (controller->gain_enter_count>=3U) controller->gain_curve=1U;
                } else {
                    controller->gain_enter_count=0;
                    if (road <= p->gain_curve_exit) controller->gain_curve=0U;
                }
            }
            out->gain_state = controller->gain_curve;
            gain = controller->gain_curve ? p->pp_gain : p->pp_straight_gain;
            controller->active_pp_gain += Ack_Clamp(gain-controller->active_pp_gain,
                -TRACK_GAIN_RATE_PER_S*ACK_CONTROL_DT_S, TRACK_GAIN_RATE_PER_S*ACK_CONTROL_DT_S);
        }
        if (!fresh || !curve_known) {
            controller->gain_enter_count=0;
            controller->gain_last_accepted=latest->accepted;
            controller->gain_frame_seen=1;
        }
        gain = controller->active_pp_gain;
    } else {
        controller->active_pp_gain = gain;
        controller->gain_enter_count=0;controller->gain_frame_seen=0;
    }
    if (!Ack_PurePursuitGain(x,y,gain,&out->steer_left_rad)) return 0U;
    out->active_pp_gain = gain;
    out->raw_steer_left_rad = atanf(ACK_WHEELBASE_M*gain*curvature);
    /* Local heading is an optional straight-road supplement, never a held yaw.
     * Reference must remain near the calibrated 41.1 cm forward sample.
     * Endpoint curvature is only a conservative gate, not signed feedforward. */
    if (fresh && latest->version==6U && latest->heading_valid &&
        !latest->fallback && curve_known && !controller->gain_curve &&
        latest->endpoint_abs_curvature_cm_inv*100.0f <= p->gain_curve_exit &&
        latest->heading_reference_y_centi_cm>=2500 && latest->heading_reference_y_centi_cm<=5500 &&
        latest->heading_reference_x_centi_cm>=-2000 && latest->heading_reference_x_centi_cm<=2000 &&
        latest->heading_centi_deg>=-3000 && latest->heading_centi_deg<=3000 &&
        p->local_heading_gain>0.0f) {
        float limit=p->local_heading_limit_deg*0.0174532925f;
        float desired=Ack_Clamp(p->local_heading_gain*latest->heading_centi_deg*0.000174532925f,-limit,limit);
        controller->local_heading_rad += Ack_Clamp(desired-controller->local_heading_rad,
            -0.523598776f*ACK_CONTROL_DT_S,0.523598776f*ACK_CONTROL_DT_S);
        controller->local_heading_rad=Ack_Clamp(controller->local_heading_rad,-limit,limit);
        out->heading_used=1U;
    } else controller->local_heading_rad=0.0f;
    out->heading_rad=controller->local_heading_rad;
    out->raw_steer_left_rad+=out->heading_rad;
    if(controller->tape_entry_blending) {
        float blend=Ack_Clamp((uint32_t)(now-controller->tape_entry_blend_ms)/200.0f,0.0f,1.0f);
        out->raw_steer_left_rad=controller->tape_entry_blend_from_rad+
            blend*(out->raw_steer_left_rad-controller->tape_entry_blend_from_rad);
        if(blend>=1.0f) controller->tape_entry_blending=0;
    }
    out->steer_left_rad=Ack_Clamp(out->raw_steer_left_rad,-ACK_MAX_STEER_RAD,ACK_MAX_STEER_RAD);
    if(controller->tape_entry_active) controller->tape_entry_last_steer_rad=out->steer_left_rad;
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
        if(controller->exit_slow_active) requested=fminf(requested,p->exit_slow_speed_mps);
        if(FinishSlowActive(controller,p)) requested=fminf(requested,p->finish_approach_speed_mps);
        out->requested_speed_mps = requested;
        step = (requested > controller->route_speed_mps ?
            (controller->launch_complete ? p->accel_mps2 : p->start_accel_mps2) :
            p->decel_mps2)*ACK_CONTROL_DT_S;
        controller->route_speed_mps += Ack_Clamp(requested-controller->route_speed_mps,-step,step);
        out->speed_mps = Ack_Clamp(controller->route_speed_mps,0.0f,p->speed_mps);
        controller->route_speed_mps = out->speed_mps;
        /* Only a fresh positive target speed can finish the first launch.
         * Holding stale data must never re-arm or complete the launch phase. */
        if (fresh && requested > 0.0f && out->speed_mps >= requested)
            controller->launch_complete = 1U;
        out->startup_active = !controller->launch_complete;
    } else {
        /* Backward-compatible V3 tuning, including existing TXTH/TXK commands. */
        out->requested_speed_mps = out->speed_mps = Tracking_TurnSpeed(x,p);
        controller->route_speed_mps = out->speed_mps;
        if (!fresh) {
            out->route_reasons |= TRACK_REASON_HOLD;
            out->route_state = TRACK_ROUTE_HOLD;
        }
    }
    if(controller->tape_state==TAPE_APPROACH || controller->tape_state==TAPE_BRAKE) {
        if(controller->finish_approach_limit && controller->tape_count>=4U)
            out->tape_speed_limit=fminf(out->tape_speed_limit,p->finish_approach_speed_mps);
        /* Apply after ordinary ramps: TDEC and curve floors cannot weaken tape braking. */
        out->requested_speed_mps=fminf(out->requested_speed_mps,out->tape_speed_limit);
        out->speed_mps=fminf(out->speed_mps,out->tape_speed_limit);
        controller->route_speed_mps=out->speed_mps;
    }
    out->tape_state=controller->tape_state;
    out->valid = 1U;
    return 1U;
}

uint8_t Tracking_UpdateRoute(TrackingController *controller,
    const VisionTarget *control,const VisionTarget *latest,uint32_t now,
    float measured_speed,const TrackingParameters *p,TrackingOutput *out)
{
    float previous_speed=controller->route_speed_mps;
    float slow_speed;
    uint8_t ok;
    ExitSlowUpdate(controller,p);
    ok=Tracking_UpdateRouteCore(controller,control,latest,now,measured_speed,p,out);
    /* Reflect any tape transition in the returned diagnostics. */
    ExitSlowUpdate(controller,p);
    out->exit_slow_active=controller->exit_slow_active;
    out->exit_slow_interval=controller->exit_slow_interval;
    out->finish_slow_active=FinishSlowActive(controller,p);
    slow_speed=controller->exit_slow_active ? p->exit_slow_speed_mps : p->speed_mps;
    if(out->finish_slow_active) slow_speed=fminf(slow_speed,p->finish_approach_speed_mps);
    if(ok && (controller->exit_slow_active || out->finish_slow_active)) {
        float limit=fmaxf(slow_speed,previous_speed-p->decel_mps2*ACK_CONTROL_DT_S);
        /* Normal pursuit caps its request before ramping. This common bound also
         * covers V3 and blind entry, while never weakening a lower safety target. */
        out->requested_speed_mps=fminf(out->requested_speed_mps,slow_speed);
        out->speed_mps=fminf(out->speed_mps,limit);
        controller->route_speed_mps=out->speed_mps;
    }
    return ok;
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
