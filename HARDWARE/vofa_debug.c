#include "vofa_debug.h"
#include "usart.h"
#include "balance.h"
#include "mode_control.h"
#include "distance_measurement.h"
#include "vision_command.h"
#include <math.h>
#include <string.h>

#define RX_SIZE 256U
#define CHANNELS 115U
static uint8_t rx_byte;
static volatile uint8_t rx_ring[RX_SIZE];
static volatile unsigned int rx_head, rx_tail, rx_fault;
static char command[48];
static unsigned int command_length, discard_line;
static uint32_t last_tx;
static float accepted, rejected;
static struct { float channels[CHANNELS]; uint8_t tail[4]; } frame;

void Vofa_Init(void)
{
    rx_head = rx_tail = rx_fault = 0;
    command_length = discard_line = 0;
    accepted = rejected = 0;
    last_tx = HAL_GetTick();
    frame.tail[0] = 0; frame.tail[1] = 0;
    frame.tail[2] = 0x80; frame.tail[3] = 0x7f;
    HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
}

void Vofa_RxComplete(void)
{
    unsigned int next = (rx_head + 1U) % RX_SIZE;
    if (next == rx_tail) rx_fault = 1;
    else { rx_ring[rx_head] = rx_byte; rx_head = next; }
    HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
}

void Vofa_RxError(void)
{
    rx_fault = 1; /* Never execute a command assembled across lost bytes. */
    __HAL_UART_CLEAR_OREFLAG(&huart3);
    HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
}

/* Bounded ASCII decimal parser: no heap/locale, no NaN/Inf/exponent forms. */
static uint8_t ParseDecimal(const char *p, float *result)
{
    float value = 0.0f, scale = 0.1f, sign = 1.0f;
    unsigned int digits = 0, fraction = 0;
    if (*p == '-' || *p == '+') { if (*p == '-') sign = -1.0f; p++; }
    while (*p) {
        if (*p == '.' && !fraction) { fraction = 1; p++; continue; }
        if (*p < '0' || *p > '9') return 0;
        digits++;
        if (fraction) { value += (*p - '0') * scale; scale *= 0.1f; }
        else value = value * 10.0f + (*p - '0');
        if (value > 100000.0f) return 0;
        p++;
    }
    *result = sign * value;
    return digits ? 1U : 0U;
}

static void ExecuteCommand(void)
{
    char *separator = strchr(command, '=');
    float value;
    uint32_t mask;
    uint8_t ok;
    if (!separator || separator == command) { rejected++; return; }
    *separator++ = '\0';
    if (!ParseDecimal(separator, &value)) { rejected++; return; }
    /* Atomically update the same state used by the 100 Hz control ISR. */
    mask = __get_PRIMASK();
    __disable_irq();
    ok = ModeControl_DebugCommand(command, value);
    __set_PRIMASK(mask);
    if (ok) accepted++; else rejected++;
}

void Vofa_Task(void)
{
    VisionTarget target, control_target;
    TrackingOutput output;
    TrackingParameters params;
    AckSpeedPI left_pi, right_pi;
    unsigned int budget = RX_SIZE;
    uint32_t mask, now;
    uint8_t byte;
    while (budget--) {
        mask = __get_PRIMASK();
        __disable_irq();
        if (rx_fault) {
            rx_tail = rx_head; rx_fault = 0;
            command_length = 0; discard_line = 1; rejected++;
        }
        if (rx_tail == rx_head) { __set_PRIMASK(mask); break; }
        byte = rx_ring[rx_tail];
        rx_tail = (rx_tail + 1U) % RX_SIZE;
        __set_PRIMASK(mask);
        if (byte == '\n' || byte == ';') {
            if (!discard_line && command_length) {
                command[command_length] = '\0';
                ExecuteCommand();
            }
            command_length = discard_line = 0;
        } else if (byte == '\r') {
            /* Accept CRLF and semicolon followed by CRLF. */
        } else if (!discard_line) {
            if (byte < 32U || byte > 126U || command_length >= sizeof(command)-1U) {
                command_length = 0; discard_line = 1; rejected++;
            } else command[command_length++] = (char)byte;
        }
    }
    now = HAL_GetTick();
    if ((uint32_t)(now - last_tx) < 50U || huart3.gState != HAL_UART_STATE_READY) return;
    /* One coherent snapshot. Persistent TX buffer stays unchanged until ready. */
    mask = __get_PRIMASK();
    __disable_irq();
    frame.channels[0] = MOTOR_A.Encoder;
    frame.channels[1] = MOTOR_B.Encoder;
    frame.channels[2] = (Servo - ACK_SERVO_CENTER_US) * 57.2957795f /
                        (ACK_SERVO_DIRECTION * ACK_SERVO_US_PER_RAD);
    frame.channels[3] = MOTOR_A.Target;
    frame.channels[4] = MOTOR_B.Target;
    frame.channels[5] = (float)Servo;
    frame.channels[6] = (float)ModeControl_GetDisplayMode();
    frame.channels[7] = (float)ModeControl_GetStatus();
    frame.channels[8] = Voltage;
    frame.channels[9] = ModeControl_DebugLeft();
    frame.channels[10] = ModeControl_DebugRight();
    frame.channels[11] = ModeControl_DebugAngle();
    frame.channels[12] = accepted;
    frame.channels[13] = rejected;
    Vision_GetTarget(&target);
    Vision_GetControlTarget(&control_target);
    ModeControl_GetTrackingOutput(&output);
    now = HAL_GetTick();
    frame.channels[14] = target.x_centi_cm * 0.01f;
    frame.channels[15] = !target.has_frame ? 0.0f : !target.link_alive ? 3.0f : target.valid ? 1.0f : 2.0f;
    frame.channels[16] = target.valid;
    frame.channels[17] = target.sequence;
    frame.channels[18] = target.has_frame ? (float)(uint32_t)(now-target.received_ms) : -1.0f;
    frame.channels[19] = -target.x_centi_cm * 0.0001f;
    frame.channels[20] = output.steer_left_rad * 57.2957795f;
    frame.channels[21] = control_target.valid;
    Tracking_GetParameters(&params);
    frame.channels[22] = target.y_centi_cm * 0.01f;
    {
        float x = target.x_centi_cm * 0.0001f;
        float y = target.y_centi_cm * 0.0001f;
        float d2 = x*x+y*y;
        frame.channels[23] = sqrtf(d2);
        frame.channels[24] = target.valid && d2 > 0.0f ? -2.0f*x/d2 : 0.0f;
        frame.channels[25] = atanf(ACK_WHEELBASE_M*frame.channels[24])*57.2957795f;
    }
    frame.channels[26] = target.link_alive;
    frame.channels[27] = 0.5f * (MOTOR_A.Encoder + MOTOR_B.Encoder);
    frame.channels[28] = params.speed_mps;
    frame.channels[29] = params.turn_x_threshold_cm;
    frame.channels[30] = params.turn_speed_gain;
    frame.channels[31] = params.turn_min_speed_mps;
    frame.channels[32] = output.speed_mps;
    frame.channels[33] = output.valid &&
        (output.correction_right_rad >= TRACK_STEER_LIMIT_RAD ||
         output.correction_right_rad <= -TRACK_STEER_LIMIT_RAD) ? 1.0f : 0.0f;
    frame.channels[34] = target.version;
    frame.channels[35] = target.mode;
    frame.channels[36] = target.fallback;
    frame.channels[37] = target.endpoint_valid;
    frame.channels[38] = target.endpoint_x_centi_cm*0.01f;
    frame.channels[39] = target.endpoint_y_centi_cm*0.01f;
    frame.channels[40] = target.curvature_valid;
    frame.channels[41] = target.endpoint_abs_curvature_cm_inv; /* wire cm^-1 */
    frame.channels[42] = output.route_state;
    frame.channels[43] = output.route_reasons;
    frame.channels[44] = output.speed_curvature_m_inv;
    frame.channels[45] = output.requested_speed_mps; /* before speed ramp */
    frame.channels[46] = output.control_x_m*100.0f;
    frame.channels[47] = output.control_y_m*100.0f;
    frame.channels[65] = params.start_accel_mps2;
    frame.channels[66] = output.startup_active;
    SpeedPI_GetState(&left_pi, &right_pi);
    /* Positional PI: P/I are full contributions before output saturation.
     * PWM is clamp(P + I), then truncated to the signed motor command. */
    frame.channels[80] = MOTOR_A.Motor_Pwm;
    frame.channels[81] = left_pi.p_output;
    frame.channels[82] = left_pi.i_output;
    frame.channels[83] = MOTOR_B.Motor_Pwm;
    frame.channels[84] = right_pi.p_output;
    frame.channels[85] = right_pi.i_output;
    frame.channels[86] = params.tape_recover_angle_deg;
    frame.channels[87] = params.tape_recover_speed_mps;
    frame.channels[88] = params.tape_recover_time_s;
    frame.channels[89] = output.tape_count;
    frame.channels[90] = output.tape_recover_remaining_s;
    frame.channels[91] = params.tape_finish_gap_cm;
    frame.channels[92] = params.finish_approach_speed_mps;
    frame.channels[93] = target.heading_valid;
    frame.channels[94] = target.heading_centi_deg*0.01f;
    frame.channels[95] = target.heading_reference_x_centi_cm*0.01f;
    frame.channels[96] = target.heading_reference_y_centi_cm*0.01f;
    frame.channels[97] = output.heading_rad*57.2957795f;
    frame.channels[98] = params.local_heading_gain;
    frame.channels[99] = params.local_heading_limit_deg;
    frame.channels[100] = output.heading_used;
    frame.channels[101] = params.finish_blind_speed_mps;
    frame.channels[102] = params.finish_blind_time_s;
    frame.channels[103] = params.tape1_straight_s;
    frame.channels[104] = params.tape1_right_deg;
    frame.channels[105] = params.tape1_speed_mps;
    frame.channels[106] = params.tape1_max_s;
    frame.channels[107] = params.tape3_straight_s;
    frame.channels[108] = params.tape3_right_deg;
    frame.channels[109] = params.tape3_speed_mps;
    frame.channels[110] = params.tape3_max_s;
    frame.channels[111] = output.tape_entry_phase;
    frame.channels[112] = output.tape_entry_straight_remaining_s;
    frame.channels[113] = params.tape1_min_s;
    frame.channels[114] = params.tape3_min_s;
    frame.channels[76] = MOTOR_A.Velocity_KP;
    frame.channels[77] = MOTOR_A.Velocity_KI;
    frame.channels[78] = MOTOR_B.Velocity_KP;
    frame.channels[79] = MOTOR_B.Velocity_KI;
    frame.channels[67] = target.tape_detected;
    frame.channels[68] = target.tape_distance_y_centi_cm*0.01f;
    frame.channels[69] = output.tape_state;
    frame.channels[70] = output.tape_front_cm;
    frame.channels[71] = output.tape_speed_limit;
    frame.channels[72] = params.tape_decel_mps2;
    frame.channels[73] = params.tape_gap_cm;
    frame.channels[74] = params.tape_wait_s;
    frame.channels[75] = output.tape_wait_remaining_s;
    frame.channels[60] = params.pp_straight_gain;
    frame.channels[61] = params.gain_curve_enter;
    frame.channels[62] = params.gain_curve_exit;
    frame.channels[63] = output.active_pp_gain;
    frame.channels[64] = output.gain_state;
    frame.channels[48] = params.pp_gain;
    frame.channels[49] = params.curve_threshold;
    frame.channels[50] = params.curve_speed_gain;
    frame.channels[51] = params.single_speed_mps;
    frame.channels[52] = params.fallback_speed_mps;
    frame.channels[53] = params.unknown_speed_mps;
    frame.channels[54] = params.hold_speed_mps;
    frame.channels[55] = params.saturated_speed_mps;
    frame.channels[56] = params.accel_mps2;
    frame.channels[57] = params.decel_mps2;
    frame.channels[58] = output.raw_steer_left_rad*57.2957795f;
    frame.channels[59] = control_target.has_frame ? (float)(uint32_t)(now-control_target.received_ms) : -1.0f;
    __set_PRIMASK(mask);
    if (HAL_UART_Transmit_IT(&huart3, (uint8_t *)&frame, sizeof(frame)) == HAL_OK)
        last_tx = now;
}
