#include "vofa_debug.h"
#include "usart.h"
#include "balance.h"
#include "mode_control.h"
#include "distance_measurement.h"
#include <string.h>

#define RX_SIZE 256U
#define CHANNELS 34U
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
        if (value > 10000.0f) return 0;
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
    DistanceMeasurement distance;
    TrackingInput tracking;
    TrackingOutput output;
    TrackingParameters params;
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
    Distance_Get(&distance);
    Tracking_GetInput(&tracking);
    ModeControl_GetTrackingOutput(&output);
    now = HAL_GetTick();
    frame.channels[14] = distance.measurement_valid ? distance.distance_raw * 0.01f : 0.0f;
    frame.channels[15] = (float)distance.state;
    frame.channels[16] = (float)distance.measurement_valid;
    frame.channels[17] = (float)distance.sequence;
    frame.channels[18] = distance.has_frame ? (float)(uint32_t)(now-distance.received_ms) : -1.0f;
    frame.channels[19] = output.error_m;
    frame.channels[20] = output.steer_left_rad * 57.2957795f;
    frame.channels[21] = (float)Tracking_CanRun(&tracking, now);
    Tracking_GetParameters(&params);
    frame.channels[22] = output.derivative_mps;
    frame.channels[23] = output.p_rad * 57.2957795f;
    frame.channels[24] = output.d_rad * 57.2957795f;
    frame.channels[25] = output.stanley_rad * 57.2957795f;
    frame.channels[26] = output.heading_rad * 57.2957795f;
    frame.channels[27] = 0.5f * (MOTOR_A.Encoder + MOTOR_B.Encoder);
    frame.channels[28] = params.speed_mps;
    frame.channels[29] = params.kp;
    frame.channels[30] = params.kd;
    frame.channels[31] = params.stanley_k;
    frame.channels[32] = params.filter_tau_s;
    frame.channels[33] = output.valid &&
        (output.correction_right_rad >= TRACK_STEER_LIMIT_RAD ||
         output.correction_right_rad <= -TRACK_STEER_LIMIT_RAD) ? 1.0f : 0.0f;
    __set_PRIMASK(mask);
    if (HAL_UART_Transmit_IT(&huart3, (uint8_t *)&frame, sizeof(frame)) == HAL_OK)
        last_tx = now;
}
