/* Real parser/state machine/telemetry with mocked UART. */
#define test_entry original_control_tests
#include "test_control.c"
#undef test_entry
#define __USART_H__
#define HAL_UART_STATE_READY 0
#define HAL_OK 0
typedef struct { int gState; } UART_HandleTypeDef;
static UART_HandleTypeDef huart3;
static uint8_t transmitted[160];
static unsigned int transmitted_size, tx_count;
#define __HAL_UART_CLEAR_OREFLAG(h) ((void)(h))
int HAL_UART_Receive_IT(UART_HandleTypeDef *h, uint8_t *p, unsigned int n)
{ (void)h; (void)p; (void)n; return HAL_OK; }
int HAL_UART_Transmit_IT(UART_HandleTypeDef *h, uint8_t *p, unsigned int n)
{
    unsigned int j;
    for (j = 0; j < n; ++j) transmitted[j] = p[j];
    transmitted_size = n; tx_count++; h->gState = 1;
    return HAL_OK;
}
#include "../HARDWARE/vofa_debug.c"
static void receive_text(const char *p)
{
    while (*p) { rx_byte = (uint8_t)*p++; Vofa_RxComplete(); }
    Vofa_Task();
}
void test_entry(void)
{
    unsigned int j;
    float sample;
    reset_board(0, 0);
    Vofa_Init();
    case_id = 1;
    receive_text("M=4;V=0.3;A=0;RUN=1;");
    advance(10);
    REQUIRE(accepted == 4 && rejected == 0 && ModeControl_IsActive());
    REQUIRE(MOTOR_A.Target == 0.3f && MOTOR_B.Target == 0.3f && Servo == 1538);
    receive_text("L=0.");
    REQUIRE(ModeControl_DebugLeft() == 0.3f);
    receive_text("25\r\nR=-0.2;A=10;"); advance(10);
    REQUIRE(MOTOR_A.Target == 0.25f && MOTOR_B.Target == -0.2f);
    receive_text("A=27;"); advance(10);
    REQUIRE(ModeControl_DebugAngle()==27.0f && Servo==1020 && Servo_PWM==1020);
    receive_text("A=-27;"); advance(10);
    REQUIRE(ModeControl_DebugAngle()==-27.0f && Servo==2056 && Servo_PWM==2056);
    REQUIRE(!ModeControl_DebugCommand("A",27.1f) && ModeControl_DebugAngle()==-27.0f);
    REQUIRE(!ModeControl_DebugCommand("A",-27.1f) && ModeControl_DebugAngle()==-27.0f);
    REQUIRE(Ack_ServoPulse(1.0f)==Ack_ServoPulse(ACK_MAX_STEER_RAD));
    receive_text("A=10;"); advance(10);
    receive_text("A=nan;V=inf;L=12;R=2junk;BAD=1;M=2.5;=2;");
    REQUIRE(rejected == 7 && MOTOR_A.Target == 0.25f);
    receive_text("RUN=0;");
    REQUIRE(!ModeControl_IsActive() && Servo == 1538);
    case_id = 2;
    for (j = 0; j < 60; ++j) { rx_byte = 'X'; Vofa_RxComplete(); }
    receive_text("RUN=1;V=0.2;");
    REQUIRE(!ModeControl_IsActive() && ModeControl_DebugLeft() == 0.2f);
    for (j = 0; j < 300; ++j) { rx_byte = 'X'; Vofa_RxComplete(); }
    Vofa_Task();
    receive_text("RUN=1;V=0.3;");
    REQUIRE(!ModeControl_IsActive() && ModeControl_DebugRight() == 0.3f);
    receive_text("RUN="); Vofa_RxError(); Vofa_Task();
    receive_text("1;A=0;"); REQUIRE(!ModeControl_IsActive());
    case_id = 3;
    tx_count = 0; /* Earlier endpoint checks also advance the telemetry clock. */
    huart3.gState = HAL_UART_STATE_READY;
    now_ms = 100; MOTOR_A.Encoder = 1.5f; MOTOR_B.Encoder = -0.5f;
    Vofa_Task();
    REQUIRE(transmitted_size == 140 && tx_count == 1);
    memcpy(&sample, transmitted, 4); REQUIRE(sample == 1.5f);
    memcpy(&sample, transmitted+4, 4); REQUIRE(sample == -0.5f);
    REQUIRE(transmitted[136] == 0 && transmitted[137] == 0 && transmitted[138] == 0x80 && transmitted[139] == 0x7f);
    now_ms = 200; MOTOR_A.Encoder = 9; Vofa_Task();
    REQUIRE(tx_count == 1 && frame.channels[0] == 1.5f);
    huart3.gState = HAL_UART_STATE_READY; Vofa_Task();
    REQUIRE(tx_count == 2 && frame.channels[0] == 9 && irq_mask == 0);
    {
        uint8_t distance_frame[14] = {0xAA,0x55,2,14,1,0,1,0,0,0,0x92,0xFE,0,0};
        uint16_t crc = Vision_Crc16(distance_frame,12);
        distance_frame[12]=(uint8_t)crc; distance_frame[13]=(uint8_t)(crc>>8);
        for(j=0;j<14;j++) Vision_ProcessByte(distance_frame[j]);
        now_ms+=50; huart3.gState=HAL_UART_STATE_READY; Vofa_Task();
        REQUIRE(NEAR(frame.channels[14],12.34f,0.001f));
        REQUIRE(frame.channels[15]==DISTANCE_VALID && frame.channels[16]==1);
        REQUIRE(frame.channels[17]==1 && frame.channels[18]==50 && frame.channels[21]==1);
        now_ms+=210; huart3.gState=HAL_UART_STATE_READY; Vofa_Task();
        REQUIRE(frame.channels[14]==0 && frame.channels[16]==0 && frame.channels[21]==1);
        REQUIRE(frame.channels[15]==DISTANCE_TIMEOUT);
    }
    case_id=4;
    receive_text("M=3;TV=0.4;TKP=2;TKD=0;TKS=0;TDF=0.1;RUN=1;");
    REQUIRE(ModeControl_GetStatus()==MODE_STATUS_WAIT_COMMAND);
    REQUIRE(MOTOR_A.Target==0 && MOTOR_B.Target==0);
    advance(10);
    REQUIRE(ModeControl_GetStatus()==MODE_STATUS_WAIT_COMMAND);
    {
        uint8_t f[14]={0xAA,0x55,2,14,2,0,1,0,0,0,0x24,0xFA,0,0};
        uint16_t crc=Vision_Crc16(f,12);
        TrackingOutput out;
        TrackingParameters p;
        f[12]=(uint8_t)crc;f[13]=(uint8_t)(crc>>8);
        for(j=0;j<14;j++) Vision_ProcessByte(f[j]);
        advance(10);
        REQUIRE(ModeControl_GetStatus()==MODE_STATUS_RUNNING);
        ModeControl_GetTrackingOutput(&out);
        REQUIRE(NEAR(out.p_rad,0.02f,0.0001f));
        REQUIRE(out.stanley_rad==0 && out.d_rad==0 && out.speed_mps==0.4f);
        REQUIRE(NEAR(out.steer_left_rad,-0.02f,0.0001f));
        receive_text("TV=0.81;TKP=-1;TKD=3;TKS=11;TDF=2;");
        Tracking_GetParameters(&p);
        REQUIRE(p.speed_mps==0.4f && p.kp==2 && p.kd==0 && p.stanley_k==0 && NEAR(p.filter_tau_s,0.1f,0.0001f));
        now_ms+=50;huart3.gState=HAL_UART_STATE_READY;Vofa_Task();
        REQUIRE(frame.channels[28]==0.4f && frame.channels[29]==2 && frame.channels[31]==0);
        REQUIRE(NEAR(frame.channels[23],0.02f*57.2957795f,0.0001f));
        receive_text("RUN=1;");REQUIRE(ModeControl_GetStatus()==MODE_STATUS_RUNNING);
        advance(1010);REQUIRE(ModeControl_GetStatus()==MODE_STATUS_LINK_ERROR);
        receive_text("TRESET=1;");Tracking_GetParameters(&p);
        REQUIRE(p.kp==TRACK_KP && p.speed_mps==TRACK_BASE_SPEED_MPS && !ModeControl_IsActive());
        receive_text("RUN=1;");REQUIRE(ModeControl_GetStatus()==MODE_STATUS_WAIT_COMMAND);
        receive_text("RUN=0;");REQUIRE(!ModeControl_IsActive());
    }
    finish(0);
}
