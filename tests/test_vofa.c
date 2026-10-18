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
    REQUIRE(MOTOR_A.Target == 0.3f && MOTOR_B.Target == 0.3f && Servo == 1576);
    receive_text("L=0.");
    REQUIRE(ModeControl_DebugLeft() == 0.3f);
    receive_text("25\r\nR=-0.2;A=10;"); advance(10);
    REQUIRE(MOTOR_A.Target == 0.25f && MOTOR_B.Target == -0.2f);
    receive_text("A=29;"); advance(10);
    REQUIRE(ModeControl_DebugAngle()==29.0f && Servo==1020 && Servo_PWM==1020);
    receive_text("A=-25;"); advance(10);
    REQUIRE(ModeControl_DebugAngle()==-25.0f && Servo==2056 && Servo_PWM==2056);
    REQUIRE(!ModeControl_DebugCommand("A",29.1f) && ModeControl_DebugAngle()==-25.0f);
    REQUIRE(!ModeControl_DebugCommand("A",-25.1f) && ModeControl_DebugAngle()==-25.0f);
    REQUIRE(Ack_ServoPulse(1.0f)==Ack_ServoPulse(ACK_MAX_STEER_RAD));
    receive_text("A=10;"); advance(10);
    receive_text("A=nan;V=inf;L=12;R=2junk;BAD=1;M=2.5;=2;");
    REQUIRE(rejected == 7 && MOTOR_A.Target == 0.25f);
    receive_text("RUN=0;");
    REQUIRE(!ModeControl_IsActive() && Servo == 1576);
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
    case_id=4;
    receive_text("M=3;TV=0.4;RUN=1;");
    REQUIRE(ModeControl_GetStatus()==MODE_STATUS_WAIT_COMMAND);
    {
        uint8_t f[14]={0xAA,0x55,3,14,0,0,1,0,0xF4,1,0x94,0x11,0xF7,0xF6};
        for(j=0;j<14;j++) Vision_ProcessByte(f[j]);
        advance(10); REQUIRE(ModeControl_GetStatus()==MODE_STATUS_RUNNING);
        REQUIRE(MOTOR_A.Target>MOTOR_B.Target);
        now_ms+=50;huart3.gState=HAL_UART_STATE_READY;Vofa_Task();
        REQUIRE(frame.channels[14]==5 && frame.channels[22]==45);
        REQUIRE(frame.channels[16]==1 && frame.channels[26]==1 && frame.channels[21]==1);
        REQUIRE(frame.channels[20]<0 && frame.channels[24]<0 && frame.channels[25]<0);
        REQUIRE(frame.channels[28]==0.4f && frame.channels[29]==5);
        now_ms+=300;huart3.gState=HAL_UART_STATE_READY;Vofa_Task();
        REQUIRE(frame.channels[16]==0 && frame.channels[15]==3 && frame.channels[26]==0);
        advance(10); REQUIRE(ModeControl_GetStatus()==MODE_STATUS_RUNNING);
        REQUIRE(frame.channels[21]==1);
        advance(650);REQUIRE(ModeControl_GetStatus()==MODE_STATUS_LINK_ERROR);
    }
    case_id=5;
    receive_text("TXTH=8;TXK=0.02;TVMIN=0.12;");
    {
        TrackingParameters p;
        Tracking_GetParameters(&p);
        REQUIRE(p.turn_x_threshold_cm==8 && NEAR(p.turn_speed_gain,0.02f,0.0001f) && NEAR(p.turn_min_speed_mps,0.12f,0.0001f));
        now_ms+=50;huart3.gState=HAL_UART_STATE_READY;Vofa_Task();
        REQUIRE(frame.channels[29]==8 && frame.channels[30]==p.turn_speed_gain && frame.channels[31]==p.turn_min_speed_mps);
        REQUIRE(frame.channels[32]==0); /* Stopped output. */
        {
            float before=rejected;
            receive_text("TXTH=-1;TXK=1.1;TVMIN=0;TXK=nan;");
            REQUIRE(rejected==before+4);
        }
        receive_text("TRESET=1;");Tracking_GetParameters(&p);
        REQUIRE(p.turn_x_threshold_cm==5 && p.turn_speed_gain==0.020f && p.turn_min_speed_mps==0.27f && p.speed_mps==0.35f);
    }
    finish(0);
}
