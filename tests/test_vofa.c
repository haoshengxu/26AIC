/* Real parser/state machine/telemetry with mocked UART. */
#define test_entry original_control_tests
#include "test_control.c"
#undef test_entry
#define __USART_H__
#define HAL_UART_STATE_READY 0
#define HAL_OK 0
typedef struct { int gState; } UART_HandleTypeDef;
static UART_HandleTypeDef huart3;
static uint8_t transmitted[512];
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
    REQUIRE(transmitted_size == 376 && tx_count == 1);
    memcpy(&sample, transmitted, 4); REQUIRE(sample == 1.5f);
    memcpy(&sample, transmitted+4, 4); REQUIRE(sample == -0.5f);
    REQUIRE(transmitted[364] == 0 && transmitted[365] == 0 && transmitted[374] == 0x80 && transmitted[375] == 0x7f);
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
        REQUIRE(p.turn_x_threshold_cm==5 && p.turn_speed_gain==0.040f && p.turn_min_speed_mps==0.60f && p.speed_mps==0.90f);
    }
    case_id=6;
    receive_text("TPG=1.1;TCTH=0.9;TCK=0.1;TSINGLE=0.24;TFALL=0.19;TUNKNOWN=0.23;THOLD=0.16;TSAT=0.18;TACC=0.4;TDEC=0.9;");
    {
        uint8_t v[22]={0xAA,0x55,4,22,2,0,15,2,0xC8,0,0xA0,0x0F,0x2C,1,0x30,0x11,0,0,0,0,0x26,0x5C};
        TrackingParameters p;float before=rejected;
        Tracking_GetParameters(&p);REQUIRE(NEAR(p.pp_gain,1.1f,0.0001f) && NEAR(p.hold_speed_mps,0.16f,0.0001f));
        receive_text("TPG=2;TCK=-1;TACC=0;TDEC=6;");REQUIRE(rejected==before+4);
        receive_text("TPGS=0.9;TPGC=1.2;TGEN=1.0;TGEX=0.5;TSTART=0.6;");
        Tracking_GetParameters(&p);
        REQUIRE(NEAR(p.pp_straight_gain,0.9f,0.0001f) && NEAR(p.pp_gain,1.2f,0.0001f));
        REQUIRE(p.gain_curve_enter==1.0f && p.gain_curve_exit==0.5f);
        receive_text("M=3;RUN=1;");
        for(j=0;j<22;j++) Vision_ProcessByte(v[j]);advance(10);
        now_ms+=50;huart3.gState=HAL_UART_STATE_READY;Vofa_Task();
        REQUIRE(frame.channels[34]==4 && frame.channels[35]==2 && frame.channels[36]==1);
        REQUIRE(frame.channels[37]==1 && frame.channels[38]==3 && frame.channels[39]==44);
        REQUIRE(frame.channels[40]==1 && frame.channels[41]==0);
        REQUIRE(frame.channels[42]==TRACK_ROUTE_FALLBACK && frame.channels[43]==3);
        REQUIRE(NEAR(frame.channels[45],0.19f,0.0001f) && NEAR(frame.channels[46],2,0.0001f) && NEAR(frame.channels[47],40,0.0001f));
        REQUIRE(frame.channels[48]==p.pp_gain && frame.channels[56]==p.accel_mps2);
        REQUIRE(NEAR(frame.channels[60],0.9f,0.0001f) && frame.channels[61]==1.0f && frame.channels[62]==0.5f);
        REQUIRE(NEAR(frame.channels[63],0.9f,0.0001f) && frame.channels[64]==0);
        REQUIRE(NEAR(frame.channels[65],0.6f,0.0001f) && frame.channels[66]==1);
        REQUIRE(transmitted_size==376 && transmitted[374]==0x80 && transmitted[375]==0x7F);
        receive_text("RUN=0;");now_ms+=50;huart3.gState=HAL_UART_STATE_READY;Vofa_Task();
        REQUIRE(frame.channels[42]==0 && frame.channels[32]==0 && frame.channels[46]==0);
    }
    case_id=7;
    receive_text("TTDEC=0.5;TTDIST=12;TTWAIT=2;");
    now_ms+=50;huart3.gState=HAL_UART_STATE_READY;Vofa_Task();
    REQUIRE(frame.channels[72]==0.5f && frame.channels[73]==12 && frame.channels[74]==2);
    {float before=rejected;receive_text("TTDEC=0;TTDIST=-1;TTWAIT=0;");REQUIRE(rejected==before+3);}
    case_id=8;
    receive_text("M=4;LKP=1200;LKI=200;RKP=1300;RKI=250;");
    REQUIRE(MOTOR_A.Velocity_KP==1200 && MOTOR_A.Velocity_KI==200);
    REQUIRE(MOTOR_B.Velocity_KP==1300 && MOTOR_B.Velocity_KI==250);
    now_ms+=50;huart3.gState=HAL_UART_STATE_READY;Vofa_Task();
    REQUIRE(frame.channels[76]==1200 && frame.channels[77]==200 && frame.channels[78]==1300 && frame.channels[79]==250);
    {float before=rejected;receive_text("LKP=-1;RKI=10001;M=3;LKI=100;");REQUIRE(rejected==before+3);}
    REQUIRE(MOTOR_A.Velocity_KI==200 && MOTOR_B.Velocity_KI==250);
    case_id=9;
    receive_text("M=4;LKP=1000;LKI=100;RKP=2000;RKI=200;L=0.2;R=-0.1;RUN=1;");
    advance(10);
    now_ms+=50;huart3.gState=HAL_UART_STATE_READY;Vofa_Task();
    REQUIRE(NEAR(frame.channels[80],220,1) && NEAR(frame.channels[83],-220,1));
    REQUIRE(NEAR(frame.channels[81],200,0.001f) && NEAR(frame.channels[82],20,0.001f));
    REQUIRE(NEAR(frame.channels[84],-200,0.001f) && NEAR(frame.channels[85],-20,0.001f));
    memcpy(&sample,transmitted+80*4,4);REQUIRE(sample==MOTOR_A.Motor_Pwm);
    memcpy(&sample,transmitted+85*4,4);REQUIRE(NEAR(sample,-20,0.001f));
    REQUIRE(transmitted_size==376 && transmitted[374]==0x80 && transmitted[375]==0x7F);
    advance(10);now_ms+=50;huart3.gState=HAL_UART_STATE_READY;Vofa_Task();
    REQUIRE(NEAR(frame.channels[81],200,0.001f) && NEAR(frame.channels[82],40,0.001f));
    REQUIRE(NEAR(frame.channels[84],-200,0.001f) && NEAR(frame.channels[85],-40,0.001f));
    REQUIRE(NEAR(frame.channels[80],240,1) && NEAR(frame.channels[83],-240,1));
    receive_text("LKI=0;");
    REQUIRE(s_left_pi.integral==0 && s_left_pi.output==0);
    REQUIRE(s_right_pi.integral!=0 && s_right_pi.output!=0);
    receive_text("RUN=0;");now_ms+=50;huart3.gState=HAL_UART_STATE_READY;Vofa_Task();
    for(j=80;j<86;j++) REQUIRE(frame.channels[j]==0);
    case_id=10;
    receive_text("M=4;LKP=12673;RKP=30000;");
    REQUIRE(MOTOR_A.Velocity_KP==12673 && MOTOR_B.Velocity_KP==30000);
    now_ms+=50;huart3.gState=HAL_UART_STATE_READY;Vofa_Task();
    REQUIRE(frame.channels[76]==12673 && frame.channels[78]==30000);
    receive_text("LKP=100000;RKP=100000.00;");
    REQUIRE(MOTOR_A.Velocity_KP==100000 && MOTOR_B.Velocity_KP==100000);
    {float before=rejected;
     receive_text("LKP=100001;RKP=100000.5;LKI=10001;RKI=10001;LKP=-1;");
     REQUIRE(rejected==before+5);
     REQUIRE(MOTOR_A.Velocity_KP==100000 && MOTOR_B.Velocity_KP==100000);
    }
    receive_text("LKP=12345.5;");REQUIRE(MOTOR_A.Velocity_KP==12345.5f);
    /* Other command limits still apply after lifting the numeric parser ceiling. */
    {float before=rejected;receive_text("V=10001;TV=10001;");REQUIRE(rejected==before+2);}
    {
        float before=rejected;
        receive_text("TBA=-12;TBV=0.65;TBT=1.5;");
        now_ms+=50;huart3.gState=HAL_UART_STATE_READY;Vofa_Task();
        REQUIRE(frame.channels[86]==-12 && NEAR(frame.channels[87],0.65f,0.0001f) && frame.channels[88]==1.5f);
        REQUIRE(frame.channels[89]==0 && frame.channels[90]==0);
        receive_text("TBA=21;TBA=-21;TBV=0;TBV=1.601;TBT=0;TBT=5.1;");
        REQUIRE(rejected==before+6);
    }
    finish(0);
}
