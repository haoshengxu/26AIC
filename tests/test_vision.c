/* Runs production C code, not a duplicate Python model. No physical I/O. */
#include <stdint.h>
#include <math.h>
#include "shims/stm32f4xx_hal.h"
#define __MAIN_H
#define __SYS_H
#define __SYSTEM_H
typedef uint8_t u8;
typedef struct { uint32_t CCR1, CCR2, CCR3, CCR4; } TestTimer;
typedef struct { TestTimer *Instance; } TIM_HandleTypeDef;
typedef struct { float Encoder, Motor_Pwm, Target, Velocity_KP, Velocity_KI; } Motor_parameter;
static TestTimer timer1, timer7, timer8, timer9, timer10, timer11;
#define TIM1 (&timer1)
#define TIM7 (&timer7)
#define TIM8 (&timer8)
#define TIM9 (&timer9)
#define TIM10 (&timer10)
#define TIM11 (&timer11)
static int enable_input = 1;
#define PDin(pin) enable_input
#define CONTROL_FREQUENCY 100
static volatile Motor_parameter MOTOR_A, MOTOR_B, MOTOR_C, MOTOR_D;
static volatile u8 Flag_Stop;
static volatile int Servo;
static volatile float Voltage;
static int Check;
static float Wheel_perimeter, Encoder_precision;
static uint32_t now_ms;
static GPIO_PinState user_key = GPIO_PIN_SET, buzzer;
static int left_count, right_count, send_imu;
static float simulated_yaw;
static unsigned int assertions, case_id;
uint32_t HAL_GetTick(void) { return now_ms; }
GPIO_PinState HAL_GPIO_ReadPin(void *p, uint16_t n) { (void)p; (void)n; return user_key; }
void HAL_GPIO_WritePin(void *p, uint16_t n, GPIO_PinState v) { (void)p; (void)n; buzzer=v; }
int Read_Encoder(u8 timer) { return timer == 2U ? left_count : right_count; }
void Led_Flash(uint16_t n) { (void)n; }
#include "../BALANCE/ackermann_control.c"
#include "../BALANCE/tracking_control.c"
#include "../HARDWARE/vision_command.c"
#include "../HARDWARE/distance_measurement.c"
#include "../BALANCE/balance.c"
#include "../HARDWARE/gyro6.c"
#include "../BALANCE/mode_control.c"

static void finish(unsigned int line)
{
    volatile uint32_t *mailbox = (volatile uint32_t *)0x2001FFF0U;
    mailbox[0]=assertions; mailbox[1]=line; mailbox[2]=case_id; mailbox[3]=0xACCE5501U;
    __asm { BKPT 0 }
    for (;;) {}
}
#define REQUIRE(x) do { ++assertions; if (!(x)) finish(__LINE__); } while (0)
#define NEAR(a,b) (fabsf((a)-(b)) < 0.0001f)
static void yaw_frame(float yaw)
{
    uint8_t f[11]={0x5A,0xBB,0,0,0,0,0,0,0,0,0};
    int16_t raw=(int16_t)(yaw*32768.0f/180.0f);
    int i;
    f[6]=(uint8_t)raw; f[7]=(uint8_t)((uint16_t)raw>>8);
    for(i=0;i<10;i++) f[10]+=f[i];
    for(i=0;i<11;i++) Gyro6_ProcessByte(f[i]);
}
static void advance(unsigned int ms)
{
    TIM_HandleTypeDef t={TIM7};
    unsigned int i;
    for(i=0;i<ms;i+=10U) {
        now_ms+=10U;
        if(send_imu) yaw_frame(simulated_yaw);
        Balance_TimerCallback(&t);
    }
}
static void reset(void)
{
    now_ms=0U; simulated_yaw=0.0f; send_imu=1;
    s_valid=0; s_frame_index=0; user_key=GPIO_PIN_SET;
    enable_input=1; Voltage=12.0f; Check=0; left_count=right_count=0;
    Encoder_precision=4.0f*ACK_ENCODER_LINES*ACK_GEAR_RATIO;
    Wheel_perimeter=ACK_WHEEL_DIAMETER_M*3.14159265359f;
    Vision_Init(); yaw_frame(0.0f); ModeControl_Init();
}
static void make(uint8_t *f, uint16_t seq, int16_t v, int16_t s, uint8_t flags)
{
    uint16_t crc;
    f[0]=0xAA;f[1]=0x55;f[2]=1;f[3]=14;
    f[4]=(uint8_t)seq;f[5]=(uint8_t)(seq>>8);
    f[6]=flags;f[7]=0;
    f[8]=(uint8_t)v;f[9]=(uint8_t)((uint16_t)v>>8);
    f[10]=(uint8_t)s;f[11]=(uint8_t)((uint16_t)s>>8);
    crc=Vision_Crc16(f,12); f[12]=(uint8_t)crc;f[13]=(uint8_t)(crc>>8);
}
static void feed(uint16_t seq, int16_t v, int16_t s, uint8_t flags)
{
    uint8_t f[14],i;make(f,seq,v,s,flags);
    for(i=0;i<14;i++) Vision_ProcessByte(f[i]);
}
static void feed_wire_distance(uint16_t seq, int16_t raw, uint8_t valid)
{
    uint8_t f[14],j;
    uint16_t crc;
    make(f,seq,0,raw,valid); f[2]=2;
    crc=Vision_Crc16(f,12); f[12]=(uint8_t)crc; f[13]=(uint8_t)(crc>>8);
    for(j=0;j<14;j++) Vision_ProcessByte(f[j]);
}
/* Scenarios specify desired center-relative error in original polarity. */
static void feed_distance(uint16_t seq, int16_t relative, uint8_t valid)
{
    int32_t wire = valid ? -2000L + relative : 0;
    if (wire < -32768L) wire = -32768L;
    if (wire > 32767L) wire = 32767L;
    feed_wire_distance(seq, (int16_t)wire, valid);
}
static void feed_target(uint16_t seq, int16_t x, int16_t y, uint8_t valid)
{
    uint8_t f[14],j; uint16_t crc;
    make(f,seq,x,y,valid); f[2]=3;
    crc=Vision_Crc16(f,12);f[12]=(uint8_t)crc;f[13]=(uint8_t)(crc>>8);
    for(j=0;j<14;j++) Vision_ProcessByte(f[j]);
}
static void press(void) { user_key=GPIO_PIN_RESET;advance(60);user_key=GPIO_PIN_SET;advance(100); }
static void single(void) { press();advance(310); }
static void double_click(void) { press();press();advance(310); }
static void start_m3(void) { reset();s_mode=MODE_CONTROL_VISION_LINE;StartTask(); }
void test_entry(void)
{
    VisionCommand c;
    uint8_t f[14],i;
    float l,r;
    case_id=1;reset();
    REQUIRE(Vision_Crc16((const uint8_t *)"123456789",9)==0x29B1);
    feed(0,200,-100,3);Vision_Get(&c);
    REQUIRE(c.accepted==1 && c.sequence==0 && c.speed_mmps==200 && c.steer_mrad==-100);
    REQUIRE(Vision_CanRun(&c,now_ms));
    case_id=2;
    feed(0,100,0,3);feed(65535,100,0,3);Vision_Get(&c);
    REQUIRE(c.accepted==1 && c.sequence_errors==2);
    make(f,1,200,0,3);f[13]^=1;
    for(i=0;i<14;i++) Vision_ProcessByte(f[i]);
    Vision_Get(&c); REQUIRE(c.crc_errors==1 && c.accepted==1);
    feed(1,251,0,3);feed(1,200,351,3);feed(1,-1,0,3);
    Vision_Get(&c); REQUIRE(c.range_errors==3 && c.accepted==1);
    case_id=3;
    Vision_Init();feed(65535,0,0,3);feed(0,200,100,3);Vision_Get(&c);
    REQUIRE(c.accepted==2 && c.sequence_errors==0);
    now_ms+=201;feed(0,100,0,3);Vision_Get(&c);
    REQUIRE(c.accepted==3); /* Restart session after stale link. */
    case_id=4;Vision_Init();
    make(f,1,200,100,3);
    for(i=0;i<5;i++) Vision_ProcessByte(f[i]);
    Vision_Get(&c);REQUIRE(!c.valid);
    for(i=5;i<14;i++) Vision_ProcessByte(f[i]);
    Vision_Get(&c);REQUIRE(c.accepted==1);
    Vision_ProcessByte(0xAA);Vision_ProcessByte(0x55);Vision_ProcessByte(0x11);
    feed(2,200,-100,3);feed(3,200,-100,3);
    Vision_Get(&c);REQUIRE(c.sequence==3 && c.steer_mrad==-100);
    case_id=5;reset();
    double_click();REQUIRE(s_mode==MODE_CONTROL_TURN_90 && !ModeControl_IsActive());
    double_click();REQUIRE(s_mode==MODE_CONTROL_VISION_LINE && !ModeControl_IsActive());
    single();REQUIRE(s_status==MODE_STATUS_WAIT_COMMAND && MOTOR_A.Target==0);
    feed_distance(1,0,1);advance(10);REQUIRE(s_status==MODE_STATUS_WAIT_COMMAND);
    feed_target(0,0,4500,1);advance(10);
    REQUIRE(s_status==MODE_STATUS_RUNNING && NEAR(MOTOR_A.Target,TRACK_BASE_SPEED_MPS));
    case_id=6;start_m3();feed_target(1,500,4500,1);advance(10);
    REQUIRE(MOTOR_A.Target>MOTOR_B.Target && Servo>SERVO_INIT);
    Ack_Kinematics(TRACK_BASE_SPEED_MPS,atanf(-2*0.145f*0.05f/(0.05f*0.05f+0.45f*0.45f)),&l,&r);
    REQUIRE(NEAR(MOTOR_A.Target,l) && NEAR(MOTOR_B.Target,r));
    feed_target(2,-500,4500,1);advance(10);REQUIRE(MOTOR_A.Target<MOTOR_B.Target && Servo<SERVO_INIT);
    case_id=7;advance(300);REQUIRE(s_status==MODE_STATUS_LINK_ERROR && MOTOR_A.Target==0 && Servo==SERVO_INIT);
    feed_target(3,0,4500,1);advance(10);REQUIRE(!ModeControl_IsActive());
    case_id=8;start_m3();feed_target(1,0,4500,1);advance(10);
    feed_target(2,0,0,0);feed_target(3,0,4500,1);advance(10);
    REQUIRE(s_status==MODE_STATUS_LINK_ERROR); /* Invalid event cannot be hidden by next good frame. */
    case_id=9;start_m3();feed_target(1,0,4500,1);advance(10);
    Vision_UartError();feed_target(2,0,4500,1);advance(10);REQUIRE(!ModeControl_IsActive());
    case_id=10;start_m3();feed_target(1,0,4500,1);advance(10);
    feed(1,0,0,4);advance(10);REQUIRE(s_status==MODE_STATUS_REMOTE_STOP);
    case_id=11;start_m3();feed_target(1,0,4500,1);advance(10);
    enable_input=0;advance(10);REQUIRE(s_status==MODE_STATUS_POWER_ERROR);
    case_id=12;start_m3();feed_target(1,0,4500,1);advance(10);
    Voltage=9;advance(10);REQUIRE(s_status==MODE_STATUS_POWER_ERROR);
    case_id=13;start_m3();send_imu=0;
    for(i=0;i<102;i++){feed_target(i,0,4500,1);advance(10);}
    REQUIRE(s_status==MODE_STATUS_RUNNING && !Gyro6_IsValid());
    case_id=14;start_m3();advance(1010);REQUIRE(s_status==MODE_STATUS_LINK_ERROR);
    case_id=15;reset();feed_target(1,0,4500,1);s_mode=MODE_CONTROL_VISION_LINE;StartTask();advance(10);
    REQUIRE(s_status==MODE_STATUS_WAIT_COMMAND);
    feed_target(2,0,4500,1);advance(10);REQUIRE(s_status==MODE_STATUS_RUNNING);
    case_id=16;reset();REQUIRE(Encoder_precision==60000.0f);
    left_count=881;right_count=-881;Get_Velocity_Form_Encoder();REQUIRE(fabsf(MOTOR_A.Encoder-0.3f)<0.001f);
    case_id=17;reset();StartTask();advance(5010);REQUIRE(s_status==MODE_STATUS_DONE);
    case_id=18;reset();s_mode=MODE_CONTROL_TURN_90;StartTask();simulated_yaw=89.5f;advance(10);REQUIRE(s_status==MODE_STATUS_DONE);
    case_id=19;start_m3();feed_target(1,1,1,1);advance(10);REQUIRE(s_status==MODE_STATUS_CONTROL_ERROR);
    case_id=20;start_m3();feed_target(65535,0,4500,1);advance(10);
    feed_target(0,0,4500,1);advance(10);REQUIRE(s_status==MODE_STATUS_RUNNING);
    case_id=21;reset();
    {
        DistanceMeasurement d;
        TrackingInput input;
        feed_wire_distance(1,-1800,1);Distance_Get(&d);Tracking_GetInput(&input);
        REQUIRE(d.distance_raw==200 && input.distance_raw==200);
        feed_wire_distance(2,-2200,1);Distance_Get(&d);Tracking_GetInput(&input);
        REQUIRE(d.distance_raw==-200 && input.distance_raw==-200);
        feed_wire_distance(3,-32768,1);Distance_Get(&d);Tracking_GetInput(&input);
        REQUIRE(d.distance_raw==-30768 && input.distance_raw==-30768);
        feed_wire_distance(4,32767,1);Distance_Get(&d);
        REQUIRE(d.distance_raw==32767);
        feed_wire_distance(5,-2000,1);Distance_Get(&d);
        REQUIRE(d.distance_raw==0 && d.measurement_valid);
        feed_wire_distance(6,0,1);Distance_Get(&d);
        REQUIRE(d.distance_raw==2000 && d.measurement_valid);
        feed_wire_distance(7,0,0);Distance_Get(&d);Tracking_GetInput(&input);
        REQUIRE(d.distance_raw==0 && !d.measurement_valid && input.valid);
    }
    case_id=22;reset();
    {
        static const uint8_t a[14]={0xAA,0x55,3,14,0,0,1,0,0xF4,1,0x94,0x11,0xF7,0xF6};
        static const uint8_t b[14]={0xAA,0x55,3,14,1,0,1,0,0x52,0xFB,0x95,0x11,0x69,0x6B};
        static const uint8_t c[14]={0xAA,0x55,3,14,2,0,0,0,0,0,0,0,0x42,0x88};
        uint8_t f[14]; uint16_t crc; unsigned int j;
        VisionTarget t; VisionCommand cmd; TrackingInput input;
        REQUIRE(Vision_Crc16((const uint8_t *)"123456789",9)==0x29B1);
        for(j=0;j<7;j++) Vision_ProcessByte(a[j]);
        Vision_GetTarget(&t); REQUIRE(!t.has_frame);
        for(j=7;j<14;j++) Vision_ProcessByte(a[j]);
        Vision_GetTarget(&t); REQUIRE(t.valid && t.x_centi_cm==500 && t.y_centi_cm==4500);
        for(j=0;j<14;j++) Vision_ProcessByte(b[j]);
        Vision_GetTarget(&t); REQUIRE(t.valid && t.x_centi_cm==-1198 && t.y_centi_cm==4501);
        for(j=0;j<14;j++) Vision_ProcessByte(c[j]);
        Vision_GetTarget(&t); REQUIRE(!t.valid && t.link_alive && !t.x_centi_cm && !t.y_centi_cm);
        /* Sequence restart and duplicates accepted, wrap is consecutive. */
        for(j=0;j<14;j++) Vision_ProcessByte(a[j]);
        for(j=0;j<14;j++) Vision_ProcessByte(a[j]);
        Vision_GetTarget(&t); REQUIRE(t.valid && t.duplicate_frames==1 && t.sequence_discontinuities==1);
        now_ms+=299; Vision_GetTarget(&t); REQUIRE(t.valid);
        now_ms++; Vision_GetTarget(&t); REQUIRE(!t.valid && !t.link_alive && t.timeouts==1);
        Vision_GetTarget(&t); REQUIRE(t.timeouts==1);
        /* Payload AA55 must not split a frame, and int16 is not angle-limited. */
        memcpy(f,a,14);f[4]=255;f[5]=255;f[8]=0xAA;f[9]=0x55;
        crc=Vision_Crc16(f,12);f[12]=(uint8_t)crc;f[13]=(uint8_t)(crc>>8);
        for(j=0;j<14;j++) Vision_ProcessByte(f[j]);
        for(j=0;j<14;j++) Vision_ProcessByte(a[j]);
        Vision_GetTarget(&t); REQUIRE(t.sequence==0 && t.valid && t.sequence_discontinuities==2);
        /* Reject reserved, invalid nonzero XY, and nonpositive forward Y. */
        for(i=0;i<4;i++) {
            memcpy(f,a,14);
            if(i==0) f[7]=1;
            if(i==1) f[6]=0;
            if(i==2) f[10]=f[11]=0;
            if(i==3) f[6]=2;
            crc=Vision_Crc16(f,12);f[12]=(uint8_t)crc;f[13]=(uint8_t)(crc>>8);
            for(j=0;j<14;j++) Vision_ProcessByte(f[j]);
        }
        Vision_GetTarget(&t); REQUIRE(t.format_errors==4);
        memcpy(f,a,14);f[12]^=1;
        for(j=0;j<14;j++) Vision_ProcessByte(f[j]);
        for(j=0;j<14;j++) Vision_ProcessByte(b[j]);
        Vision_GetTarget(&t); REQUIRE(t.crc_errors==1 && t.valid && t.sequence==1);
        /* Broken length and truncated frame recover at the next valid header. */
        memcpy(f,a,14);f[3]=13;
        for(j=0;j<14;j++) Vision_ProcessByte(f[j]);
        for(j=0;j<8;j++) Vision_ProcessByte(a[j]);
        for(j=0;j<14;j++) Vision_ProcessByte(b[j]);
        Vision_GetTarget(&t); REQUIRE(t.valid && t.x_centi_cm==-1198);
        Vision_Get(&cmd);Tracking_GetInput(&input);
        REQUIRE(cmd.accepted==0 && cmd.stop_events==0 && !input.valid);
        Vision_UartError();Vision_GetTarget(&t);
        REQUIRE(!t.valid && !t.link_alive && t.uart_errors==1);
    }
    case_id=23;reset();
    {
        float angle; union {uint32_t bits;float f;} bad;
        REQUIRE(Ack_PurePursuit(0,0.408f,&angle) && angle==0);
        REQUIRE(Ack_PurePursuit(-0.15413333f,0.377769f,&angle));
        REQUIRE(fabsf(angle-atanf(0.145f/0.54f))<0.001f);
        REQUIRE(Ack_PurePursuit(0.5f,0.01f,&angle) && angle==-ACK_MAX_STEER_RAD);
        REQUIRE(!Ack_PurePursuit(0,0,&angle) && angle==0);
        REQUIRE(!Ack_PurePursuit(0,-0.5f,&angle));
        REQUIRE(!Ack_PurePursuit(0.001f,0.001f,&angle));
        bad.bits=0x7F800000;REQUIRE(!Ack_PurePursuit(bad.f,1,&angle));
        bad.bits=0x7FC00000;REQUIRE(!Ack_PurePursuit(0,bad.f,&angle));
        start_m3();feed_target(1,0,4500,1);advance(10);advance(280);
        REQUIRE(s_status==MODE_STATUS_RUNNING);advance(10);
        REQUIRE(s_status==MODE_STATUS_LINK_ERROR && MOTOR_A.Target==0);
        start_m3();feed_target(2,500,4500,1);advance(10);
        REQUIRE(ModeControl_DebugCommand("TV",0.8f));advance(10);
        REQUIRE(MOTOR_A.Target<=0.8f && MOTOR_B.Target<=0.8f);
        REQUIRE(!ModeControl_DebugCommand("TKP",2));
        REQUIRE(ModeControl_DebugCommand("RUN",0));advance(10);
        REQUIRE(s_status==MODE_STATUS_STOPPED && Servo==SERVO_INIT);
    }
    finish(0);
}
