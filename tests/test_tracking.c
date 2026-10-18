/* Production controller and full 100 Hz mode/motor chain, hardware mocked. */
#define test_entry other_tests_entry
#include "test_vision.c"
#undef test_entry

static void solve(TrackingController *controller, TrackingOutput *out,
                  int16_t raw, uint16_t seq, float speed, uint8_t imu, float yaw)
{
    TrackingInput in;
    feed_distance(seq,raw,1);
    Tracking_GetInput(&in);
    REQUIRE(Tracking_Update(controller,&in,now_ms,speed,imu,yaw,out));
}

static void tracking_advance(unsigned int ms)
{
    static uint16_t test_sequence=1000;
    unsigned int j;
    for(j=0;j<ms;j+=10) {feed_distance(test_sequence++,100,1);advance(10);}
}

void test_entry(void)
{
    TrackingController controller;
    TrackingOutput out;
    TrackingInput in;
    float expected, saved;
    unsigned int i;
    case_id=1;reset(); Tracking_Reset(&controller,&out);
    solve(&controller,&out,200,0,0.20f,1,80.0f);
    expected=5.45f*0.02f+atanf(0.6f*0.02f/0.2f);
    REQUIRE(NEAR(out.error_m,0.02f) && out.derivative_mps==0);
    REQUIRE(NEAR(out.correction_right_rad,expected));
    REQUIRE(NEAR(out.steer_left_rad,-expected) && !out.heading_used);
    REQUIRE(Ack_ServoPulse(out.steer_left_rad)>ACK_SERVO_CENTER_US);
    case_id=2; now_ms=20;
    solve(&controller,&out,300,1,0.20f,0,0);
    REQUIRE(NEAR(out.derivative_mps,0.1f)); /* .5 m/s * .02/(.08+.02). */
    REQUIRE(NEAR(out.d_rad,0.03f));
    REQUIRE(NEAR(out.steer_left_rad,-(5.45f*0.03f+0.03f+atanf(0.09f))));
    Tracking_GetInput(&in); saved=out.derivative_mps; now_ms=30;
    REQUIRE(Tracking_Update(&controller,&in,now_ms,0.20f,0,0,&out));
    REQUIRE(out.derivative_mps==saved); /* No new packet -> no repeated differentiation. */
    now_ms=40;solve(&controller,&out,300,2,0.20f,0,0);
    REQUIRE(NEAR(out.derivative_mps,0.08f));
    case_id=3;reset();Tracking_Reset(&controller,&out);
    solve(&controller,&out,-200,0,0.20f,0,0);
    REQUIRE(NEAR(out.steer_left_rad,expected));
    REQUIRE(Ack_ServoPulse(out.steer_left_rad)<ACK_SERVO_CENTER_US);
    case_id=4;reset();Tracking_Reset(&controller,&out);
    solve(&controller,&out,32767,0,0.0f,0,0);
    REQUIRE(NEAR(out.denominator_mps,0.1f));
    REQUIRE(NEAR(out.steer_left_rad,-0.35f));
    Tracking_Reset(&controller,&out);
    solve(&controller,&out,-32767,1,-0.0f,0,0);
    REQUIRE(NEAR(out.error_m,-3.1168f) && NEAR(out.steer_left_rad,0.35f));
    Tracking_Reset(&controller,&out);
    solve(&controller,&out,0,2,0,0,0); REQUIRE(out.steer_left_rad==0);
    REQUIRE(Ack_ServoPulse(out.steer_left_rad)==ACK_SERVO_CENTER_US);
    case_id=5;reset();Tracking_Reset(&controller,&out);
    Tracking_SetHeadingReference(-179.0f,1);
    solve(&controller,&out,0,0,0.20f,1,179.0f);
    REQUIRE(out.heading_used && NEAR(out.steer_left_rad,2.0f*0.01745329252f));
    now_ms=20;solve(&controller,&out,0,1,0.20f,0,179.0f);
    REQUIRE(!out.heading_used && out.steer_left_rad==0);
    now_ms=201;solve(&controller,&out,0,2,0.20f,1,179.0f);
    REQUIRE(!out.heading_used && out.steer_left_rad==0); /* Reference expired. */
    Tracking_SetHeadingReference(999.0f,1);
    now_ms=220;solve(&controller,&out,0,3,0.20f,1,0);
    REQUIRE(!out.heading_used);
    case_id=6;reset();Tracking_Reset(&controller,&out);
    solve(&controller,&out,0,0,0.20f,0,0);
    now_ms=100;solve(&controller,&out,100,1,0.20f,0,0);
    REQUIRE(NEAR(out.derivative_mps,0.1f*(0.1f/0.18f)));
    now_ms=101;solve(&controller,&out,30000,2,0.20f,0,0);
    REQUIRE(fabsf(out.derivative_mps)<=TRACK_D_LIMIT_MPS && fabsf(out.steer_left_rad)<=0.35f);
    Tracking_Reset(&controller,&out);
    now_ms=102;solve(&controller,&out,-30000,3,0.20f,0,0);
    REQUIRE(out.derivative_mps==0); /* Rearm/reset never produces a derivative kick. */
    case_id=7;reset();Tracking_Reset(&controller,&out);
    now_ms=0xFFFFFFF0U;solve(&controller,&out,0,65535,0.20f,0,0);
    now_ms+=20;solve(&controller,&out,100,0,0.20f,0,0);
    REQUIRE(NEAR(out.derivative_mps,0.1f)); /* Tick and sequence wrap. */
    Tracking_GetInput(&in);
    REQUIRE(Tracking_CanRun(&in,now_ms+999));
    REQUIRE(!Tracking_CanRun(&in,now_ms+1000));
    case_id=8;start_m3();feed_distance(1,222,1);advance(10);
    REQUIRE(s_status==MODE_STATUS_RUNNING && MOTOR_A.Target>MOTOR_B.Target);
    REQUIRE(MOTOR_A.Motor_Pwm!=0 && MOTOR_B.Motor_Pwm!=0); /* Existing wheel PI still used. */
    for(i=0;i<98;i++) advance(10);
    REQUIRE(s_status==MODE_STATUS_RUNNING); /* age 990 ms: still holding the previous sample */
    advance(10); REQUIRE(s_status==MODE_STATUS_LINK_ERROR && Flag_Stop);
    feed_distance(2,222,1);advance(10);REQUIRE(!ModeControl_IsActive());
    StartTask();advance(10);REQUIRE(s_status==MODE_STATUS_WAIT_COMMAND);
    feed_distance(3,-222,1);advance(10); REQUIRE(s_status==MODE_STATUS_RUNNING);
    REQUIRE(MOTOR_A.Target<MOTOR_B.Target);
    ModeControl_GetTrackingOutput(&out); REQUIRE(out.derivative_mps==0);
    case_id=9;start_m3();feed_distance(1,100,1);advance(10);
    feed_distance(2,0,0);feed_distance(3,100,1);advance(10);
    REQUIRE(s_status==MODE_STATUS_RUNNING && !Flag_Stop && MOTOR_A.Target>0);
    ModeControl_GetTrackingOutput(&out);REQUIRE(out.valid && out.steer_left_rad!=0);
    StartTask();feed_distance(4,100,1);advance(10);REQUIRE(s_status==MODE_STATUS_RUNNING);
    Vision_UartError();feed_distance(5,100,1);advance(10);REQUIRE(s_status==MODE_STATUS_REMOTE_STOP);
    case_id=10;start_m3();feed_distance(1,100,1);advance(10);
    feed(1,0,0,4);feed_distance(2,100,1);advance(10);
    REQUIRE(s_status==MODE_STATUS_REMOTE_STOP); /* Legacy STOP retained. */
    start_m3();feed_distance(1,100,1);advance(10);
    for(i=0;i<102;i++){feed((uint16_t)(i+1),200,0,3);advance(10);}
    REQUIRE(s_status==MODE_STATUS_LINK_ERROR); /* V1 packets cannot renew V2 heartbeat. */
    case_id=11;start_m3();feed_distance(1,100,1);advance(10);
    Check=1;advance(10); REQUIRE(s_status==MODE_STATUS_POWER_ERROR && Flag_Stop);
    start_m3();send_imu=0;
    for(i=0;i<40;i++){feed_distance((uint16_t)(i+1),100,1);advance(10);}
    REQUIRE(!Gyro6_IsValid() && s_status==MODE_STATUS_RUNNING);
    ModeControl_GetTrackingOutput(&out);REQUIRE(!out.heading_used);
    /* Switch to M4 stops; its manual start is independent of vision. */
    double_click();REQUIRE(s_mode==MODE_CONTROL_DEBUG && Flag_Stop);
    feed_distance(99,1000,1);advance(10);REQUIRE(!ModeControl_IsActive());
    single();REQUIRE(ModeControl_IsActive());
    REQUIRE(NEAR(MOTOR_A.Target,0.3f) && NEAR(MOTOR_B.Target,0.3f));
    Vision_UartError();advance(10);REQUIRE(ModeControl_IsActive());
    single();REQUIRE(!ModeControl_IsActive());
    case_id=12;start_m3();feed_distance(1,100,1);advance(10);
    saved=MOTOR_A.Target;
    feed_distance(1,-30000,1);advance(10);
    REQUIRE(NEAR(MOTOR_A.Target,saved)); /* Duplicate cannot change steering. */
    for(i=0;i<102;i++){feed_distance(1,100,1);advance(10);}
    REQUIRE(s_status==MODE_STATUS_LINK_ERROR);
    Tracking_GetInput(&in); REQUIRE(in.sequence_errors>=20 && in.frames==2);
    /* After stale link, a restarted sequence can be received, but cannot rearm. */
    /* Zero valid error still runs; first valid packet does not require IMU. */
    start_m3();send_imu=0;s_valid=0;feed_distance(1,0,1);advance(10);
    REQUIRE(s_status==MODE_STATUS_RUNNING && Servo==SERVO_INIT);
    REQUIRE(NEAR(MOTOR_A.Target,TRACK_BASE_SPEED_MPS) && NEAR(MOTOR_B.Target,TRACK_BASE_SPEED_MPS));
    case_id=13;start_m3();tracking_advance(10);
    REQUIRE(s_status==MODE_STATUS_RUNNING);
    user_key=GPIO_PIN_RESET;tracking_advance(60);
    user_key=GPIO_PIN_SET;tracking_advance(450);
    REQUIRE(s_status==MODE_STATUS_STOPPED && Flag_Stop && MOTOR_A.Target==0);
    tracking_advance(100);REQUIRE(s_status==MODE_STATUS_STOPPED);
    StartTask();tracking_advance(10);REQUIRE(s_status==MODE_STATUS_RUNNING);
    user_key=GPIO_PIN_RESET;tracking_advance(600);
    REQUIRE(s_status==MODE_STATUS_STOPPED && Flag_Stop);
    user_key=GPIO_PIN_SET;tracking_advance(450);REQUIRE(s_status==MODE_STATUS_STOPPED);
    case_id=14;start_m3();feed_distance(1,100,1);advance(10);
    {
        uint8_t f[14],j;
        uint16_t crc;
        make(f,2,0,100,1);f[2]=2;crc=Vision_Crc16(f,12);
        f[12]=(uint8_t)(crc^1U);f[13]=(uint8_t)(crc>>8);
        for(i=0;i<102;i++) {for(j=0;j<14;j++) Vision_ProcessByte(f[j]);advance(10);}
    }
    REQUIRE(s_status==MODE_STATUS_LINK_ERROR); /* Bad CRC never refreshes heartbeat. */
    case_id=16;start_m3();feed_distance(1,100,1);advance(10);
    saved=MOTOR_A.Target;
    advance(970);
    REQUIRE(s_status==MODE_STATUS_RUNNING && NEAR(MOTOR_A.Target,saved));
    feed_distance(2,-100,1);advance(10);
    REQUIRE(s_status==MODE_STATUS_RUNNING && MOTOR_A.Target<MOTOR_B.Target);
    advance(980);REQUIRE(s_status==MODE_STATUS_RUNNING);
    advance(20);REQUIRE(s_status==MODE_STATUS_LINK_ERROR && MOTOR_A.Target==0);
    case_id=17;start_m3();feed_distance(1,100,1);advance(10);
    for(i=0;i<98;i++){feed_distance((uint16_t)(i+2),0,0);advance(10);}
    REQUIRE(s_status==MODE_STATUS_RUNNING);
    advance(10);REQUIRE(s_status==MODE_STATUS_LINK_ERROR && MOTOR_A.Target==0);
    case_id=18;reset();Tracking_Reset(&controller,&out);
    feed_distance(1,100,1);Tracking_GetInput(&in);
    {
        union {uint32_t bits;float number;} nonfinite;
        nonfinite.bits=0x7FC00000U;
        REQUIRE(!Tracking_Update(&controller,&in,now_ms,nonfinite.number,0,0,&out));
        REQUIRE(!out.valid && out.speed_mps==0 && out.steer_left_rad==0);
    }
    finish(0);
}
