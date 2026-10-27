/* V4 production controller through the real parser and 100 Hz motor chain. */
#define test_entry previous_vision_tests
#include "test_vision.c"
#undef test_entry

static uint16_t route_seq;
static void route_frame(int16_t x, int16_t y, uint8_t flags, uint8_t mode, float k_cm)
{
    uint8_t v[22]; uint32_t bits;
    memcpy(v,v4_examples[0],22);
    v[4]=(uint8_t)route_seq;v[5]=(uint8_t)(route_seq>>8);route_seq++;
    v[6]=flags;v[7]=mode;
    v[8]=(uint8_t)x;v[9]=(uint8_t)((uint16_t)x>>8);
    v[10]=(uint8_t)y;v[11]=(uint8_t)((uint16_t)y>>8);
    memcpy(&bits,&k_cm,4);
    v[16]=(uint8_t)bits;v[17]=(uint8_t)(bits>>8);
    v[18]=(uint8_t)(bits>>16);v[19]=(uint8_t)(bits>>24);
    send_v4(v);
}
static void route_run(unsigned int ticks, int16_t x, int16_t y, uint8_t flags, uint8_t mode, float k)
{
    unsigned int j;
    for(j=0;j<ticks;j++){route_frame(x,y,flags,mode,k);advance(10);}
}
/* Fixed calibration fixture keeps behavior tests independent of tuned defaults. */
static void start_route_fixture(void)
{
    start_m3();
    ModeControl_DebugCommand("TCK",0.08f);
    ModeControl_DebugCommand("TSINGLE",0.25f);
    ModeControl_DebugCommand("TFALL",0.20f);
    ModeControl_DebugCommand("TUNKNOWN",0.25f);
    ModeControl_DebugCommand("TSAT",0.20f);
    ModeControl_DebugCommand("TDEC",0.80f);
    ModeControl_DebugCommand("TV",0.35f);
    ModeControl_DebugCommand("TVMIN",0.27f);
    ModeControl_DebugCommand("TPG",1.0f);
    ModeControl_DebugCommand("TCTH",0.8f);
    ModeControl_DebugCommand("THOLD",0.18f);
    ModeControl_DebugCommand("TACC",0.30f);
    ModeControl_DebugCommand("TSTART",0.30f);
    ModeControl_DebugCommand("TGEN",0.82f);
    ModeControl_DebugCommand("TGEX",0.60f); ModeControl_DebugCommand("TTWAIT",1.0f);
    ModeControl_DebugCommand("TEDIST",10000.0f); /* Keep the route fixture outside the new zone. */
    ModeControl_DebugCommand("TE3S",10000.0f);
}
static void tape_frame_mode(uint16_t cm100,uint8_t detected,uint8_t mode)
{
    uint8_t v[25];uint16_t crc;
    memcpy(v,v4_examples[0],20);v[2]=5;v[3]=25;
    v[7]=mode;
    v[4]=(uint8_t)route_seq;v[5]=(uint8_t)(route_seq++>>8);
    v[8]=v[9]=0;memset(v+16,0,4);v[20]=detected;
    v[21]=(uint8_t)cm100;v[22]=(uint8_t)(cm100>>8);
    crc=Vision_Crc16(v,23);v[23]=(uint8_t)crc;v[24]=(uint8_t)(crc>>8);
    send_bytes(v,25);
}
static void tape_frame(uint16_t cm100,uint8_t detected)
{ tape_frame_mode(cm100,detected,1); }
void test_entry(void)
{
    TrackingOutput out;TrackingParameters p;float angle,speed,request;
    case_id=1;start_route_fixture();
    route_frame(500,4500,7,1,0.02f);advance(10);ModeControl_GetTrackingOutput(&out);
    REQUIRE(s_status==MODE_STATUS_RUNNING && out.route_state==TRACK_ROUTE_NORMAL);
    REQUIRE(NEAR(out.speed_curvature_m_inv,2.0f) && NEAR(out.requested_speed_mps,0.27f));
    REQUIRE(NEAR(out.speed_mps,0.003f) && MOTOR_A.Target>MOTOR_B.Target);
    REQUIRE(Ack_PurePursuit(0.05f,0.45f,&angle) && NEAR(out.steer_left_rad,angle));
    route_run(110,500,4500,7,1,0.02f);ModeControl_GetTrackingOutput(&out);
    REQUIRE(NEAR(out.speed_mps,0.27f));

    case_id=2;start_route_fixture();
    route_run(130,0,4500,7,1,0);ModeControl_GetTrackingOutput(&out);
    REQUIRE(out.route_state==TRACK_ROUTE_NORMAL && out.route_reasons==0 && out.steer_left_rad==0);
    REQUIRE(NEAR(out.speed_mps,0.35f)); /* valid zero curvature is not unknown */
    route_frame(0,4500,3,1,0);advance(10);ModeControl_GetTrackingOutput(&out);
    REQUIRE(out.route_state==TRACK_ROUTE_UNKNOWN && NEAR(out.requested_speed_mps,0.25f));
    REQUIRE(NEAR(out.speed_mps,0.342f)); /* bounded deceleration */
    route_run(30,0,4500,7,2,0);ModeControl_GetTrackingOutput(&out);
    REQUIRE(out.route_state==TRACK_ROUTE_SINGLE && NEAR(out.speed_mps,0.25f));
    route_run(20,0,4500,15,2,0);ModeControl_GetTrackingOutput(&out);
    REQUIRE(out.route_state==TRACK_ROUTE_FALLBACK && NEAR(out.speed_mps,0.20f));
    REQUIRE(out.route_reasons==(TRACK_REASON_SINGLE|TRACK_REASON_FALLBACK));
    route_frame(0,4500,11,3,0);advance(10);ModeControl_GetTrackingOutput(&out);
    REQUIRE(out.route_reasons==(TRACK_REASON_SINGLE|TRACK_REASON_FALLBACK|TRACK_REASON_UNKNOWN));
    REQUIRE(NEAR(out.requested_speed_mps,0.20f));

    case_id=3;start_route_fixture();
    REQUIRE(ModeControl_DebugCommand("TVMIN",0.1f));
    /* Two target distances on the same R=.54m circle yield the same curvature.
     * Different X no longer changes speed merely because lookahead changed. */
    route_frame(1541,3778,7,1,0.0185f);advance(10);ModeControl_GetTrackingOutput(&out);
    request=out.requested_speed_mps;angle=out.steer_left_rad;
    route_frame(910,3000,7,1,0.0185f);advance(10);ModeControl_GetTrackingOutput(&out);
    REQUIRE(fabsf(out.requested_speed_mps-request)<0.001f && fabsf(out.steer_left_rad-angle)<0.001f);
    REQUIRE(ModeControl_DebugCommand("TXK",1));
    route_frame(910,3000,7,1,0.0185f);advance(10);ModeControl_GetTrackingOutput(&out);
    REQUIRE(fabsf(out.requested_speed_mps-request)<0.001f); /* V4 ignores old X gain */

    case_id=4;start_route_fixture();
    route_frame(500,4500,7,1,0);advance(10);ModeControl_GetTrackingOutput(&out);angle=out.steer_left_rad;
    REQUIRE(ModeControl_DebugCommand("TPG",1.1f));
    route_run(10,500,4500,7,1,0);ModeControl_GetTrackingOutput(&out);
    REQUIRE(NEAR(tanf(out.steer_left_rad),1.1f*tanf(angle)));
    angle=out.steer_left_rad;
    route_frame(500,4500,7,1,0.10f);advance(10);ModeControl_GetTrackingOutput(&out);
    REQUIRE(out.steer_left_rad==angle); /* endpoint curvature never adds steering */
    route_frame(-500,4500,7,1,0.10f);advance(10);ModeControl_GetTrackingOutput(&out);
    REQUIRE(NEAR(out.steer_left_rad,-angle));
    REQUIRE(!ModeControl_DebugCommand("TPG",1.51f) && !ModeControl_DebugCommand("TPG",0.49f));

    case_id=5;start_route_fixture();
    route_run(120,500,4500,7,1,0);ModeControl_GetTrackingOutput(&out);angle=out.steer_left_rad;
    route_frame(0,0,6,3,0.03f);advance(10);ModeControl_GetTrackingOutput(&out);
    REQUIRE(out.route_state==TRACK_ROUTE_HOLD && out.steer_left_rad==angle);
    REQUIRE(NEAR(out.control_x_m,0.05f) && NEAR(out.control_y_m,0.45f));
    REQUIRE(NEAR(out.requested_speed_mps,0.18f));
    route_run(97,0,0,6,3,0.03f);ModeControl_GetTrackingOutput(&out);
    REQUIRE(s_status==MODE_STATUS_RUNNING && NEAR(out.speed_mps,0.18f));
    advance(10);REQUIRE(s_status==MODE_STATUS_LINK_ERROR && MOTOR_A.Target==0 && Servo==SERVO_INIT);
    route_frame(0,4500,7,1,0);advance(10);REQUIRE(!ModeControl_IsActive());
    /* Lost immediately after startup must not accelerate on stale data. */
    start_route_fixture();route_frame(500,4500,7,1,0);advance(10);ModeControl_GetTrackingOutput(&out);speed=out.speed_mps;
    route_run(30,0,0,6,3,0.02f);ModeControl_GetTrackingOutput(&out);
    REQUIRE(out.speed_mps<=speed && out.steer_left_rad<0);

    case_id=6;start_route_fixture();
    route_run(120,0,4500,7,1,0);advance(140);ModeControl_GetTrackingOutput(&out);
    REQUIRE(out.route_state==TRACK_ROUTE_HOLD); /* no new target for 150ms */
    speed=out.speed_mps;
    route_frame(0,4500,7,1,0);advance(10);ModeControl_GetTrackingOutput(&out);
    REQUIRE(out.route_state==TRACK_ROUTE_NORMAL && out.speed_mps<=speed+0.0031f);
    advance(980);REQUIRE(s_status==MODE_STATUS_RUNNING);
    route_frame(-500,4500,7,1,0);advance(10);REQUIRE(s_status==MODE_STATUS_RUNNING && MOTOR_A.Target<MOTOR_B.Target);
    feed(1,0,0,4);advance(10);REQUIRE(s_status==MODE_STATUS_REMOTE_STOP && MOTOR_A.Target==0);

    case_id=7;start_route_fixture();
    route_run(100,500,100,7,1,0);ModeControl_GetTrackingOutput(&out);
    REQUIRE(out.route_state==TRACK_ROUTE_SATURATED && (out.route_reasons&TRACK_REASON_SATURATED));
    REQUIRE(out.steer_left_rad==-ACK_MAX_STEER_RAD && NEAR(out.requested_speed_mps,0.20f));
    route_frame(-500,100,7,1,0);advance(10);ModeControl_GetTrackingOutput(&out);
    REQUIRE(out.steer_left_rad==ACK_MAX_STEER_RAD);
    REQUIRE(MOTOR_A.Target<=0.8f && MOTOR_B.Target<=0.8f);
    route_frame(1,1,7,1,0);advance(10);REQUIRE(s_status==MODE_STATUS_CONTROL_ERROR && MOTOR_A.Target==0);

    case_id=8;start_route_fixture();
    REQUIRE(ModeControl_DebugCommand("TV",0.12f));
    route_run(100,0,4500,15,2,0);ModeControl_GetTrackingOutput(&out);
    REQUIRE(NEAR(out.speed_mps,0.12f)); /* caps/floor cannot raise base speed */
    REQUIRE(ModeControl_DebugCommand("TFALL",0.05f));
    route_run(30,0,4500,15,2,0);ModeControl_GetTrackingOutput(&out);
    REQUIRE(NEAR(out.speed_mps,0.05f)); /* degradation cap can be below curve floor */
    {
        union {uint32_t u;float f;} huge;
        huge.u=0x7F7FFFFFU;route_frame(0,4500,7,1,huge.f);advance(10);ModeControl_GetTrackingOutput(&out);
        REQUIRE(out.speed_curvature_m_inv==100.0f && out.requested_speed_mps==0.12f);
    }
    REQUIRE(!ModeControl_DebugCommand("TACC",0) && !ModeControl_DebugCommand("TDEC",6));
    REQUIRE(!ModeControl_DebugCommand("TCTH",-1) && !ModeControl_DebugCommand("THOLD",0));

    case_id=9;start_route_fixture();
    route_run(100,500,4500,7,1,0);REQUIRE(ModeControl_DebugCommand("TPG",1.3f));
    REQUIRE(ModeControl_DebugCommand("M",4) && ModeControl_DebugCommand("RUN",1));
    advance(10);REQUIRE(NEAR(MOTOR_A.Target,ACK_DEBUG_DEFAULT_SPEED_MPS) && NEAR(MOTOR_B.Target,ACK_DEBUG_DEFAULT_SPEED_MPS));
    REQUIRE(ModeControl_DebugCommand("M",3) && ModeControl_DebugCommand("RUN",1));
    route_frame(0,4500,7,1,0);advance(10);ModeControl_GetTrackingOutput(&out);
    REQUIRE(NEAR(out.speed_mps,0.003f));
    REQUIRE(ModeControl_DebugCommand("TRESET",1));Tracking_GetParameters(&p);
    REQUIRE(p.pp_gain==1.39f && p.curve_threshold==0.76f && p.curve_speed_gain==0.16f);
    REQUIRE(p.fallback_speed_mps==0.30f && p.hold_speed_mps==0.40f && p.accel_mps2==0.395f);
    REQUIRE(p.speed_mps==0.95f && p.turn_min_speed_mps==0.60f && p.unknown_speed_mps==0.50f);
    REQUIRE(p.decel_mps2==1.60f && p.start_accel_mps2==0.43f);
    Vision_UartError();advance(10);REQUIRE(!ModeControl_IsActive() && MOTOR_A.Target==0);
    case_id=10;start_route_fixture();
    REQUIRE(ModeControl_DebugCommand("TPGS",0.8f));
    REQUIRE(ModeControl_DebugCommand("TPGC",1.3f));
    REQUIRE(!ModeControl_DebugCommand("TPGS",0.49f));
    REQUIRE(!ModeControl_DebugCommand("TPGC",1.51f));
    REQUIRE(!ModeControl_DebugCommand("TGEN",0.60f));
    REQUIRE(!ModeControl_DebugCommand("TGEX",0.82f));
    REQUIRE(ModeControl_DebugCommand("TGEN",1.0f));
    REQUIRE(ModeControl_DebugCommand("TGEX",0.5f));
    route_run(40,500,4500,7,1,0);ModeControl_GetTrackingOutput(&out);
    REQUIRE(out.gain_state==0 && NEAR(out.active_pp_gain,0.8f));
    angle=out.steer_left_rad;
    route_frame(500,4500,7,1,0.012f);advance(10);ModeControl_GetTrackingOutput(&out);
    REQUIRE(out.gain_state==0 && NEAR(out.active_pp_gain,0.8f));
    advance(30);ModeControl_GetTrackingOutput(&out);
    REQUIRE(out.gain_state==0); /* Same packet cannot count as three frames. */
    route_frame(500,4500,7,1,0.012f);advance(10);ModeControl_GetTrackingOutput(&out);
    REQUIRE(out.gain_state==0);
    route_frame(500,4500,7,1,0.007f);advance(10); /* Below entry resets confirmation. */
    route_run(2,500,4500,7,1,0.012f);ModeControl_GetTrackingOutput(&out);
    REQUIRE(out.gain_state==0);
    route_frame(500,4500,3,1,0);advance(10); /* Unknown curvature resets confirmation. */
    route_run(2,500,4500,7,1,0.012f);ModeControl_GetTrackingOutput(&out);
    REQUIRE(out.gain_state==0);
    route_frame(500,4500,7,1,0.012f);advance(10);ModeControl_GetTrackingOutput(&out);
    REQUIRE(out.gain_state==1 && out.active_pp_gain>0.8f && out.active_pp_gain<0.83f);
    route_run(40,500,4500,7,1,0.012f);ModeControl_GetTrackingOutput(&out);
    REQUIRE(NEAR(out.active_pp_gain,1.3f));
    REQUIRE(NEAR(tanf(out.steer_left_rad)/tanf(angle),1.3f/0.8f));
    route_run(10,500,4500,7,1,0.007f);ModeControl_GetTrackingOutput(&out);
    REQUIRE(out.gain_state==1 && NEAR(out.active_pp_gain,1.3f)); /* hysteresis */
    route_run(10,500,4500,3,1,0);ModeControl_GetTrackingOutput(&out);
    REQUIRE(out.gain_state==2 && NEAR(out.active_pp_gain,1.3f)); /* unknown freezes gain */
    route_frame(0,0,6,1,0);advance(10);ModeControl_GetTrackingOutput(&out);
    REQUIRE(out.gain_state==3 && NEAR(out.active_pp_gain,1.3f));
    route_run(40,500,4500,7,1,0.004f);ModeControl_GetTrackingOutput(&out);
    REQUIRE(out.gain_state==0 && NEAR(out.active_pp_gain,0.8f));
    route_run(10,500,4500,7,1,0.007f);ModeControl_GetTrackingOutput(&out);
    REQUIRE(out.gain_state==0 && NEAR(out.active_pp_gain,0.8f));
    feed_target(20,500,4500,1);advance(10);ModeControl_GetTrackingOutput(&out);
    REQUIRE(out.gain_state==4 && NEAR(out.active_pp_gain,1.3f)); /* V3 fixed gain */
    REQUIRE(ModeControl_DebugCommand("TPG",1.1f));Tracking_GetParameters(&p);
    REQUIRE(p.pp_gain==1.1f && p.pp_straight_gain==1.1f);
    REQUIRE(ModeControl_DebugCommand("TRESET",1));Tracking_GetParameters(&p);
    REQUIRE(p.pp_gain==1.39f && p.pp_straight_gain==0.91f);
    REQUIRE(p.gain_curve_enter==0.82f && p.gain_curve_exit==0.31f);
    case_id=11;start_route_fixture();
    REQUIRE(ModeControl_DebugCommand("TSTART",0.5f));
    REQUIRE(ModeControl_DebugCommand("TACC",0.1f));
    REQUIRE(ModeControl_DebugCommand("TV",0.30f));
    REQUIRE(ModeControl_DebugCommand("TVMIN",0.10f));
    REQUIRE(!ModeControl_DebugCommand("TSTART",0) && !ModeControl_DebugCommand("TSTART",3.1f));
    route_frame(0,4500,7,1,0);advance(10);ModeControl_GetTrackingOutput(&out);
    REQUIRE(out.startup_active && NEAR(out.speed_mps,0.005f));
    route_run(65,0,4500,7,1,0);ModeControl_GetTrackingOutput(&out);
    REQUIRE(!out.startup_active && NEAR(out.speed_mps,0.30f));
    REQUIRE(ModeControl_DebugCommand("TV",0.40f));
    route_frame(0,4500,7,1,0);advance(10);ModeControl_GetTrackingOutput(&out);
    REQUIRE(!out.startup_active && NEAR(out.speed_mps,0.301f));
    route_run(50,0,4500,15,1,0);ModeControl_GetTrackingOutput(&out);
    REQUIRE(!out.startup_active && NEAR(out.speed_mps,0.20f));
    route_run(20,0,0,6,1,0);ModeControl_GetTrackingOutput(&out);
    REQUIRE(!out.startup_active && out.route_state==TRACK_ROUTE_HOLD);
    speed=out.speed_mps;
    route_frame(0,4500,7,1,0);advance(10);ModeControl_GetTrackingOutput(&out);
    REQUIRE(!out.startup_active && NEAR(out.speed_mps,speed+0.001f));
    REQUIRE(ModeControl_DebugCommand("RUN",0) && ModeControl_DebugCommand("RUN",1));
    route_frame(0,4500,7,1,0);advance(10);ModeControl_GetTrackingOutput(&out);
    REQUIRE(out.startup_active && NEAR(out.speed_mps,0.005f));
    speed=out.speed_mps;
    route_run(30,0,0,6,1,0);ModeControl_GetTrackingOutput(&out);
    REQUIRE(out.startup_active && out.speed_mps<=speed); /* hold cannot complete launch */
    REQUIRE(ModeControl_DebugCommand("TRESET",1));Tracking_GetParameters(&p);
    REQUIRE(p.start_accel_mps2==0.43f && p.accel_mps2==0.395f);
    case_id=12;start_route_fixture();
    {
        uint8_t v[25]; uint16_t crc;
        memcpy(v,v4_examples[0],20);v[2]=5;v[3]=25;
        v[20]=0;v[21]=0;v[22]=0;
        crc=Vision_Crc16(v,23);v[23]=(uint8_t)crc;v[24]=(uint8_t)(crc>>8);
        send_bytes(v,25);advance(10);ModeControl_GetTrackingOutput(&out);
        REQUIRE(s_status==MODE_STATUS_RUNNING && out.route_state==TRACK_ROUTE_NORMAL);
        REQUIRE(NEAR(out.speed_curvature_m_inv,2.0f) && NEAR(out.requested_speed_mps,0.27f));
        REQUIRE(out.startup_active && NEAR(out.speed_mps,0.003f) && out.gain_state==0);
        /* A tape-only frame cannot renew the one-second motion target hold. */
        memset(v+6,0,14);v[20]=0;
        crc=Vision_Crc16(v,23);v[23]=(uint8_t)crc;v[24]=(uint8_t)(crc>>8);
        send_bytes(v,25);advance(10);ModeControl_GetTrackingOutput(&out);
        REQUIRE(out.route_state==TRACK_ROUTE_HOLD);
        advance(1000);REQUIRE(s_status==MODE_STATUS_LINK_ERROR && MOTOR_A.Target==0);
    }
    case_id=13;start_route_fixture();
    {
        unsigned int j;
        /* This handover/loss scenario intentionally spans a two-second window. */
        REQUIRE(ModeControl_DebugCommand("T1BMAX",2.0f));
        REQUIRE(ModeControl_DebugCommand("T1BMIN",0.90f));
        REQUIRE(ModeControl_DebugCommand("TTDEC",2));
        REQUIRE(ModeControl_DebugCommand("TTDIST",100));
        left_count=(int)(0.4f*Encoder_precision/(Wheel_perimeter*100));right_count=-left_count;
        tape_frame(60000,1);advance(10);ModeControl_GetTrackingOutput(&out);
        REQUIRE(out.tape_state==TAPE_CONFIRM && out.speed_mps>0);
        for(j=0;j<6;j++){tape_frame(60000,1);advance(20);}
        REQUIRE(s_tracking.tape_state==TAPE_APPROACH);
        tape_frame(12000,1);advance(10);ModeControl_GetTrackingOutput(&out);
        REQUIRE(out.tape_state==TAPE_WAIT && out.speed_mps==0 && out.requested_speed_mps==0);
        REQUIRE(MOTOR_A.Target==0 && MOTOR_B.Target==0);
        advance(2100);REQUIRE(s_tracking.tape_state==TAPE_WAIT); /* still rolling: no dwell */
        left_count=right_count=0;
        advance(500);tape_frame(3270,1);advance(10);
        REQUIRE(s_tracking.tape_state==TAPE_WAIT); /* less than one second */
        advance(510);REQUIRE(s_tracking.tape_state==TAPE_RECOVER); /* blind departure without route */
        tape_frame(3270,1);advance(10);REQUIRE(s_tracking.tape_state==TAPE_RECOVER);
        advance(900); /* Minimum blind time (T1BMIN=0.90s) precedes vision takeover. */
        for(j=0;j<3;j++){route_frame(0,4500,7,3,0);advance(20);}ModeControl_GetTrackingOutput(&out);
        REQUIRE(out.tape_state==TAPE_PASS);
        REQUIRE(out.steer_left_rad<=0 && out.steer_left_rad>=-TRACK_TAPE1_RIGHT_DEG*0.01745329252f);
        { float prev_steer=out.steer_left_rad;
          route_frame(0,0,6,3,0);advance(10);ModeControl_GetTrackingOutput(&out);
          REQUIRE(s_status==MODE_STATUS_RUNNING && out.tape_state==TAPE_RECOVER);
          REQUIRE(NEAR(out.speed_mps,TRACK_TAPE1_SPEED_MPS) && out.tape_entry_phase==2 && NEAR(out.steer_left_rad,prev_steer));
        }
        for(j=0;j<3;j++){route_frame(0,4500,7,3,0);advance(20);}
        REQUIRE(s_tracking.tape_state==TAPE_PASS);
        for(j=0;j<55;j++){tape_frame_mode(3270,1,3);advance(20);}
        REQUIRE(s_tracking.tape_state==TAPE_PASS); /* same line ignored */
        left_count=(int)(0.5f*Encoder_precision/(Wheel_perimeter*100));right_count=-left_count;
        for(j=0;j<60;j++){tape_frame_mode(0,0,3);advance(20);}
        REQUIRE(s_tracking.tape_state==TAPE_SEARCH);
    }
    case_id=14;start_route_fixture();
    {
        tape_frame(0,0);advance(10);REQUIRE(s_tracking.tape_state==TAPE_SEARCH);
        tape_frame(0,1);advance(10);REQUIRE(s_tracking.tape_state==TAPE_CONFIRM);
        { unsigned int j;for(j=0;j<6;j++){tape_frame(0,1);advance(20);} }
        REQUIRE(s_tracking.tape_state==TAPE_WAIT && MOTOR_A.Target==0);
        advance(4100);REQUIRE(s_tracking.tape_state==TAPE_FAULT && MOTOR_A.Target==0);
        REQUIRE(!ModeControl_DebugCommand("TRESET",1));
        REQUIRE(ModeControl_DebugCommand("RUN",0));
        REQUIRE(s_tracking.tape_state==TAPE_SEARCH);
    }
    {
        TrackingController c;
        TrackingParameters p;
        TrackingOutput o;
        VisionTarget v;
        float gentle, strong;
        memset(&v,0,sizeof(v));v.version=5;v.link_alive=1;v.valid=1;
        v.mode=1;v.y_centi_cm=5000;v.endpoint_valid=1;v.endpoint_y_centi_cm=6000;
        v.curvature_valid=1;v.tape_detected=1;v.received_ms=1000;
        v.tape_distance_y_centi_cm=4000;
        Tracking_DefaultParameters();Tracking_GetParameters(&p);
        p.decel_mps2=0.05f;p.tape_decel_mps2=0.40f;
        Tracking_Reset(&c,&o);c.route_speed_mps=0.45f;c.launch_complete=1;
        c.tape_state=TAPE_APPROACH;c.tape_distance_m=0.4f;c.tape_last_ms=1000;
        REQUIRE(Tracking_UpdateRoute(&c,&v,&v,1000,0.4f,&p,&o));
        gentle=o.speed_mps;
        REQUIRE(o.tape_state==TAPE_BRAKE && gentle>0 && gentle<0.30f);
        REQUIRE(o.speed_mps<=o.tape_speed_limit); /* bypass small TDEC and floor */
        p.tape_decel_mps2=1.0f;c.route_speed_mps=0.45f;c.tape_distance_m=0.4f;
        REQUIRE(Tracking_UpdateRoute(&c,&v,&v,1000,0.4f,&p,&o));
        strong=o.speed_mps;REQUIRE(strong>gentle);
        p.tape_gap_cm=15;c.route_speed_mps=0.45f;c.tape_distance_m=0.4f;
        REQUIRE(Tracking_UpdateRoute(&c,&v,&v,1000,0.4f,&p,&o));
        REQUIRE(o.speed_mps<strong);
        v.tape_detected=0;v.received_ms=1300;
        REQUIRE(Tracking_UpdateRoute(&c,&v,&v,1300,0.4f,&p,&o));
        REQUIRE(o.tape_state==TAPE_BRAKE && o.speed_mps>0);
        REQUIRE(NEAR(c.tape_distance_m,0.392f));
        Tracking_Reset(&c,&o);v.tape_detected=1;v.received_ms=2000;
        REQUIRE(Tracking_UpdateRoute(&c,&v,&v,2000,0,&p,&o));
        REQUIRE(Tracking_UpdateRoute(&c,&v,&v,2050,0,&p,&o));
        REQUIRE(c.tape_state==TAPE_CONFIRM && c.tape_seen_count==1);
        v.received_ms=2060;v.tape_detected=0;
        REQUIRE(Tracking_UpdateRoute(&c,&v,&v,2060,0,&p,&o));
        REQUIRE(c.tape_state==TAPE_SEARCH);
    }
    {
        TrackingController c; TrackingOutput o; TrackingParameters p; VisionTarget v;
        memset(&v,0,sizeof(v));Tracking_DefaultParameters();Tracking_GetParameters(&p);
        Tracking_Reset(&c,&o);c.tape_state=TAPE_RECOVER;c.tape_recover_ms=1000;
        c.tape_reliable_ms=1000;v.version=5;v.valid=v.link_alive=1;
        v.mode=1;v.y_centi_cm=5000;v.received_ms=1000;
        REQUIRE(Tracking_UpdateRoute(&c,&v,&v,1010,0,&p,&o));
        REQUIRE(c.tape_reliable_count==0 && o.speed_mps==0.5f);
        v.received_ms=1020;v.valid=0;
        REQUIRE(Tracking_UpdateRoute(&c,&v,&v,1020,0,&p,&o));
        REQUIRE(c.tape_reliable_count==0);
        v.valid=1;v.received_ms=900;
        REQUIRE(Tracking_UpdateRoute(&c,&v,&v,1040,0,&p,&o));
        REQUIRE(c.tape_reliable_count==0); /* Expired targets cannot recover. */
        v.mode=2;v.fallback=1;v.received_ms=1060;
        REQUIRE(Tracking_UpdateRoute(&c,&v,&v,1060,0,&p,&o));
        REQUIRE(c.tape_reliable_count==1);
        v.received_ms=1080;
        REQUIRE(Tracking_UpdateRoute(&c,&v,&v,1080,0,&p,&o));
        REQUIRE(c.tape_state==TAPE_RECOVER);
        v.received_ms=1100;
        REQUIRE(Tracking_UpdateRoute(&c,&v,&v,1100,0,&p,&o));
        REQUIRE(c.tape_state==TAPE_PASS); /* Single-boundary fallback is allowed. */
        Tracking_Reset(&c,&o);c.tape_state=TAPE_RECOVER;c.tape_recover_ms=2000;
        memset(&v,0,sizeof(v));
        REQUIRE(Tracking_UpdateRoute(&c,&v,&v,2999,0,&p,&o));
        REQUIRE(o.speed_mps==0.5f);
        REQUIRE(Tracking_UpdateRoute(&c,&v,&v,3000,0,&p,&o));
        REQUIRE(c.tape_state==TAPE_FAULT && o.speed_mps==0);
        v.version=5;v.valid=v.link_alive=1;v.mode=1;v.y_centi_cm=5000;v.received_ms=3020;
        REQUIRE(Tracking_UpdateRoute(&c,&v,&v,3020,0,&p,&o));
        REQUIRE(c.tape_state==TAPE_PASS && o.speed_mps>0);
    }
    {
        TrackingController c;TrackingOutput o;TrackingParameters p;VisionTarget v;
        uint32_t t=1000;unsigned int line,j;
        Tracking_Reset(&c,&o);Tracking_DefaultParameters();Tracking_GetParameters(&p);
        memset(&v,0,sizeof(v));v.version=5;v.valid=v.link_alive=1;
        v.mode=1;v.y_centi_cm=5000;v.tape_distance_y_centi_cm=60000;
        for(line=1;line<=4;line++) {
            v.tape_detected=1;
            for(j=0;j<6;j++) {
                v.received_ms=t;
                REQUIRE(Tracking_UpdateRoute(&c,&v,&v,t,0.6f,&p,&o));t+=20;
            }
            REQUIRE(c.tape_count==line && o.tape_count==line);
            if(line<4) {
                REQUIRE(c.tape_state==TAPE_APPROACH);
                c.tape_state=TAPE_PASS; /* Prior tests cover stop/wait/recovery. */
                for(j=0;j<40;j++) {
                    v.received_ms=t;REQUIRE(Tracking_UpdateRoute(&c,&v,&v,t,0.6f,&p,&o));t+=20;
                }
                REQUIRE(c.tape_count==line); /* Same tape cannot increment count. */
                v.tape_detected=0;
                for(j=0;j<60;j++) {
                    v.received_ms=t;REQUIRE(Tracking_UpdateRoute(&c,&v,&v,t,0.6f,&p,&o));t+=20;
                }
                REQUIRE(c.tape_state==TAPE_SEARCH);
            }
        }
        REQUIRE(c.tape_count==4 && c.tape_state!=TAPE_FINISH);
        p.finish_blind_time_s=0.30f; /* Explicit opt-in coverage of legacy finish crawl. */
        p.tape_finish_gap_cm=20;p.tape_gap_cm=5; c.tape_distance_m=TRACK_REAR_TO_FRONT_M+0.210f; c.route_speed_mps=0.40f; c.launch_complete=1; v.tape_detected=0; v.received_ms=t; REQUIRE(Tracking_UpdateRoute(&c,&v,&v,t,0,&p,&o)); REQUIRE(c.tape_state==TAPE_BRAKE && NEAR(o.speed_mps,p.finish_blind_speed_mps));
        c.tape_distance_m=TRACK_REAR_TO_FRONT_M+0.204f;
        v.tape_detected=0;v.received_ms=t; REQUIRE(Tracking_UpdateRoute(&c,&v,&v,t,0,&p,&o));
        REQUIRE(c.tape_state==TAPE_FINISH_BLIND && NEAR(o.speed_mps,p.finish_blind_speed_mps) && o.steer_left_rad==0 && o.tape_count==4);
        memset(&v,0,sizeof(v));t+=150; REQUIRE(Tracking_UpdateRoute(&c,&v,&v,t,0,&p,&o));
        REQUIRE(c.tape_state==TAPE_FINISH_BLIND && NEAR(o.tape_recover_remaining_s,0.15f) && NEAR(o.speed_mps,p.finish_blind_speed_mps));
        t+=150;
        REQUIRE(Tracking_UpdateRoute(&c,&v,&v,t,0,&p,&o));
        REQUIRE(c.tape_state==TAPE_FINISH && o.speed_mps==0 && o.tape_count==4);
        memset(&v,0,sizeof(v));t+=5000;
        REQUIRE(Tracking_UpdateRoute(&c,&v,&v,t,0,&p,&o));
        REQUIRE(c.tape_state==TAPE_FINISH && o.speed_mps==0 && o.tape_count==4);
        Tracking_Reset(&c,&o);REQUIRE(c.tape_count==0);
        c.tape_state=TAPE_RECOVER;c.tape_recover_ms=t;
        p.tape_recover_angle_deg=-12;p.tape_recover_speed_mps=0.65f;p.tape_recover_time_s=1.5f;
        REQUIRE(Tracking_UpdateRoute(&c,&v,&v,t+1200,0,&p,&o));
        REQUIRE(NEAR(o.steer_left_rad,-12*0.01745329252f) && o.speed_mps==0.65f);
        REQUIRE(NEAR(o.tape_recover_remaining_s,0.3f));
        REQUIRE(Tracking_UpdateRoute(&c,&v,&v,t+1500,0,&p,&o));
        REQUIRE(c.tape_state==TAPE_FAULT && o.speed_mps==0);
    }
    {
        TrackingController c;TrackingOutput o;TrackingParameters p;VisionTarget v;
        Tracking_Reset(&c,&o);Tracking_DefaultParameters();Tracking_GetParameters(&p);
        memset(&v,0,sizeof(v));v.version=5;v.valid=v.link_alive=1;
        v.mode=1;v.y_centi_cm=5000;v.received_ms=1000;
        c.tape_state=TAPE_BRAKE;c.tape_last_ms=1000;
        c.tape_distance_m=TRACK_REAR_TO_FRONT_M+p.tape_gap_cm*0.01f+0.004f;
        REQUIRE(Tracking_UpdateRoute(&c,&v,&v,1000,0,&p,&o));
        REQUIRE(c.tape_state==TAPE_WAIT && o.speed_mps==0 && c.tape_dwell_started);
        v.received_ms=2000;
        REQUIRE(Tracking_UpdateRoute(&c,&v,&v,2000,0,&p,&o));
        REQUIRE(c.tape_state==TAPE_RECOVER && NEAR(o.speed_mps,p.tape_recover_speed_mps));
    }
    {
        TrackingController c;TrackingOutput o;TrackingParameters p;VisionTarget v;
        uint32_t t;
        Tracking_Reset(&c,&o);Tracking_DefaultParameters();Tracking_GetParameters(&p);
        memset(&v,0,sizeof(v));v.version=5;v.valid=v.link_alive=1;
        v.mode=1;v.y_centi_cm=5000;
        c.tape_state=TAPE_BRAKE;c.tape_last_ms=1000;c.tape_count=1;
        c.tape_distance_m=TRACK_REAR_TO_FRONT_M+p.tape_gap_cm*0.01f+0.10f;
        /* Route remains fresh; tape disappears for longer than the old 300 ms limit. */
        for(t=1010;t<=1900;t+=10) {
            v.received_ms=t;
            REQUIRE(Tracking_UpdateRoute(&c,&v,&v,t,0.10f,&p,&o));
            REQUIRE(c.tape_state!=TAPE_FAULT && c.tape_state!=TAPE_WAIT);
        }
        REQUIRE(NEAR(c.tape_distance_m,TRACK_REAR_TO_FRONT_M+p.tape_gap_cm*0.01f+0.01f));
        for(;t<=2000;t+=10) {
            v.received_ms=t;
            REQUIRE(Tracking_UpdateRoute(&c,&v,&v,t,0.10f,&p,&o));
        }
        REQUIRE(c.tape_state==TAPE_WAIT && o.speed_mps==0 && c.tape_count==1);
        v.received_ms=2010;
        REQUIRE(Tracking_UpdateRoute(&c,&v,&v,2010,0,&p,&o));
        v.received_ms=3010;
        REQUIRE(Tracking_UpdateRoute(&c,&v,&v,3010,0,&p,&o));
        REQUIRE(c.tape_state==TAPE_RECOVER && NEAR(o.speed_mps,p.tape_recover_speed_mps));
        /* Tape odometry never makes an expired route valid for normal tracking. */
        Tracking_Reset(&c,&o);c.tape_state=TAPE_APPROACH;c.tape_distance_m=1;
        v.valid=0;v.link_alive=0;
        REQUIRE(!Tracking_UpdateRoute(&c,&v,&v,3020,0.1f,&p,&o));
    }
    {
        TrackingController c;TrackingOutput o;TrackingParameters p;VisionTarget v;
        uint32_t t;unsigned n;
        for(n=1;n<=3;n+=2) {
            Tracking_Reset(&c,&o);Tracking_DefaultParameters();Tracking_GetParameters(&p);
            memset(&v,0,sizeof(v));v.version=5;v.valid=v.link_alive=1;
            v.mode=1;v.y_centi_cm=5000;p.tape_recover_time_s=0.5f;
            c.tape_count=n;c.tape_state=TAPE_RECOVER;c.tape_recover_ms=1000;
            for(t=1010;t<2000;t+=10) {
                v.received_ms=t;
                REQUIRE(Tracking_UpdateRoute(&c,&v,&v,t,0,&p,&o));
                REQUIRE(c.tape_state==TAPE_RECOVER);
            }
            v.received_ms=2000;
            REQUIRE(Tracking_UpdateRoute(&c,&v,&v,2000,0,&p,&o));
            REQUIRE(c.tape_state==TAPE_PASS);
        }
    }
    {
        TrackingController c;TrackingOutput o;TrackingParameters p;VisionTarget v;
        Tracking_Reset(&c,&o);Tracking_DefaultParameters();Tracking_GetParameters(&p);
        memset(&v,0,sizeof(v));c.tape_state=TAPE_WAIT;c.tape_skip_recover=1;
        REQUIRE(Tracking_UpdateRoute(&c,&v,&v,1000,0,&p,&o));
        REQUIRE(Tracking_UpdateRoute(&c,&v,&v,2010,0,&p,&o));
        REQUIRE(c.tape_state==TAPE_RECOVER && NEAR(o.speed_mps,p.hold_speed_mps));
        REQUIRE(o.steer_left_rad==0);
        v.valid=v.link_alive=1;v.version=5;v.mode=1;v.y_centi_cm=5000;v.received_ms=2020;
        REQUIRE(Tracking_UpdateRoute(&c,&v,&v,2020,0,&p,&o));
        REQUIRE(c.tape_state==TAPE_PASS && o.speed_mps>0);
        c.tape_state=TAPE_RECOVER;c.tape_recover_ms=3000;
        v.valid=0;v.received_ms=4000;
        REQUIRE(Tracking_UpdateRoute(&c,&v,&v,4000,0,&p,&o));
        REQUIRE(c.tape_state==TAPE_FAULT && o.speed_mps==0);
        REQUIRE(ModeControl_DebugCommand("M",5));
        REQUIRE(ModeControl_GetDisplayMode()==MODE_CONTROL_VISION_NO_BLIND);
    }
    {
        uint8_t frame[9];
        REQUIRE(Vision_BuildLaneCommand(frame,1,5)==9);
        REQUIRE(frame[0]==0x55 && frame[1]==0xAA && frame[6]==5);
        REQUIRE(frame[7]==0x67 && frame[8]==0x3B);
    }
    {
        TrackingController c;TrackingOutput o;TrackingParameters p;VisionTarget v;
        uint8_t n;
        for(n=1;n<=3;n+=2) {
            Tracking_Reset(&c,&o);Tracking_DefaultParameters();Tracking_GetParameters(&p);
            REQUIRE(NEAR(p.tape1_straight_s,0.0f) && NEAR(p.tape1_right_deg,8.9f) && NEAR(p.tape1_speed_mps,0.58f) && NEAR(p.tape1_min_s,0.57f) && NEAR(p.tape1_max_s,1.0f));
            REQUIRE(NEAR(p.tape3_straight_s,0.60f) && NEAR(p.tape3_right_deg,15.2f) && NEAR(p.tape3_speed_mps,0.51f) && NEAR(p.tape3_min_s,1.00f) && NEAR(p.tape3_max_s,2.80f));
            p.tape1_straight_s=p.tape3_straight_s=0.30f;
            p.tape1_straight_cm=p.tape3_straight_cm=8.4f;
            p.tape1_right_deg=p.tape3_right_deg=10.0f;
            p.tape1_speed_mps=p.tape3_speed_mps=0.28f;
            p.tape1_min_s=p.tape3_min_s=0.30f;
            p.tape1_max_s=p.tape3_max_s=3.0f;
            memset(&v,0,sizeof(v));c.tape_state=TAPE_WAIT;c.tape_count=n;
            c.tape_lane_switch=1;
            REQUIRE(Tracking_UpdateRoute(&c,&v,&v,1000,0,&p,&o));
            REQUIRE(Tracking_UpdateRoute(&c,&v,&v,2000,0,&p,&o));
            REQUIRE(c.tape_state==TAPE_RECOVER);
            REQUIRE(c.tape_lane_event==0);
            REQUIRE(NEAR(o.speed_mps,0.28f));
            REQUIRE(o.steer_left_rad==0 && o.tape_entry_phase==1);
            v.valid=v.link_alive=1;v.version=5;v.mode=1;v.y_centi_cm=5000;v.received_ms=2005; REQUIRE(Tracking_UpdateRoute(&c,&v,&v,2005,0,&p,&o) && c.tape_state==TAPE_RECOVER && o.tape_recover_remaining_s>0.9f); v.mode=3;v.received_ms=2010; REQUIRE(Tracking_UpdateRoute(&c,&v,&v,2010,0,&p,&o) && c.tape_state==TAPE_RECOVER); v.received_ms=2020; REQUIRE(Tracking_UpdateRoute(&c,&v,&v,2020,0,&p,&o) && c.tape_state==TAPE_RECOVER); v.received_ms=2030; REQUIRE(Tracking_UpdateRoute(&c,&v,&v,2030,0,&p,&o));
            REQUIRE(c.tape_state==TAPE_RECOVER); /* Straight phase is mandatory. */
            Tracking_AdvanceOdometry(&c,0.085f);
            v.received_ms=2300;REQUIRE(Tracking_UpdateRoute(&c,&v,&v,2300,0,&p,&o));
            REQUIRE(o.tape_entry_phase==2 && NEAR(o.steer_left_rad,-10*0.01745329252f));
            v.received_ms=2310;REQUIRE(Tracking_UpdateRoute(&c,&v,&v,2310,0,&p,&o));
            v.received_ms=2320;REQUIRE(Tracking_UpdateRoute(&c,&v,&v,2320,0,&p,&o));
            REQUIRE(c.tape_state==TAPE_PASS && o.tape_entry_phase==3);
        }
        /* M5 never requests a lane change, including its second tape. */
        Tracking_Reset(&c,&o);memset(&v,0,sizeof(v));
        c.tape_state=TAPE_WAIT;c.tape_count=2;c.tape_skip_recover=1;
        REQUIRE(Tracking_UpdateRoute(&c,&v,&v,3000,0,&p,&o));
        REQUIRE(Tracking_UpdateRoute(&c,&v,&v,4000,0,&p,&o));
        REQUIRE(c.tape_state==TAPE_RECOVER && !c.tape_lane_event);

    }
    {
        TrackingController c;TrackingOutput o;TrackingParameters p;VisionTarget v;
        Tracking_Reset(&c,&o);Tracking_DefaultParameters();Tracking_GetParameters(&p);
        memset(&v,0,sizeof(v));c.tape_state=TAPE_FAULT;c.tape_reliable_ms=1000;
        v.valid=v.link_alive=1;v.version=5;v.mode=3;v.y_centi_cm=5000;
        v.received_ms=1000;
        REQUIRE(Tracking_UpdateRoute(&c,&v,&v,1010,0,&p,&o));
        REQUIRE(c.tape_state==TAPE_FAULT && o.speed_mps==0);
        v.received_ms=1020;
        REQUIRE(Tracking_UpdateRoute(&c,&v,&v,1020,0,&p,&o));
        REQUIRE(c.tape_state==TAPE_PASS && o.speed_mps>0);
        c.tape_state=TAPE_FINISH;v.received_ms=1030;
        REQUIRE(Tracking_UpdateRoute(&c,&v,&v,1030,0,&p,&o));
        REQUIRE(c.tape_state==TAPE_FINISH && o.speed_mps==0);
    }
    {
        TrackingController c;TrackingOutput o;TrackingParameters p;VisionTarget v;
        uint8_t n;uint32_t straight_end,start=10000;
        Tracking_DefaultParameters();Tracking_GetParameters(&p);
        p.tape1_straight_s=0.2f;p.tape1_right_deg=8;p.tape1_speed_mps=0.21f;p.tape1_min_s=0.2f;p.tape1_max_s=3;
        p.tape3_straight_s=0.4f;p.tape3_right_deg=12;p.tape3_speed_mps=0.31f;p.tape3_min_s=0.4f;p.tape3_max_s=4;
        p.tape1_straight_cm=4.2f;p.tape3_straight_cm=12.4f;
        for(n=1;n<=3;n+=2) {
            float angle=(n==1 ? -8.0f : -12.0f)*0.01745329252f;
            float speed=n==1 ? 0.21f : 0.31f;
            uint32_t deadline=start+(n==1 ? 3000 : 4000);
            Tracking_Reset(&c,&o);memset(&v,0,sizeof(v));
            c.tape_count=n;c.tape_lane_switch=1;c.tape_state=TAPE_RECOVER;
            c.tape_recover_ms=start;c.tape_reliable_ms=start;
            Vision_OnLaneSent(5,start);
            REQUIRE(Tracking_UpdateRoute(&c,&v,&v,start,0,&p,&o));
            REQUIRE(o.tape_entry_phase==1 && o.steer_left_rad==0 && NEAR(o.speed_mps,speed));
            if(n==1) {p.tape1_max_s=0.1f;p.tape1_straight_s=0;p.tape1_right_deg=20;p.tape1_speed_mps=0.8f;}
            straight_end=start+(n==1 ? 200 : 400);
            Tracking_AdvanceOdometry(&c,n==1 ? 0.0421f : 0.1241f);
            REQUIRE(Tracking_UpdateRoute(&c,&v,&v,straight_end,0,&p,&o));
            REQUIRE(o.tape_entry_phase==2 && NEAR(o.steer_left_rad,angle));
            v.valid=v.link_alive=1;v.version=5;v.mode=3;v.y_centi_cm=5000;
            v.received_ms=straight_end+10;REQUIRE(Tracking_UpdateRoute(&c,&v,&v,v.received_ms,0,&p,&o));
            REQUIRE(c.tape_reliable_count==1);
            REQUIRE(Tracking_UpdateRoute(&c,&v,&v,v.received_ms+10,0,&p,&o));
            REQUIRE(c.tape_reliable_count==1); /* Cached packet is not another frame. */
            v.received_ms+=20;REQUIRE(Tracking_UpdateRoute(&c,&v,&v,v.received_ms,0,&p,&o));
            v.received_ms+=20;REQUIRE(Tracking_UpdateRoute(&c,&v,&v,v.received_ms,0,&p,&o));
            REQUIRE(c.tape_state==TAPE_PASS && o.tape_entry_phase==3 && NEAR(o.steer_left_rad,angle));
            v.received_ms+=100;REQUIRE(Tracking_UpdateRoute(&c,&v,&v,v.received_ms,0,&p,&o));
            REQUIRE(NEAR(o.steer_left_rad,angle*0.5f)); /* 200 ms steering handover. */
            v.x_centi_cm=500;v.received_ms+=100;REQUIRE(Tracking_UpdateRoute(&c,&v,&v,v.received_ms,0,&p,&o));
            { float last_vis_steer=o.steer_left_rad;
              REQUIRE(last_vis_steer<0.0f);
              v.valid=0;v.received_ms+=10;
              REQUIRE(Tracking_UpdateRoute(&c,&v,&v,v.received_ms,0,&p,&o));
              REQUIRE(c.tape_state==TAPE_RECOVER && NEAR(o.speed_mps,speed) && NEAR(o.steer_left_rad,last_vis_steer));
              v.x_centi_cm=0;
            }
            REQUIRE(c.tape_recover_ms==start && NEAR(c.tape_entry_max_s,n==1 ? 3 : 4));
            /* First right frame followed by invalid must restart the 3-frame group. */
            v.valid=1;v.received_ms+=10;REQUIRE(Tracking_UpdateRoute(&c,&v,&v,v.received_ms,0,&p,&o));
            v.valid=0;v.received_ms+=10;REQUIRE(Tracking_UpdateRoute(&c,&v,&v,v.received_ms,0,&p,&o));
            REQUIRE(c.tape_reliable_count==0);
            v.valid=1;v.received_ms+=10;REQUIRE(Tracking_UpdateRoute(&c,&v,&v,v.received_ms,0,&p,&o));
            v.received_ms+=10;REQUIRE(Tracking_UpdateRoute(&c,&v,&v,v.received_ms,0,&p,&o));
            REQUIRE(c.tape_state==TAPE_RECOVER);
            v.received_ms+=10;REQUIRE(Tracking_UpdateRoute(&c,&v,&v,v.received_ms,0,&p,&o));
            REQUIRE(c.tape_state==TAPE_PASS);
            v.valid=0;v.received_ms=deadline-1;
            REQUIRE(Tracking_UpdateRoute(&c,&v,&v,deadline-1,0,&p,&o));
            REQUIRE(c.tape_state==TAPE_RECOVER && o.speed_mps>0);
            REQUIRE(Tracking_UpdateRoute(&c,&v,&v,deadline,0,&p,&o));
            REQUIRE(c.tape_state==TAPE_FAULT && o.speed_mps==0 && o.tape_entry_phase==4);
            /* Any valid mode recovers after three fresh post-deadline frames. */
            v.valid=1;v.mode=1;
            v.received_ms=deadline+10;REQUIRE(Tracking_UpdateRoute(&c,&v,&v,v.received_ms,0,&p,&o));
            REQUIRE(c.tape_state==TAPE_FAULT && c.tape_reliable_count==1);
            v.received_ms=deadline+20;REQUIRE(Tracking_UpdateRoute(&c,&v,&v,v.received_ms,0,&p,&o));
            REQUIRE(c.tape_state==TAPE_FAULT && c.tape_reliable_count==2 && o.speed_mps==0);
            v.received_ms=deadline+30;REQUIRE(Tracking_UpdateRoute(&c,&v,&v,v.received_ms,0,&p,&o));
            REQUIRE(c.tape_state==TAPE_PASS && !c.tape_entry_active && o.speed_mps>0);
            start+=10000;
        }
        /* Minimum total duration delays takeover even when straight distance is zero. */
        p.tape1_straight_cm=0;
        p.tape1_straight_s=0.1f;p.tape1_min_s=0.5f;p.tape1_right_deg=10.0f;p.tape1_speed_mps=0.30f;p.tape1_max_s=3.0f;
        Tracking_Reset(&c,&o);memset(&v,0,sizeof(v));
        c.tape_count=1;c.tape_lane_switch=1;c.tape_state=TAPE_RECOVER;c.tape_recover_ms=start;
        v.valid=v.link_alive=1;v.version=5;v.mode=3;v.y_centi_cm=5000;
        v.received_ms=start+200;REQUIRE(Tracking_UpdateRoute(&c,&v,&v,v.received_ms,0,&p,&o));
        v.received_ms=start+220;REQUIRE(Tracking_UpdateRoute(&c,&v,&v,v.received_ms,0,&p,&o));
        v.received_ms=start+240;REQUIRE(Tracking_UpdateRoute(&c,&v,&v,v.received_ms,0,&p,&o));
        REQUIRE(c.tape_state==TAPE_RECOVER && o.tape_entry_phase==2 && NEAR(o.steer_left_rad,-10.0f*0.01745329252f));
        v.received_ms=start+480;REQUIRE(Tracking_UpdateRoute(&c,&v,&v,v.received_ms,0,&p,&o));
        v.received_ms=start+490;REQUIRE(Tracking_UpdateRoute(&c,&v,&v,v.received_ms,0,&p,&o));
        REQUIRE(c.tape_state==TAPE_RECOVER);
        v.received_ms=start+500;REQUIRE(Tracking_UpdateRoute(&c,&v,&v,v.received_ms,0,&p,&o));
        REQUIRE(c.tape_state==TAPE_PASS && o.tape_entry_phase==3);
        /* Successful visual departure releases protection at its original deadline. */
        p.tape1_max_s=3;p.tape1_straight_s=0.2f;p.tape1_min_s=0.2f;
        Tracking_Reset(&c,&o);memset(&v,0,sizeof(v));
        c.tape_count=1;c.tape_lane_switch=1;c.tape_state=TAPE_RECOVER;c.tape_recover_ms=start;
        v.valid=v.link_alive=1;v.version=5;v.mode=3;v.y_centi_cm=5000;
        v.received_ms=start+500;REQUIRE(Tracking_UpdateRoute(&c,&v,&v,v.received_ms,0,&p,&o));
        v.received_ms+=10;REQUIRE(Tracking_UpdateRoute(&c,&v,&v,v.received_ms,0,&p,&o));
        v.received_ms+=10;REQUIRE(Tracking_UpdateRoute(&c,&v,&v,v.received_ms,0,&p,&o));
        REQUIRE(c.tape_state==TAPE_PASS && c.tape_entry_active);
        v.received_ms=start+3000;REQUIRE(Tracking_UpdateRoute(&c,&v,&v,v.received_ms,0,&p,&o));
        REQUIRE(!c.tape_entry_active && c.tape_state==TAPE_PASS);
        /* M5 first/third tapes retain THOLD straight and one-frame takeover. */
        for(n=1;n<=3;n+=2) {
            Tracking_Reset(&c,&o);memset(&v,0,sizeof(v));
            c.tape_count=n;c.tape_skip_recover=1;c.tape_state=TAPE_RECOVER;c.tape_recover_ms=start;
            REQUIRE(Tracking_UpdateRoute(&c,&v,&v,start,0,&p,&o));
            REQUIRE(!c.tape_entry_active && o.tape_entry_phase==0 && o.steer_left_rad==0 && NEAR(o.speed_mps,p.hold_speed_mps));
            v.valid=v.link_alive=1;v.version=5;v.mode=1;v.y_centi_cm=5000;v.received_ms=start+10;
            REQUIRE(Tracking_UpdateRoute(&c,&v,&v,v.received_ms,0,&p,&o));
            REQUIRE(c.tape_state==TAPE_PASS && !c.tape_entry_active);
        }
    }
    Tracking_DefaultParameters();
    {
        TrackingParameters p;Tracking_GetParameters(&p);
        REQUIRE(p.tape_gap_cm==7.0f && p.tape_finish_gap_cm==8.0f && p.tape_finish_gap_m5_cm==2.0f && p.finish_approach_speed_mps==0.56f && NEAR(p.tape_wait_s,0.30f));
        REQUIRE(Tracking_SetParameter("TFDIST",0.0f) && Tracking_SetParameter("TTWAIT",0.30f) && !Tracking_SetParameter("TTWAIT",0.09f));
        REQUIRE(p.finish_blind_speed_mps==0.25f && NEAR(p.finish_blind_time_s,0.0f));
        REQUIRE(Tracking_SetParameter("TFBV",0.35f) && Tracking_SetParameter("TFBT",0.45f) && !Tracking_SetParameter("TFBV",-0.1f) && !Tracking_SetParameter("TFBT",5.1f));
        REQUIRE(!Tracking_SetParameter("TFDIST",-0.1f));
    }
    {
        TrackingController c;TrackingOutput o;TrackingParameters p;VisionTarget v;
        unsigned mode,n;
        REQUIRE(Tracking_SetParameter("TFDIST3",5) && Tracking_SetParameter("TFDIST5",15));
        Tracking_GetParameters(&p);
        REQUIRE(p.tape_finish_gap_cm==5 && p.tape_finish_gap_m5_cm==15);
        REQUIRE(Tracking_SetParameter("TFDIST",10));Tracking_GetParameters(&p);
        REQUIRE(p.tape_finish_gap_cm==10 && p.tape_finish_gap_m5_cm==15);
        REQUIRE(!Tracking_SetParameter("TFDIST3",-0.1f) && !Tracking_SetParameter("TFDIST5",-0.1f));
        REQUIRE(!Tracking_SetParameter("TFDIST3",100.1f) && !Tracking_SetParameter("TFDIST5",100.1f));
        REQUIRE(Tracking_SetParameter("TFDIST3",100) && Tracking_SetParameter("TFDIST5",100));
        REQUIRE(Tracking_SetParameter("TFDIST3",0) && Tracking_SetParameter("TFDIST5",0));
        Tracking_GetParameters(&p);REQUIRE(p.tape_finish_gap_cm==0 && p.tape_finish_gap_m5_cm==0);
        REQUIRE(Tracking_SetParameter("TFDIST3",10) && Tracking_SetParameter("TFDIST5",15));
        Tracking_GetParameters(&p);
        memset(&v,0,sizeof(v));v.valid=v.link_alive=1;v.version=5;v.mode=1;
        v.y_centi_cm=5000;v.received_ms=1000;
        for(mode=0;mode<2;mode++) {
            Tracking_Reset(&c,&o);c.finish_approach_limit=mode;
            c.tape_count=4;c.tape_state=TAPE_APPROACH;
            c.tape_distance_m=TRACK_REAR_TO_FRONT_M+0.12f;
            c.launch_complete=1;
            REQUIRE(Tracking_UpdateRoute(&c,&v,&v,1000,0,&p,&o));
            if(mode) REQUIRE(c.tape_state==TAPE_FINISH_BLIND);
            else REQUIRE(c.tape_state==TAPE_APPROACH);
            /* All first three tapes retain the shared 7cm clearance in both maps. */
            for(n=1;n<=3;n++) {
                Tracking_Reset(&c,&o);c.finish_approach_limit=mode;
                c.tape_count=n;c.tape_state=TAPE_APPROACH;
                c.tape_distance_m=TRACK_REAR_TO_FRONT_M+0.08f;c.launch_complete=1;
                REQUIRE(Tracking_UpdateRoute(&c,&v,&v,1000,0,&p,&o));
                REQUIRE(c.tape_state==TAPE_APPROACH);
            }
        }
        Tracking_DefaultParameters();Tracking_GetParameters(&p);
        REQUIRE(p.tape_finish_gap_cm==8 && p.tape_finish_gap_m5_cm==2);
        for(mode=0;mode<2;mode++) {
            Tracking_Reset(&c,&o);c.finish_approach_limit=mode;
            c.tape_count=4;c.tape_state=TAPE_APPROACH;
            c.tape_distance_m=TRACK_REAR_TO_FRONT_M+0.004f;
            v.received_ms=1000;
            REQUIRE(Tracking_UpdateRoute(&c,&v,&v,1000,0,&p,&o));
            REQUIRE(c.tape_state==TAPE_FINISH && o.speed_mps==0);
            v.received_ms=2000;
            REQUIRE(Tracking_UpdateRoute(&c,&v,&v,2000,0,&p,&o));
            REQUIRE(c.tape_state==TAPE_FINISH && o.speed_mps==0);
        }
    }
    {
        TrackingController c;TrackingOutput o;TrackingParameters p;VisionTarget v;
        unsigned mode;
        Tracking_DefaultParameters();
        REQUIRE(Tracking_SetParameter("TFV",0.2f));
        REQUIRE(!Tracking_SetParameter("TFV",0));
        Tracking_GetParameters(&p);
        memset(&v,0,sizeof(v));v.valid=v.link_alive=1;v.version=5;
        v.mode=1;v.y_centi_cm=5000;v.received_ms=1000;
        for(mode=0;mode<2;mode++) {
            Tracking_Reset(&c,&o);c.tape_count=4;c.tape_state=TAPE_APPROACH;
            c.tape_distance_m=2;c.finish_approach_limit=mode;
            c.route_speed_mps=0.8f;c.launch_complete=1;
            REQUIRE(Tracking_UpdateRoute(&c,&v,&v,1000,0.8f,&p,&o));
            if(mode) REQUIRE(o.speed_mps<=0.2f);
            else REQUIRE(o.speed_mps>0.2f);
        }
    }

    {
        TrackingController c;TrackingOutput o;TrackingParameters p;VisionTarget v;
        Tracking_DefaultParameters();Tracking_GetParameters(&p);
        REQUIRE(p.local_heading_gain==0);
        REQUIRE(Tracking_SetParameter("THG",0.2f));REQUIRE(Tracking_SetParameter("THMAX",3));
        REQUIRE(!Tracking_SetParameter("THG",-1));Tracking_GetParameters(&p);
        memset(&v,0,sizeof(v));v.valid=v.link_alive=1;v.version=6;v.mode=1;
        v.y_centi_cm=5000;v.received_ms=1000;v.endpoint_valid=v.curvature_valid=1;
        v.endpoint_y_centi_cm=6000;v.heading_valid=1;v.heading_centi_deg=800;
        v.heading_reference_y_centi_cm=4110;
        Tracking_Reset(&c,&o);
        REQUIRE(Tracking_UpdateRoute(&c,&v,&v,1000,0.3f,&p,&o));
        REQUIRE(o.heading_used && o.heading_rad>0 && o.steer_left_rad>0);
        REQUIRE(o.heading_rad<=0.005237f);
        v.heading_valid=0;Tracking_UpdateRoute(&c,&v,&v,1010,0.3f,&p,&o);
        REQUIRE(!o.heading_used && o.heading_rad==0);
        v.heading_valid=1;v.heading_reference_y_centi_cm=6000;
        Tracking_UpdateRoute(&c,&v,&v,1020,0.3f,&p,&o);REQUIRE(!o.heading_used);
        v.heading_reference_y_centi_cm=4110;v.endpoint_abs_curvature_cm_inv=0.02f;
        Tracking_UpdateRoute(&c,&v,&v,1030,0.3f,&p,&o);REQUIRE(!o.heading_used);
        v.endpoint_abs_curvature_cm_inv=0;v.heading_centi_deg=-800;
        Tracking_UpdateRoute(&c,&v,&v,1040,0.3f,&p,&o);REQUIRE(o.heading_rad<0);
        Tracking_UpdateRoute(&c,&v,&v,1300,0.3f,&p,&o);REQUIRE(!o.heading_used && o.heading_rad==0);
    }

    case_id=15; /* Signed encoder travel, distance gate, stalled wheels and fresh trips. */
    {
        EncoderOdometry before,after;
        TrackingController c;TrackingOutput o;VisionTarget v;
        uint8_t n;unsigned int i;float step;
        reset();Balance_GetOdometry(&before);
        left_count=600;right_count=-1200;Get_Velocity_Form_Encoder();
        Balance_GetOdometry(&after);step=900.0f*Wheel_perimeter/Encoder_precision;
        REQUIRE(NEAR(after.left_m-before.left_m,600*Wheel_perimeter/Encoder_precision));
        REQUIRE(NEAR(after.right_m-before.right_m,1200*Wheel_perimeter/Encoder_precision));
        REQUIRE(NEAR(after.delta_m,step) && NEAR(after.centre_m-before.centre_m,step));
        REQUIRE(NEAR(after.delta_m,0.005f*(MOTOR_A.Encoder+MOTOR_B.Encoder)));
        left_count=-600;right_count=1200;Get_Velocity_Form_Encoder();Balance_GetOdometry(&after);
        REQUIRE(NEAR(after.centre_m,before.centre_m) && NEAR(after.delta_m,-step));
        left_count=right_count=600;Get_Velocity_Form_Encoder();Balance_GetOdometry(&after);
        REQUIRE(NEAR(after.delta_m,0)); /* Opposite wheel motion cancels at rear axle. */
        before=after;left_count=right_count=0;Get_Velocity_Form_Encoder();Balance_GetOdometry(&after);
        REQUIRE(after.delta_m==0 && after.centre_m==before.centre_m);
        Encoder_precision=0;left_count=600;right_count=-600;
        Get_Velocity_Form_Encoder();Balance_GetOdometry(&after);
        REQUIRE(after.delta_m==0 && MOTOR_A.Encoder==0 && after.centre_m==before.centre_m);
        reset();Tracking_DefaultParameters();Tracking_GetParameters(&p);
        REQUIRE(p.tape1_straight_cm==0 && NEAR(p.tape3_straight_cm,7.0f));
        for(n=1;n<=3;n+=2) {
            Tracking_Reset(&c,&o);memset(&v,0,sizeof(v));
            p.tape1_straight_cm=p.tape3_straight_cm=10;
            p.tape1_min_s=p.tape3_min_s=0;
            p.tape1_max_s=p.tape3_max_s=1;
            p.tape1_straight_s=p.tape3_straight_s=0; /* Legacy zero must not skip BD. */
            c.tape_count=n;c.tape_lane_switch=1;c.tape_state=TAPE_RECOVER;c.tape_recover_ms=1000;
            REQUIRE(Tracking_UpdateRoute(&c,&v,&v,1000,0,&p,&o));
            p.tape1_straight_cm=p.tape3_straight_cm=0; /* Locked for this departure. */
            v.valid=v.link_alive=1;v.version=5;v.mode=3;v.y_centi_cm=5000;
            for(i=1;i<=3;i++) {
                v.received_ms=1000+i*10;
                REQUIRE(Tracking_UpdateRoute(&c,&v,&v,v.received_ms,0,&p,&o));
            }
            REQUIRE(o.tape_entry_phase==1 && c.tape_state==TAPE_RECOVER);
            REQUIRE(NEAR(o.tape_entry_remaining_m,0.1f));
            Tracking_AdvanceOdometry(&c,0.06f);Tracking_AdvanceOdometry(&c,-0.04f);
            Tracking_AdvanceOdometry(&c,0.07f);v.received_ms=1100;
            REQUIRE(Tracking_UpdateRoute(&c,&v,&v,1100,0,&p,&o));
            REQUIRE(o.tape_entry_phase==1 && NEAR(o.tape_entry_travel_m,0.09f));
            REQUIRE(NEAR(o.tape_entry_remaining_m,0.01f));
            /* A commanded speed and elapsed time do not produce encoder travel. */
            v.received_ms=2000;REQUIRE(Tracking_UpdateRoute(&c,&v,&v,2000,0,&p,&o));
            REQUIRE(c.tape_state==TAPE_FAULT && o.speed_mps==0);
            for(i=1;i<=3;i++) {
                v.received_ms=2000+i*10;
                REQUIRE(Tracking_UpdateRoute(&c,&v,&v,v.received_ms,0,&p,&o));
            }
            REQUIRE(c.tape_state==TAPE_FAULT); /* Frames cannot bypass missing distance. */
            Tracking_Reset(&c,&o);Tracking_AdvanceOdometry(&c,1.0f);
            REQUIRE(c.tape_entry_travel_m==0 && !c.tape_entry_active);
        }
        /* Production encoder -> M3 tick integration, unequal wheels and reverse travel. */
        for(n=1;n<=3;n+=2) {
            start_route_fixture();route_run(1,0,4500,7,1,0);
            REQUIRE(ModeControl_DebugCommand(n==1 ? "T1BD" : "T3BD",2));
            REQUIRE(ModeControl_DebugCommand(n==1 ? "T1BMIN" : "T3BMIN",0.2f));
            s_tracking.tape_count=n;s_tracking.tape_state=TAPE_RECOVER;
            s_tracking.tape_recover_ms=now_ms;s_tracking.tape_reliable_ms=now_ms;
            tape_frame_mode(0,0,3);advance(10);
            REQUIRE(s_tracking.tape_entry_travel_m==0 && s_tracking_output.tape_entry_phase==1);
            left_count=300;right_count=-900;step=600*Wheel_perimeter/Encoder_precision;
            for(i=0;i<6;i++){tape_frame_mode(0,0,3);advance(10);}
            REQUIRE(NEAR(s_tracking.tape_entry_travel_m,6*step) && s_tracking_output.tape_entry_phase==1);
            left_count=-300;right_count=900;tape_frame_mode(0,0,3);advance(10);
            REQUIRE(NEAR(s_tracking.tape_entry_travel_m,5*step));
            left_count=300;right_count=-900;
            for(i=0;i<5;i++){tape_frame_mode(0,0,3);advance(10);}
            REQUIRE(s_tracking.tape_entry_straight_complete && s_tracking_output.tape_entry_phase==2);
            left_count=right_count=0;
            for(i=0;i<10;i++){tape_frame_mode(0,0,3);advance(10);}
            REQUIRE(s_tracking.tape_state==TAPE_PASS && s_tracking_output.tape_entry_phase==3);
            speed=s_tracking.tape_entry_travel_m;
            route_frame(0,4500,6,3,0);advance(10);
            REQUIRE(s_tracking_output.tape_entry_phase==2 && NEAR(s_tracking.tape_entry_travel_m,speed));
            REQUIRE(s_tracking.tape_entry_straight_complete); /* No repeat after vision loss. */
            REQUIRE(ModeControl_DebugCommand("RUN",0));
            REQUIRE(!s_tracking.tape_entry_active && s_tracking.tape_entry_travel_m==0);
        }
    }
    case_id=16; /* Calibration resets never alter encoder deltas or active M3 entry. */
    {
        EncoderOdometry before,after;TrackingController saved;TrackingOutput saved_output;
        float target_a,target_b;int servo_before;uint32_t status_before;
        start_route_fixture();route_run(1,0,4500,7,1,0);
        REQUIRE(ModeControl_DebugCommand("T3BD",20));
        s_tracking.tape_count=3;s_tracking.tape_state=TAPE_RECOVER;
        s_tracking.tape_recover_ms=now_ms;s_tracking.tape_reliable_ms=now_ms;
        tape_frame_mode(0,0,3);advance(10);
        left_count=300;right_count=-900;tape_frame_mode(0,0,3);advance(10);
        Balance_GetOdometry(&before);saved=s_tracking;saved_output=s_tracking_output;
        target_a=MOTOR_A.Target;target_b=MOTOR_B.Target;servo_before=Servo;
        status_before=ModeControl_GetStatus();
        REQUIRE(ModeControl_DebugCommand("ORESET",1));Balance_GetOdometry(&after);
        REQUIRE(after.calibration_m==0 && before.delta_m==after.delta_m);
        REQUIRE(before.left_m==after.left_m && before.right_m==after.right_m && before.centre_m==after.centre_m);
        REQUIRE(memcmp(&saved,&s_tracking,sizeof(saved))==0 && memcmp(&saved_output,&s_tracking_output,sizeof(saved_output))==0);
        REQUIRE(ModeControl_GetStatus()==status_before && MOTOR_A.Target==target_a && MOTOR_B.Target==target_b && Servo==servo_before);
        tape_frame_mode(0,0,3);advance(10);Balance_GetOdometry(&after);
        REQUIRE(NEAR(after.calibration_m,after.delta_m));
        REQUIRE(NEAR(s_tracking.tape_entry_travel_m,saved.tape_entry_travel_m+after.delta_m));
        REQUIRE(ModeControl_DebugCommand("RUN",0));Balance_GetOdometry(&before);
        REQUIRE(before.calibration_m==after.calibration_m);
        left_count=right_count=0;advance(10);Balance_GetOdometry(&after);
        REQUIRE(after.calibration_m==before.calibration_m && after.delta_m==0);
        /* Subtract the origin in double precision before narrowing to telemetry float. */
        {
            double saved_left=s_odometry_left_m,saved_right=s_odometry_right_m;
            double saved_origin=s_odometry_calibration_origin_m;
            s_odometry_left_m=s_odometry_right_m=100000000.0;
            irq_mask=1;Balance_ResetCalibrationOdometry();REQUIRE(irq_mask==1);irq_mask=0;
            Balance_GetOdometry(&before);
            left_count=600;right_count=-600;Get_Velocity_Form_Encoder();Balance_GetOdometry(&after);
            REQUIRE(after.centre_m==before.centre_m && after.calibration_m>0);
            REQUIRE(NEAR(after.calibration_m,after.delta_m));
            s_odometry_left_m=saved_left;s_odometry_right_m=saved_right;
            s_odometry_calibration_origin_m=saved_origin;
        }
    }
    case_id=17; /* Two bounded M3 intervals, each gated by its preceding yellow. */
    {
        TrackingController c;TrackingOutput o;TrackingParameters p;VisionTarget v;
        uint32_t t;uint8_t n;float start_m,end_m;uint8_t done_bit;
        Tracking_DefaultParameters();Tracking_GetParameters(&p);
        REQUIRE(p.exit_slow_distance_cm==157 && p.exit_slow_end_cm==217);
        REQUIRE(p.exit_slow3_start_cm==738 && p.exit_slow3_end_cm==799);
        REQUIRE(NEAR(p.exit_slow_speed_mps,0.30f));
        REQUIRE(!Tracking_SetParameter("TE1S",-1) && !Tracking_SetParameter("TE1E",10000.1f));
        REQUIRE(!Tracking_SetParameter("TE3S",-1) && !Tracking_SetParameter("TE3E",10000.1f));
        REQUIRE(!Tracking_SetParameter("TEV",0) && !Tracking_SetParameter("TEV",1.601f));
        {uint32_t nan_bits=0x7fc00000;float nan;memcpy(&nan,&nan_bits,4);
         REQUIRE(!Tracking_SetParameter("TE1S",nan) && !Tracking_SetParameter("TE3E",nan));}
        p.speed_mps=0.8f;p.decel_mps2=0.8f;
        memset(&v,0,sizeof(v));v.valid=v.link_alive=v.endpoint_valid=v.curvature_valid=1;
        v.version=5;v.mode=1;v.y_centi_cm=5000;v.endpoint_y_centi_cm=6000;
        for(n=1;n<=3;n+=2) {
            start_m=(n==1 ? p.exit_slow_distance_cm : p.exit_slow3_start_cm)*0.01f;
            end_m=(n==1 ? p.exit_slow_end_cm : p.exit_slow3_end_cm)*0.01f;
            done_bit=n==1 ? 1 : 2;
            Tracking_Reset(&c,&o);c.tape_lane_switch=1;c.tape_count=n-1;
            c.launch_complete=1;c.route_speed_mps=0.8f;c.exit_calibration_m=start_m;
            v.received_ms=1000;
            REQUIRE(Tracking_UpdateRoute(&c,&v,&v,1000,0.8f,&p,&o));
            REQUIRE(!o.exit_slow_active); /* Correct preceding yellow is mandatory. */
            c.tape_count=n;c.exit_calibration_m=start_m-0.001f;
            v.received_ms=1010;REQUIRE(Tracking_UpdateRoute(&c,&v,&v,1010,0.8f,&p,&o));
            REQUIRE(!o.exit_slow_active);
            c.exit_calibration_m=start_m;
            v.received_ms=1020;REQUIRE(Tracking_UpdateRoute(&c,&v,&v,1020,0.8f,&p,&o));
            REQUIRE(o.exit_slow_active && o.exit_slow_interval==n);
            REQUIRE(NEAR(o.requested_speed_mps,0.3f) && NEAR(o.speed_mps,0.792f));
            for(t=1030;t<=1700;t+=10){v.received_ms=t;REQUIRE(Tracking_UpdateRoute(&c,&v,&v,t,0.3f,&p,&o));}
            REQUIRE(NEAR(o.speed_mps,0.3f));
            p.single_speed_mps=0.2f;v.mode=2;v.received_ms=1710;
            REQUIRE(Tracking_UpdateRoute(&c,&v,&v,1710,0.3f,&p,&o));
            REQUIRE(NEAR(o.requested_speed_mps,0.2f) && NEAR(o.speed_mps,0.292f));
            v.mode=1;c.exit_calibration_m=start_m-0.01f;v.received_ms=1720;
            REQUIRE(Tracking_UpdateRoute(&c,&v,&v,1720,0.3f,&p,&o));
            REQUIRE(!o.exit_slow_active && !(c.exit_slow_done&done_bit));
            c.exit_calibration_m=start_m+0.1f;v.received_ms=1730;
            REQUIRE(Tracking_UpdateRoute(&c,&v,&v,1730,0.3f,&p,&o) && o.exit_slow_active);
            c.exit_calibration_m=end_m-0.001f;v.received_ms=1735;
            REQUIRE(Tracking_UpdateRoute(&c,&v,&v,1735,0.3f,&p,&o) && o.exit_slow_active);
            c.exit_calibration_m=end_m;v.received_ms=1740;
            REQUIRE(Tracking_UpdateRoute(&c,&v,&v,1740,0.3f,&p,&o));
            REQUIRE(!o.exit_slow_active && o.exit_slow_interval==0 && (c.exit_slow_done&done_bit));
            c.exit_calibration_m=start_m+0.1f;v.received_ms=1750;
            REQUIRE(Tracking_UpdateRoute(&c,&v,&v,1750,0.3f,&p,&o) && !o.exit_slow_active);
            /* Confirming the next yellow ends the cap in the same control tick. */
            Tracking_Reset(&c,&o);c.tape_lane_switch=1;c.tape_count=n;
            c.exit_calibration_m=start_m+0.1f;c.route_speed_mps=0.3f;c.launch_complete=1;
            v.received_ms=2000;REQUIRE(Tracking_UpdateRoute(&c,&v,&v,2000,0.3f,&p,&o) && o.exit_slow_active);
            c.tape_state=TAPE_CONFIRM;c.tape_seen_count=2;c.tape_first_ms=2000;c.tape_last_ms=2050;
            v.tape_detected=1;v.tape_distance_y_centi_cm=20000;v.received_ms=2120;
            REQUIRE(Tracking_UpdateRoute(&c,&v,&v,2120,0.3f,&p,&o));
            REQUIRE(c.tape_count==n+1 && !o.exit_slow_active && (c.exit_slow_done&done_bit));
            REQUIRE(o.requested_speed_mps>0.3f); /* No one-tick stale cap. */
            v.tape_detected=0;c.tape_state=TAPE_WAIT;v.received_ms=2130;
            REQUIRE(Tracking_UpdateRoute(&c,&v,&v,2130,0,&p,&o));
            REQUIRE(o.speed_mps==0 && !o.exit_slow_active);
            /* M5 never applies either interval. */
            Tracking_Reset(&c,&o);c.tape_count=n;c.exit_calibration_m=start_m+0.1f;
            c.route_speed_mps=0.8f;c.launch_complete=1;v.received_ms=2500;
            REQUIRE(Tracking_UpdateRoute(&c,&v,&v,2500,0.8f,&p,&o));
            REQUIRE(!o.exit_slow_active && NEAR(o.speed_mps,0.8f));
            /* Blind entry and fault stop retain the shared cap and stronger stop. */
            Tracking_Reset(&c,&o);c.tape_lane_switch=1;c.tape_count=n;c.tape_state=TAPE_RECOVER;
            c.tape_recover_ms=3000;c.exit_calibration_m=start_m+0.1f;c.route_speed_mps=0.6f;
            p.tape1_straight_cm=10;v.valid=0;
            REQUIRE(Tracking_UpdateRoute(&c,&v,&v,3000,0.6f,&p,&o));
            REQUIRE(o.exit_slow_active && o.tape_entry_phase==1);
            for(t=3010;t<=3500;t+=10) REQUIRE(Tracking_UpdateRoute(&c,&v,&v,t,0.3f,&p,&o));
            REQUIRE(NEAR(o.speed_mps,0.3f) && NEAR(o.requested_speed_mps,0.3f));
            REQUIRE(Tracking_UpdateRoute(&c,&v,&v,7000,0,&p,&o));
            REQUIRE(o.exit_slow_active && o.speed_mps==0 && c.tape_state==TAPE_FAULT);
            v.valid=1;
        }
        /* Independent edits allow any slider order; invalid pairs are inactive. */
        Tracking_Reset(&c,&o);c.tape_lane_switch=1;c.tape_count=1;c.route_speed_mps=0.8f;
        p.exit_slow_distance_cm=300;p.exit_slow_end_cm=200;c.exit_calibration_m=3.1f;
        v.received_ms=8000;REQUIRE(Tracking_UpdateRoute(&c,&v,&v,8000,0.8f,&p,&o));
        REQUIRE(!o.exit_slow_active && !(c.exit_slow_done&1));
        p.exit_slow_end_cm=400;v.received_ms=8010;
        REQUIRE(Tracking_UpdateRoute(&c,&v,&v,8010,0.8f,&p,&o) && o.exit_slow_active);
        p.exit_slow_speed_mps=0.25f;v.received_ms=8020;
        REQUIRE(Tracking_UpdateRoute(&c,&v,&v,8020,0.8f,&p,&o) && NEAR(o.requested_speed_mps,0.25f));
        c.exit_calibration_m=4;v.received_ms=8030;
        REQUIRE(Tracking_UpdateRoute(&c,&v,&v,8030,0.8f,&p,&o) && !o.exit_slow_active);
        p.exit_slow_end_cm=500;v.received_ms=8040;
        REQUIRE(Tracking_UpdateRoute(&c,&v,&v,8040,0.8f,&p,&o) && !o.exit_slow_active);
        /* Real M3 encoder chain, ORESET outside an interval, completion and new RUN. */
        start_route_fixture();REQUIRE(ModeControl_DebugCommand("ORESET",1));
        REQUIRE(ModeControl_DebugCommand("TE1S",5));REQUIRE(ModeControl_DebugCommand("TE1E",10));
        REQUIRE(ModeControl_DebugCommand("TV",0.8f));route_run(1,0,4500,7,1,0);
        s_tracking.tape_count=1;s_tracking.tape_state=TAPE_PASS;
        s_tracking.route_speed_mps=0.6f;s_tracking.launch_complete=1;
        left_count=10000;right_count=-10000;route_run(1,0,4500,7,1,0);
        REQUIRE(!s_tracking_output.exit_slow_active);
        route_run(1,0,4500,7,1,0);REQUIRE(s_tracking_output.exit_slow_interval==1);
        REQUIRE(ModeControl_DebugCommand("ORESET",1));left_count=right_count=0;
        route_run(1,0,4500,7,1,0);REQUIRE(!s_tracking_output.exit_slow_active);
        REQUIRE(s_tracking.tape_count==1 && s_tracking.tape_state==TAPE_PASS && !(s_tracking.exit_slow_done&1));
        left_count=10000;right_count=-10000;route_run(2,0,4500,7,1,0);
        REQUIRE(s_tracking_output.exit_slow_active);
        route_run(1,0,4500,7,1,0);REQUIRE(!s_tracking_output.exit_slow_active && (s_tracking.exit_slow_done&1));
        REQUIRE(ModeControl_DebugCommand("ORESET",1));route_run(2,0,4500,7,1,0);
        REQUIRE(!s_tracking_output.exit_slow_active);
        left_count=right_count=0;
        REQUIRE(ModeControl_DebugCommand("RUN",0));REQUIRE(!s_tracking.exit_slow_done);
        REQUIRE(ModeControl_DebugCommand("RUN",1));route_run(1,0,4500,7,1,0);
        REQUIRE(!s_tracking_output.exit_slow_active); /* No first yellow in a new task. */
        s_tracking.tape_count=1;s_tracking.tape_state=TAPE_PASS;
        route_run(1,0,4500,7,1,0);REQUIRE(s_tracking_output.exit_slow_active);
        s_tracking.tape_count=2;s_tracking.tape_state=TAPE_BRAKE;
        s_tracking.tape_distance_m=TRACK_REAR_TO_FRONT_M+0.07f;
        route_run(1,0,4500,7,1,0);REQUIRE(s_tracking_output.speed_mps==0 && !s_tracking_output.exit_slow_active);
    }
    case_id=18; /* Mileage finish cap starts before yellow confirmation, in M3 and M5. */
    {
        TrackingController c;TrackingOutput o;TrackingParameters p;VisionTarget v;
        uint32_t t;uint8_t mode;
        Tracking_DefaultParameters();Tracking_GetParameters(&p);
        REQUIRE(p.finish_slow_start_cm==950 && NEAR(p.finish_approach_speed_mps,0.56f));
        REQUIRE(!Tracking_SetParameter("TFSTART",-0.1f) && !Tracking_SetParameter("TFSTART",10000.1f));
        {uint32_t nan_bits=0x7fc00000;float nan;memcpy(&nan,&nan_bits,4);
         REQUIRE(!Tracking_SetParameter("TFSTART",nan));}
        p.speed_mps=0.91f;p.decel_mps2=1.6f;
        memset(&v,0,sizeof(v));v.valid=v.link_alive=v.endpoint_valid=v.curvature_valid=1;
        v.version=5;v.mode=1;v.y_centi_cm=5000;v.endpoint_y_centi_cm=6000;
        for(mode=0;mode<2;mode++) {
            Tracking_Reset(&c,&o);c.tape_lane_switch=!mode;c.finish_approach_limit=mode;
            c.launch_complete=1;c.route_speed_mps=0.91f;c.exit_calibration_m=9.499f;
            v.received_ms=1000;REQUIRE(Tracking_UpdateRoute(&c,&v,&v,1000,0.91f,&p,&o));
            REQUIRE(!o.finish_slow_active && NEAR(o.requested_speed_mps,0.91f));
            c.exit_calibration_m=9.5f;v.received_ms=1010;
            REQUIRE(Tracking_UpdateRoute(&c,&v,&v,1010,0.91f,&p,&o));
            REQUIRE(o.finish_slow_active && c.tape_count==0 && NEAR(o.requested_speed_mps,0.56f));
            REQUIRE(NEAR(o.speed_mps,0.894f)); /* TDEC ramp, one step only. */
            for(t=1020;t<=1400;t+=10){v.received_ms=t;REQUIRE(Tracking_UpdateRoute(&c,&v,&v,t,0.56f,&p,&o));}
            REQUIRE(NEAR(o.speed_mps,0.56f));
            p.single_speed_mps=0.2f;v.mode=2;v.received_ms=1410;
            REQUIRE(Tracking_UpdateRoute(&c,&v,&v,1410,0.56f,&p,&o));
            REQUIRE(NEAR(o.requested_speed_mps,0.2f) && NEAR(o.speed_mps,0.544f));
            v.mode=1;c.exit_calibration_m=9.49f;v.received_ms=1420;
            REQUIRE(Tracking_UpdateRoute(&c,&v,&v,1420,0.54f,&p,&o));
            REQUIRE(!o.finish_slow_active && NEAR(o.requested_speed_mps,0.91f)); /* Reverse or ORESET. */
            c.exit_calibration_m=9.5f;p.finish_slow_start_cm=960;v.received_ms=1430;
            REQUIRE(Tracking_UpdateRoute(&c,&v,&v,1430,0.54f,&p,&o) && !o.finish_slow_active);
            p.finish_slow_start_cm=950;p.finish_approach_speed_mps=0.3f;v.received_ms=1440;
            REQUIRE(Tracking_UpdateRoute(&c,&v,&v,1440,0.54f,&p,&o));
            REQUIRE(o.finish_slow_active && NEAR(o.requested_speed_mps,0.3f)); /* Live TFSTART/TFV. */
            p.finish_approach_speed_mps=0.56f;
            c.tape_state=TAPE_BRAKE;c.tape_count=4;c.tape_distance_m=TRACK_REAR_TO_FRONT_M+0.11f;
            v.received_ms=1450;REQUIRE(Tracking_UpdateRoute(&c,&v,&v,1450,0.56f,&p,&o));
            REQUIRE(o.finish_slow_active && o.tape_speed_limit<0.56f && o.speed_mps<=o.tape_speed_limit);
            c.tape_distance_m=TRACK_REAR_TO_FRONT_M;v.received_ms=1460;
            REQUIRE(Tracking_UpdateRoute(&c,&v,&v,1460,0,&p,&o));
            REQUIRE(c.tape_state==TAPE_FINISH && o.speed_mps==0 && o.requested_speed_mps==0);
            v.received_ms=1470;REQUIRE(Tracking_UpdateRoute(&c,&v,&v,1470,0,&p,&o));
            REQUIRE(o.speed_mps==0 && o.finish_slow_active); /* Cap never raises a terminal zero. */
            /* V3 and blind recovery take the common mileage cap with the same ramp. */
            Tracking_Reset(&c,&o);c.exit_calibration_m=9.5f;c.route_speed_mps=0.91f;
            v.version=3;v.received_ms=2000;
            REQUIRE(Tracking_UpdateRoute(&c,&v,&v,2000,0.91f,&p,&o));
            REQUIRE(o.finish_slow_active && NEAR(o.requested_speed_mps,0.56f) && NEAR(o.speed_mps,0.894f));
            v.version=5;Tracking_Reset(&c,&o);c.exit_calibration_m=9.5f;c.route_speed_mps=0.8f;
            c.tape_state=TAPE_RECOVER;c.tape_recover_ms=2100;p.tape_recover_speed_mps=0.9f;
            v.valid=0;v.received_ms=2100;
            REQUIRE(Tracking_UpdateRoute(&c,&v,&v,2100,0.8f,&p,&o));
            REQUIRE(o.finish_slow_active && NEAR(o.requested_speed_mps,0.56f) && NEAR(o.speed_mps,0.784f));
            v.valid=1;
        }
        /* Actual encoder + mode chain feeds calibration in BOTH modes and rechecks ORESET. */
        for(mode=3;mode<=5;mode+=2) {
            start_route_fixture();REQUIRE(ModeControl_DebugCommand("RUN",0));
            REQUIRE(ModeControl_DebugCommand("M",mode));REQUIRE(ModeControl_DebugCommand("ORESET",1));
            REQUIRE(ModeControl_DebugCommand("TFSTART",5));REQUIRE(ModeControl_DebugCommand("TV",0.8f));
            REQUIRE(ModeControl_DebugCommand("RUN",1));route_run(1,0,4500,7,1,0);
            s_tracking.route_speed_mps=0.8f;s_tracking.launch_complete=1;
            left_count=10000;right_count=-10000;route_run(2,0,4500,7,1,0);
            REQUIRE(s_tracking_output.finish_slow_active && NEAR(s_tracking_output.requested_speed_mps,0.56f));
            REQUIRE(s_tracking.tape_count==0 && NEAR(s_tracking.route_speed_mps,0.792f));
            REQUIRE(ModeControl_DebugCommand("ORESET",1));left_count=right_count=0;
            route_run(1,0,4500,7,1,0);
            REQUIRE(!s_tracking_output.finish_slow_active && NEAR(s_tracking_output.requested_speed_mps,0.8f));
        }
    }
    finish(0);
}
