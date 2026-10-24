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
    ModeControl_DebugCommand("TGEX",0.60f);
}
static void tape_frame(uint16_t cm100,uint8_t detected)
{
    uint8_t v[25];uint16_t crc;
    memcpy(v,v4_examples[0],20);v[2]=5;v[3]=25;
    v[4]=(uint8_t)route_seq;v[5]=(uint8_t)(route_seq++>>8);
    v[8]=v[9]=0;memset(v+16,0,4);v[20]=detected;
    v[21]=(uint8_t)cm100;v[22]=(uint8_t)(cm100>>8);
    crc=Vision_Crc16(v,23);v[23]=(uint8_t)crc;v[24]=(uint8_t)(crc>>8);
    send_bytes(v,25);
}
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
    REQUIRE(p.pp_gain==1.40f && p.curve_threshold==0.82f && p.curve_speed_gain==0.16f);
    REQUIRE(p.fallback_speed_mps==0.40f && p.hold_speed_mps==0.40f && p.accel_mps2==0.430f);
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
    REQUIRE(p.pp_gain==1.40f && p.pp_straight_gain==0.95f);
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
    REQUIRE(p.start_accel_mps2==0.80f && p.accel_mps2==0.430f);
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
        tape_frame(3270,1);advance(10);ModeControl_GetTrackingOutput(&out);
        REQUIRE(out.tape_state==TAPE_PASS);
        REQUIRE(out.steer_left_rad==0);
        for(j=0;j<55;j++){tape_frame(3270,1);advance(20);}
        REQUIRE(s_tracking.tape_state==TAPE_PASS); /* same line ignored */
        left_count=(int)(0.5f*Encoder_precision/(Wheel_perimeter*100));right_count=-left_count;
        for(j=0;j<60;j++){tape_frame(0,0);advance(20);}
        REQUIRE(s_tracking.tape_state==TAPE_SEARCH);
    }
    case_id=14;start_route_fixture();
    {
        tape_frame(0,0);advance(10);REQUIRE(s_tracking.tape_state==TAPE_SEARCH);
        tape_frame(0,1);advance(10);REQUIRE(s_tracking.tape_state==TAPE_CONFIRM);
        { unsigned int j;for(j=0;j<6;j++){tape_frame(0,1);advance(20);} }
        REQUIRE(s_tracking.tape_state==TAPE_WAIT && MOTOR_A.Target==0);
        advance(2500);REQUIRE(s_tracking.tape_state==TAPE_FAULT && MOTOR_A.Target==0);
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
        p.tape_finish_gap_cm=20;p.tape_gap_cm=5;
        c.tape_distance_m=TRACK_REAR_TO_FRONT_M+0.204f;
        v.tape_detected=0;v.received_ms=t;
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
            memset(&v,0,sizeof(v));c.tape_state=TAPE_WAIT;c.tape_count=n;
            c.tape_lane_switch=1;
            REQUIRE(Tracking_UpdateRoute(&c,&v,&v,1000,0,&p,&o));
            REQUIRE(Tracking_UpdateRoute(&c,&v,&v,2000,0,&p,&o));
            REQUIRE(c.tape_state==TAPE_RECOVER);
            REQUIRE(c.tape_lane_event==1);
            REQUIRE(NEAR(o.speed_mps,p.hold_speed_mps));
            REQUIRE(NEAR(o.steer_left_rad,n==3 ? -0.0523598776f : 0.0f));
            v.valid=v.link_alive=1;v.version=5;v.mode=3;
            v.y_centi_cm=5000;v.received_ms=2010;
            REQUIRE(Tracking_UpdateRoute(&c,&v,&v,2010,0,&p,&o));
            REQUIRE(c.tape_state==TAPE_PASS);
        }
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
    Tracking_DefaultParameters();
    {
        TrackingParameters p;Tracking_GetParameters(&p);
        REQUIRE(p.tape_gap_cm==4.0f && p.tape_finish_gap_cm==0.5f);
        REQUIRE(Tracking_SetParameter("TFDIST",0.5f));
        REQUIRE(!Tracking_SetParameter("TFDIST",-0.1f));
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
    finish(0);
}
