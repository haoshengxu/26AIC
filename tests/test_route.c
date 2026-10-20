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
    ModeControl_DebugCommand("TV",0.35f);
    ModeControl_DebugCommand("TVMIN",0.27f);
    ModeControl_DebugCommand("TPG",1.0f);
    ModeControl_DebugCommand("TCTH",0.8f);
    ModeControl_DebugCommand("THOLD",0.18f);
    ModeControl_DebugCommand("TACC",0.30f);
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
    route_frame(500,4500,7,1,0);advance(10);ModeControl_GetTrackingOutput(&out);
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
    advance(10);REQUIRE(NEAR(MOTOR_A.Target,0.3f) && NEAR(MOTOR_B.Target,0.3f));
    REQUIRE(ModeControl_DebugCommand("M",3) && ModeControl_DebugCommand("RUN",1));
    route_frame(0,4500,7,1,0);advance(10);ModeControl_GetTrackingOutput(&out);
    REQUIRE(NEAR(out.speed_mps,0.003f));
    REQUIRE(ModeControl_DebugCommand("TRESET",1));Tracking_GetParameters(&p);
    REQUIRE(p.pp_gain==1.27f && p.curve_threshold==0.82f && p.curve_speed_gain==0.08f);
    REQUIRE(p.fallback_speed_mps==0.20f && p.hold_speed_mps==0.20f && p.accel_mps2==0.250f);
    Vision_UartError();advance(10);REQUIRE(!ModeControl_IsActive() && MOTOR_A.Target==0);
    finish(0);
}
