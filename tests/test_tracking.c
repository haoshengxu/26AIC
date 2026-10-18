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
    REQUIRE(NEAR(out.error_m,-3.0768f) && NEAR(out.steer_left_rad,0.35f));
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
    case_id=8;reset();Tracking_Reset(&controller,&out);
    feed_distance(1,100,1);Tracking_GetInput(&in);
    {
        union {uint32_t bits;float number;} nonfinite;
        nonfinite.bits=0x7FC00000U;
        REQUIRE(!Tracking_Update(&controller,&in,now_ms,nonfinite.number,0,0,&out));
        REQUIRE(!out.valid && out.speed_mps==0 && out.steer_left_rad==0);
    }
    finish(0);
}
