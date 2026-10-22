/* Execute the production C sources on a Cortex-M4 emulator with fake I/O.
 * No alternate Python implementation of the controllers is used. */
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
static GPIO_PinState user_key = GPIO_PIN_SET;
static GPIO_PinState buzzer;
static int left_count, right_count;
static float simulated_yaw;
static int send_imu;
static unsigned int assertions;
static unsigned int case_id;
uint32_t HAL_GetTick(void) { return now_ms; }
GPIO_PinState HAL_GPIO_ReadPin(void *port, uint16_t pin) { (void)port; (void)pin; return user_key; }
void HAL_GPIO_WritePin(void *port, uint16_t pin, GPIO_PinState state) { (void)port; (void)pin; buzzer = state; }
int Read_Encoder(u8 timer) { return timer == 2U ? left_count : right_count; }
void Led_Flash(uint16_t ticks) { (void)ticks; }
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
    mailbox[0] = assertions;
    mailbox[1] = line;
    mailbox[2] = case_id;
    mailbox[3] = 0xACCE5501U;
    __asm { BKPT 0 }
    for (;;) { }
}
#define REQUIRE(expr) do { ++assertions; if (!(expr)) finish(__LINE__); } while (0)
#define NEAR(a,b,eps) (fabsf((a)-(b)) < (eps))

static void feed_yaw(float yaw)
{
    uint8_t frame[11] = {0x5A, 0xBB, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    int16_t raw = (int16_t)(Ack_AngleError(yaw, 0.0f) * 32768.0f / 180.0f);
    int i;
    frame[6] = (uint8_t)raw;
    frame[7] = (uint8_t)((uint16_t)raw >> 8);
    for (i = 0; i < 10; ++i) frame[10] += frame[i];
    for (i = 0; i < 11; ++i) Gyro6_ProcessByte(frame[i]);
}
static void advance(unsigned int milliseconds)
{
    TIM_HandleTypeDef timer = {TIM7};
    unsigned int i;
    for (i = 0; i < milliseconds; i += 10U) {
        now_ms += 10U;
        if (send_imu) feed_yaw(simulated_yaw);
        Balance_TimerCallback(&timer);
    }
}
static void reset_board(uint32_t time, float yaw)
{
    now_ms = time; simulated_yaw = yaw; send_imu = 1;
    s_valid = 0; s_frame_index = 0;
    user_key = GPIO_PIN_SET; enable_input = 1; Voltage = 12.0f; Check = 0;
    left_count = right_count = 0;
    Encoder_precision = 4.0f * ACK_ENCODER_LINES * ACK_GEAR_RATIO;
    Wheel_perimeter = ACK_WHEEL_DIAMETER_M * 3.14159265359f;
    feed_yaw(yaw);
    Vision_Init();
    ModeControl_Init();
}
static void short_press(void) { user_key = GPIO_PIN_RESET; advance(60); user_key = GPIO_PIN_SET; advance(100); }
static void single_click(void) { short_press(); advance(310); }
static void double_click(void) { short_press(); short_press(); advance(310); }

void test_entry(void)
{
    float left, right, result, ratio;
    AckSpeedPI pi_a, pi_b;
    AckYawPID pid;
    uint32_t start;
    unsigned int i;

    case_id = 1; /* Kinematics, pulse limits, curvature preservation. */
    Ack_Kinematics(0.3f, 0.0f, &left, &right);
    REQUIRE(NEAR(left, 0.3f, 0.00001f) && NEAR(right, 0.3f, 0.00001f));
    Ack_Kinematics(0.3f, 0.3f, &left, &right);
    REQUIRE(right > left && left > 0.0f);
    REQUIRE(NEAR((left + right) * 0.5f, 0.3f, 0.00001f));
    ratio = left / right;
    Ack_Kinematics(30.0f, 0.3f, &left, &right);
    REQUIRE(NEAR(right, ACK_MAX_WHEEL_SPEED_MPS, 0.00001f));
    REQUIRE(NEAR(left / right, ratio, 0.00001f));
    Ack_Kinematics(0.3f, -0.3f, &left, &right);
    REQUIRE(left > right && right > 0.0f);
    REQUIRE(Ack_ServoPulse(0.0f) == ACK_SERVO_CENTER_US);
    REQUIRE(Ack_ServoPulse(100.0f) >= ACK_SERVO_MIN_US);
    REQUIRE(Ack_ServoPulse(-100.0f) <= ACK_SERVO_MAX_US);

    case_id = 2; /* Separate positional PI state, saturation and reset. */
    Ack_SpeedReset(&pi_a); Ack_SpeedReset(&pi_b);
    result = Ack_SpeedUpdate(&pi_a, 0.3f, 0.0f, 1500.0f, 500.0f);
    REQUIRE(NEAR(result, 600.0f, 0.01f));
    REQUIRE(pi_b.output == 0.0f);
    REQUIRE(NEAR(Ack_SpeedUpdate(&pi_b, 0.3f, 0.2f, 1500.0f, 500.0f), 200.0f, 0.01f));
    for (i = 0; i < 500; ++i) Ack_SpeedUpdate(&pi_a, 10.0f, 0.0f, 1500.0f, 500.0f);
    REQUIRE(pi_a.output == ACK_MOTOR_PWM_LIMIT);
    REQUIRE(pi_a.p_output==15000 && pi_a.i_output==5750);
    REQUIRE(pi_a.integral==ACK_SPEED_INTEGRAL_LIMIT);
    REQUIRE(Ack_SpeedUpdate(&pi_a, 0.0f, 1.0f, 1500.0f, 500.0f) < ACK_MOTOR_PWM_LIMIT);
    Ack_SpeedReset(&pi_a);
    REQUIRE(pi_a.output == 0.0f && pi_a.last_error == 0.0f);
    REQUIRE(pi_a.p_output==0 && pi_a.i_output==0 && pi_a.integral==0);

    case_id = 3; /* Wrapped position PID and anti-windup. */
    REQUIRE(NEAR(Ack_AngleError(-179.0f, 179.0f), 2.0f, 0.00001f));
    REQUIRE(NEAR(Ack_AngleError(179.0f, -179.0f), -2.0f, 0.00001f));
    Ack_YawReset(&pid, 2.0f);
    result = Ack_YawUpdate(&pid, -179.0f, 179.0f);
    REQUIRE(result > 0.0f && result < 0.1f);
    Ack_YawReset(&pid, 90.0f);
    for (i = 0; i < 1000; ++i) result = Ack_YawUpdate(&pid, 90.0f, 0.0f);
    REQUIRE(result == ACK_MAX_STEER_RAD && pid.integral == 0.0f);

    case_id = 4; /* Real IMU parser: corrupt bytes, checksum, freshness. */
    reset_board(0U, 0.0f);
    Gyro6_ProcessByte(0x5A); Gyro6_ProcessByte(0x00); feed_yaw(23.0f); feed_yaw(23.0f);
    REQUIRE(NEAR(Gyro6_GetYaw(), 23.0f, 0.01f));
    now_ms += ACK_IMU_TIMEOUT_MS;
    REQUIRE(Gyro6_IsValid());
    ++now_ms; REQUIRE(!Gyro6_IsValid());
    feed_yaw(-42.0f); REQUIRE(Gyro6_IsValid());

    case_id = 5; /* M1: actual ISR, 5 seconds, stop, centre, beep once. */
    reset_board(0U, 37.0f); single_click();
    REQUIRE(ModeControl_IsActive());
    REQUIRE(NEAR(ModeControl_GetTargetYaw(), 37.0f, 0.01f));
    REQUIRE(NEAR(MOTOR_A.Target, ACK_TASK1_SPEED_MPS, 0.00001f));
    start = s_start_ms;
    simulated_yaw = 35.0f; advance(10);
    REQUIRE(MOTOR_B.Target > MOTOR_A.Target && Servo < SERVO_INIT);
    simulated_yaw = 37.0f;
    while ((uint32_t)(now_ms - start) < 4990U) advance(10);
    REQUIRE(ModeControl_IsActive());
    advance(10);
    REQUIRE(!ModeControl_IsActive() && ModeControl_GetStatus() == MODE_STATUS_DONE);
    REQUIRE(ModeControl_GetElapsedMs() == 5000U && buzzer == GPIO_PIN_SET);
    REQUIRE(MOTOR_A.Target == 0.0f && MOTOR_B.Target == 0.0f && Servo_PWM == SERVO_INIT);
    REQUIRE(PWMA1 == PWMA2 && PWMB1 == PWMB2 && s_left_pi.output == 0.0f && s_right_pi.output == 0.0f);
    advance(490); REQUIRE(buzzer == GPIO_PIN_SET);
    advance(10); REQUIRE(buzzer == GPIO_PIN_RESET);
    advance(1000); REQUIRE(!ModeControl_IsActive() && ModeControl_GetElapsedMs() == 5000U);

    case_id = 6; /* Double-click must switch without a single-click start. */
    reset_board(0U, 0.0f); short_press();
    REQUIRE(!ModeControl_IsActive());
    short_press(); advance(310);
    REQUIRE(!ModeControl_IsActive() && ModeControl_GetDisplayMode() == MODE_CONTROL_TURN_90);
    single_click(); REQUIRE(ModeControl_IsActive());
    double_click(); REQUIRE(!ModeControl_IsActive() && ModeControl_GetDisplayMode() == MODE_CONTROL_VISION_LINE);
    double_click(); REQUIRE(!ModeControl_IsActive() && ModeControl_GetDisplayMode() == MODE_CONTROL_DEBUG);
    double_click(); REQUIRE(!ModeControl_IsActive() && ModeControl_GetDisplayMode() == MODE_CONTROL_STRAIGHT_5S);
    advance(1000); REQUIRE(!ModeControl_IsActive());

    case_id = 7; /* Single stop, long stop and bounce suppression. */
    reset_board(0U, 0.0f); single_click(); single_click();
    REQUIRE(ModeControl_GetStatus() == MODE_STATUS_STOPPED);
    single_click(); REQUIRE(ModeControl_IsActive());
    user_key = GPIO_PIN_RESET; advance(600);
    REQUIRE(ModeControl_GetStatus() == MODE_STATUS_STOPPED);
    user_key = GPIO_PIN_SET; advance(600); REQUIRE(!ModeControl_IsActive());
    reset_board(0U, 0.0f);
    for (i = 0; i < 10; ++i) { user_key = GPIO_PIN_RESET; advance(10); user_key = GPIO_PIN_SET; advance(10); }
    advance(700); REQUIRE(!ModeControl_IsActive());
    user_key = GPIO_PIN_RESET; ModeControl_Init(); advance(1000);
    user_key = GPIO_PIN_SET; advance(700); REQUIRE(!ModeControl_IsActive());

    case_id = 8; /* M2 ideal bicycle model, including +180/-180 crossing. */
    reset_board(0U, 170.0f); double_click(); single_click();
    REQUIRE(ModeControl_IsActive() && NEAR(s_target_yaw, -100.0f, 0.01f));
    for (i = 0; i < 1500 && ModeControl_IsActive(); ++i) {
        float steer = (Servo - ACK_SERVO_CENTER_US) / (ACK_SERVO_DIRECTION * ACK_SERVO_US_PER_RAD);
        REQUIRE(MOTOR_A.Target > 0.0f && MOTOR_B.Target > 0.0f);
        simulated_yaw = Ack_AngleError(simulated_yaw + ACK_TASK2_SPEED_MPS * tanf(steer) /
                                      ACK_WHEELBASE_M * 57.2957795f * 0.01f, 0.0f);
        advance(10);
    }
    REQUIRE(ModeControl_GetStatus() == MODE_STATUS_DONE);
    REQUIRE(fabsf(Ack_AngleError(-100.0f, simulated_yaw)) < 1.5f);
    REQUIRE(buzzer == GPIO_PIN_SET && MOTOR_A.Target == 0.0f);

    case_id = 9; /* Opposite-direction motion must not count as +90 degrees. */
    reset_board(0U, 0.0f); double_click(); single_click();
    simulated_yaw = -90.0f; advance(10); REQUIRE(ModeControl_IsActive());
    start = s_start_ms;
    while ((uint32_t)(now_ms - start) < ACK_TASK2_TIMEOUT_MS) advance(10);
    REQUIRE(ModeControl_GetStatus() == MODE_STATUS_TIMEOUT && !ModeControl_IsActive());

    case_id = 10; /* Freshness and drive interlocks stop and latch off. */
    reset_board(0U, 0.0f); single_click(); send_imu = 0; advance(210);
    REQUIRE(ModeControl_GetStatus() == MODE_STATUS_IMU_ERROR && MOTOR_A.Target == 0.0f);
    send_imu = 1; advance(1000); REQUIRE(!ModeControl_IsActive());
    single_click(); enable_input = 0; advance(10);
    REQUIRE(ModeControl_GetStatus() == MODE_STATUS_POWER_ERROR && MOTOR_B.Target == 0.0f);
    enable_input = 1; advance(1000); REQUIRE(!ModeControl_IsActive());
    single_click(); Voltage = 9.5f; advance(10);
    REQUIRE(ModeControl_GetStatus() == MODE_STATUS_POWER_ERROR);
    single_click(); REQUIRE(!ModeControl_IsActive());
    reset_board(0U, 0.0f); s_valid = 0; send_imu = 0; single_click();
    REQUIRE(!ModeControl_IsActive() && ModeControl_GetStatus() == MODE_STATUS_IMU_ERROR);

    case_id = 11; /* HAL millisecond wrap must not alter the 5 second run. */
    reset_board(0xFFFFFE00U, -179.0f); single_click();
    REQUIRE(ModeControl_IsActive());
    start = s_start_ms;
    while ((uint32_t)(now_ms - start) < 5000U) advance(10);
    REQUIRE(ModeControl_GetStatus() == MODE_STATUS_DONE && s_elapsed_ms == 5000U);

    case_id = 12; /* Encoder signs, actual independent PI and PWM channel. */
    reset_board(0U, 0.0f);
    ModeControl_DebugCommand("M", 4.0f);
    ModeControl_DebugCommand("V", 0.3f);
    ModeControl_DebugCommand("RUN", 1.0f);
    MOTOR_A.Velocity_KP=MOTOR_B.Velocity_KP=1500;
    MOTOR_A.Velocity_KI=MOTOR_B.Velocity_KI=500;
    left_count = 10; right_count = -5; advance(10);
    REQUIRE(MOTOR_A.Encoder > MOTOR_B.Encoder && MOTOR_B.Encoder > 0.0f);
    REQUIRE(MOTOR_A.Motor_Pwm != MOTOR_B.Motor_Pwm);
    Set_Pwm(100000, -100000, 123, 456, 1600);
    REQUIRE(PWMA1 <= ACK_MOTOR_PWM_TOP && PWMA2 <= ACK_MOTOR_PWM_TOP);
    REQUIRE(PWMB1 <= ACK_MOTOR_PWM_TOP && PWMB2 <= ACK_MOTOR_PWM_TOP);
    REQUIRE(TIM8->CCR4 == 1600U && PWMC1 == PWMC2 && PWMD1 == PWMD2);
    case_id = 13; /* M3 independent targets, IMU independence and interlocks. */
    reset_board(0U, 0.0f);
    REQUIRE(ModeControl_DebugCommand("M", 4.0f));
    REQUIRE(!ModeControl_IsActive());
    s_valid = 0; send_imu = 0;
    REQUIRE(ModeControl_DebugCommand("RUN", 1.0f));
    advance(10);
    REQUIRE(MOTOR_A.Target == ACK_DEBUG_DEFAULT_SPEED_MPS && MOTOR_B.Target == ACK_DEBUG_DEFAULT_SPEED_MPS && Servo == 1576);
    REQUIRE(ModeControl_DebugCommand("L", 0.2f));
    REQUIRE(ModeControl_DebugCommand("R", -0.1f));
    REQUIRE(ModeControl_DebugCommand("A", 10.0f));
    advance(10);
    REQUIRE(MOTOR_A.Target == 0.2f && MOTOR_B.Target == -0.1f);
    REQUIRE(Servo == Ack_ServoPulse(10.0f * 0.01745329252f));
    advance(16000);
    REQUIRE(ModeControl_IsActive());
    REQUIRE(!ModeControl_DebugCommand("L", 11.0f));
    REQUIRE(!ModeControl_DebugCommand("A", 30.0f));
    REQUIRE(!ModeControl_DebugCommand("M", 2.5f));
    REQUIRE(ModeControl_DebugCommand("RUN", 0.0f));
    REQUIRE(MOTOR_A.Target == 0.0f && Servo == 1576 && !ModeControl_IsActive());
    REQUIRE(ModeControl_DebugCommand("RUN", 1.0f));
    Voltage = 9.0f; advance(10);
    REQUIRE(ModeControl_GetStatus() == MODE_STATUS_POWER_ERROR);
    Voltage = 12.0f; advance(100); REQUIRE(!ModeControl_IsActive());
    REQUIRE(ModeControl_DebugCommand("RUN", 1.0f));
    enable_input = 0; advance(10); REQUIRE(!ModeControl_IsActive());
    enable_input = 1;
    REQUIRE(ModeControl_DebugCommand("M", 1.0f));
    REQUIRE(!ModeControl_DebugCommand("RUN", 1.0f));
    REQUIRE(ModeControl_DebugCommand("V", 0.5f));
    REQUIRE(ModeControl_DebugLeft() == 0.5f && ModeControl_DebugRight() == 0.5f);
    REQUIRE(ModeControl_DebugCommand("M",4.0f));
    REQUIRE(ModeControl_DebugCommand("L",0.8f));
    REQUIRE(ModeControl_DebugCommand("R",-0.8f));
    REQUIRE(!ModeControl_DebugCommand("V",1.601f));
    REQUIRE(!ModeControl_DebugCommand("L",-1.601f));
    REQUIRE(ModeControl_DebugCommand("RUN",1.0f));
    advance(10);
    REQUIRE(MOTOR_A.Target == 0.8f && MOTOR_B.Target == -0.8f);
    double_click(); REQUIRE(s_mode == MODE_CONTROL_STRAIGHT_5S && !ModeControl_IsActive());
    case_id=14; /* Positional terms and DIANSAI post-calculation integral bound. */
    Ack_SpeedReset(&pi_a);
    REQUIRE(NEAR(Ack_SpeedUpdate(&pi_a,0.2f,0,1000,100),220,0.001f));
    REQUIRE(NEAR(pi_a.p_output,200,0.001f) && NEAR(pi_a.i_output,20,0.001f));
    REQUIRE(NEAR(Ack_SpeedUpdate(&pi_a,0.2f,0.1f,1000,100),130,0.001f));
    REQUIRE(NEAR(pi_a.p_output,100,0.001f) && NEAR(pi_a.i_output,30,0.001f));
    REQUIRE(NEAR(Ack_SpeedUpdate(&pi_a,0,0.1f,1000,100),-80,0.001f));
    Ack_SpeedReset(&pi_a);
    for(i=0;i<30;i++) Ack_SpeedUpdate(&pi_a,0.2f,0,1000,100);
    REQUIRE(pi_a.integral==1.5f && NEAR(pi_a.i_output,170,0.001f));
    REQUIRE(NEAR(pi_a.output,370,0.001f));
    /* Ki=0 contributes no I output, even with stored integral. */
    REQUIRE(NEAR(Ack_SpeedUpdate(&pi_a,0,0.02f,3000,0),-60,0.001f));
    REQUIRE(pi_a.i_output==0);
    REQUIRE(Ack_SpeedUpdate(&pi_a,0,0,3000,0)==0);
    Ack_SpeedReset(&pi_a);
    REQUIRE(Ack_SpeedUpdate(&pi_a,0,-10,10000,10000)==ACK_MOTOR_PWM_LIMIT);
    REQUIRE(pi_a.integral==1.5f);
    REQUIRE(Ack_SpeedUpdate(&pi_a,0,10,10000,10000)==-ACK_MOTOR_PWM_LIMIT);
    REQUIRE(pi_a.integral==-1.5f);
    Ack_SpeedReset(&pi_a);
    REQUIRE(pi_a.integral==0 && pi_a.p_output==0 && pi_a.i_output==0);
    finish(0U);
}
