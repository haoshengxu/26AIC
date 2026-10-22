#include "balance.h"
#include "mode_control.h"
#include "ackermann_control.h"

static AckSpeedPI s_left_pi;
static AckSpeedPI s_right_pi;

void Drive_Motor(float speed_mps, float unused_vy, float steer_rad)
{
    float left, right;
    (void)unused_vy;
    Ack_Kinematics(speed_mps, steer_rad, &left, &right);
    MOTOR_A.Target = left;
    MOTOR_B.Target = right;
    MOTOR_C.Target = 0.0f;
    MOTOR_D.Target = 0.0f;
    Servo = Ack_ServoPulse(steer_rad);
}

void SpeedPI_ResetWheel(uint8_t right)
{
    if (right) {
        Ack_SpeedReset(&s_right_pi);
        MOTOR_B.Motor_Pwm = 0.0f;
    } else {
        Ack_SpeedReset(&s_left_pi);
        MOTOR_A.Motor_Pwm = 0.0f;
    }
}

void SpeedPI_Reset(void)
{
    Ack_SpeedReset(&s_left_pi);
    Ack_SpeedReset(&s_right_pi);
    MOTOR_A.Motor_Pwm = 0.0f;
    MOTOR_B.Motor_Pwm = 0.0f;
    MOTOR_C.Motor_Pwm = 0.0f;
    MOTOR_D.Motor_Pwm = 0.0f;
}

void SpeedPI_GetState(AckSpeedPI *left, AckSpeedPI *right)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    *left = s_left_pi;
    *right = s_right_pi;
    __set_PRIMASK(mask);
}

int Speed_PI_A(float encoder, float target)
{
    return (int)Ack_SpeedUpdate(&s_left_pi, target, encoder,
                                MOTOR_A.Velocity_KP, MOTOR_A.Velocity_KI);
}

int Speed_PI_B(float encoder, float target)
{
    return (int)Ack_SpeedUpdate(&s_right_pi, target, encoder,
                                MOTOR_B.Velocity_KP, MOTOR_B.Velocity_KI);
}

void Balance_TimerCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM7) return;
    Get_Velocity_Form_Encoder();
    ModeControl_Tick10ms();
    if (ModeControl_GetStatus() == MODE_STATUS_RUNNING && !Turn_Off(Voltage)) {
        MOTOR_A.Motor_Pwm = Speed_PI_A(MOTOR_A.Encoder, MOTOR_A.Target);
        MOTOR_B.Motor_Pwm = Speed_PI_B(MOTOR_B.Encoder, MOTOR_B.Target);
        Set_Pwm(ACK_MOTOR_A_SIGN * (int)MOTOR_A.Motor_Pwm,
                ACK_MOTOR_B_SIGN * (int)MOTOR_B.Motor_Pwm, 0, 0, Servo);
    } else {
        SpeedPI_Reset();
        Set_Pwm(0, 0, 0, 0, SERVO_INIT);
    }
    Led_Flash(100);
}

void Set_Pwm(int motor_a, int motor_b, int motor_c, int motor_d, int servo)
{
    (void)motor_c;
    (void)motor_d;
    motor_a = target_limit_int(motor_a, -(int)ACK_MOTOR_PWM_LIMIT, (int)ACK_MOTOR_PWM_LIMIT);
    motor_b = target_limit_int(motor_b, -(int)ACK_MOTOR_PWM_LIMIT, (int)ACK_MOTOR_PWM_LIMIT);
    /* Keep the original two-input H bridge polarity. Zero = both inputs high. */
    if (motor_a < 0) {
        PWMA1 = ACK_MOTOR_PWM_TOP; PWMA2 = ACK_MOTOR_PWM_TOP + motor_a;
    } else {
        PWMA2 = ACK_MOTOR_PWM_TOP; PWMA1 = ACK_MOTOR_PWM_TOP - motor_a;
    }
    if (motor_b < 0) {
        PWMB1 = ACK_MOTOR_PWM_TOP; PWMB2 = ACK_MOTOR_PWM_TOP + motor_b;
    } else {
        PWMB2 = ACK_MOTOR_PWM_TOP; PWMB1 = ACK_MOTOR_PWM_TOP - motor_b;
    }
    /* C/D are unused on the two-drive-wheel chassis. */
    PWMC1 = ACK_MOTOR_PWM_TOP; PWMC2 = ACK_MOTOR_PWM_TOP;
    PWMD1 = ACK_MOTOR_PWM_TOP; PWMD2 = ACK_MOTOR_PWM_TOP;
    /* Permit M4's calibration pulses through the final hardware clamp. */
    Servo_PWM = target_limit_int(servo, ACK_DEBUG_SERVO_MIN_US, ACK_DEBUG_SERVO_MAX_US);
}

u8 Turn_Off(float voltage)
{
    return (voltage < ACK_MIN_BATTERY_V || EN == 0 || Flag_Stop != 0 || Check != 0) ? 1U : 0U;
}

void Get_Velocity_Form_Encoder(void)
{
    int left_count = Read_Encoder(2);
    int right_count = Read_Encoder(3);
    float scale = 0.0f;
    if (Encoder_precision > 0.0f)
        scale = CONTROL_FREQUENCY * Wheel_perimeter / Encoder_precision;
    MOTOR_A.Encoder = ACK_ENCODER_A_SIGN * left_count * scale;
    MOTOR_B.Encoder = ACK_ENCODER_B_SIGN * right_count * scale;
    MOTOR_C.Encoder = 0.0f;
    MOTOR_D.Encoder = 0.0f;
}

float target_limit_float(float value, float low, float high)
{
    return Ack_Clamp(value, low, high);
}
int target_limit_int(int value, int low, int high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}
float float_abs(float value) { return value < 0.0f ? -value : value; }
