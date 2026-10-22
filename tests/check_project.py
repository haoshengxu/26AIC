"""Check board configuration and Keil source references without hardware."""
from pathlib import Path
import re
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def source(name):
    return (ROOT / name).read_text(encoding="utf-8")


def main():
    checks = []
    timer = source("Core/Src/tim.c")
    timer8 = timer[timer.index("void MX_TIM8_Init"):timer.index("/* TIM9 init function */")]
    config = source("BALANCE/ackermann_config.h")
    checks.append(("TIM8 counter and 50 Hz period", "htim8.Init.Prescaler = 167;" in timer8 and "htim8.Init.Period = 19999;" in timer8))
    checks.append(("PC9 AF3 TIM8_CH4", "PC9     ------> TIM8_CH4" in timer and "GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;" in timer))
    checks.append(("Servo writes CCR4", "#define Servo_PWM TIM8->CCR4" in source("BALANCE/balance.h")))
    main_c = source("Core/Src/main.c")
    checks.append(("Controller init before control IRQ", main_c.index("Robot_Init(") < main_c.index("ModeControl_Init();") < main_c.index("HAL_TIM_Base_Start_IT(&htim7)")))
    checks.append(("Servo starts after centre initialization", main_c.index("ModeControl_Init();") < main_c.index("HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4)")))
    checks.append(("No early TIM7 start", "HAL_TIM_Base_Start_IT(&htim7)" not in timer))
    checks.append(("Fixed Ackermann selection", "Car_Mode = Akm_Car;" in main_c and "Robot_Select();" not in main_c))
    checks.append(("HAL timebase moved to TIM6", "SysTick->" not in source("SYSTEM/delay/delay.c") and "HAL_IncTick();" in main_c and "htim->Instance == TIM6" in main_c))
    checks.append(("No remote overwrite after task completion", "Get_RC" not in source("BALANCE/balance.c")))
    checks.append(("M1 is 5000 ms", bool(re.search(r"ACK_TASK1_DURATION_MS\s+5000U", config))))
    checks.append(("CubeMX servo period synchronized", "TIM8.Period=19999" in source("Demo1.ioc")))
    project = ET.parse(ROOT / "MDK-ARM/Demo1.uvprojx")
    paths = [element.text for element in project.findall(".//FilePath")]
    checks.append(("All Keil source files exist", all((ROOT / "MDK-ARM" / path.replace("\\", "/")).is_file() for path in paths)))
    checks.append(("Control module in Keil build", any("ackermann_control.c" in path for path in paths)))
    checks.append(("UART priority above TIM7", "HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);" in source("Core/Src/usart.c") and "HAL_NVIC_SetPriority(TIM7_IRQn, 2, 0);" in timer))
    ioc = source("Demo1.ioc")
    for timer_id in (2, 3):
        timer_block = timer[timer.index(f"void MX_TIM{timer_id}_Init"):timer.index(f"/* TIM{timer_id+1} init function */")]
        checks.append((f"TIM{timer_id} x4 encoder agrees with software and CubeMX",
            "sConfig.EncoderMode = TIM_ENCODERMODE_TI12;" in timer_block and
            f"TIM{timer_id}.EncoderMode=TIM_ENCODERMODE_TI12" in ioc and
            f"TIM{timer_id}.IPParameters=EncoderMode" in ioc and
            bool(re.search(r"EncoderMultiples\s+4\b", (ROOT / "BALANCE/robot_select_init.h").read_text(encoding="ascii", errors="ignore")))))
    rtos = source("Core/Src/freertos.c")
    checks.append(("CubeMX CMSIS V2 and user code preservation enabled", "VP_FREERTOS_VS_CMSIS_V2.Mode=CMSIS_V2" in ioc and "ProjectManager.KeepUserCode=true" in ioc))
    checks.append(("Generated task configuration matches IOC", all(value in ioc for value in ("vofaTask,32,768,StartVofaTask", "displayTask,24,768,StartDisplayTask")) and all(value in rtos for value in ("vofaTaskBuffer[ 768 ]", "displayTaskBuffer[ 768 ]", "osPriorityAboveNormal", "osPriorityNormal"))))
    checks.append(("Task business logic survives regeneration", "Vofa_Task();" in rtos and "show_task();" in rtos and "osDelay(2);" in rtos and "osDelay(10);" in rtos))
    checks.append(("Visual receiver starts after parser initialization", main_c.index("Vision_Init();") < main_c.index("HAL_UART_Receive_IT(&huart1")))
    checks.append(("Single HAL timer callback dispatches control", "void HAL_TIM_PeriodElapsedCallback" in main_c and "Balance_TimerCallback(htim);" in main_c and "void HAL_TIM_PeriodElapsedCallback" not in source("BALANCE/balance.c")))
    checks.append(("Generated scheduler startup order", main_c.index("osKernelInitialize();") < main_c.index("  MX_FREERTOS_Init();") < main_c.index("osKernelStart();")))
    report = "\n".join(f"{'PASS' if result else 'FAIL'}: {name}" for name, result in checks) + "\n"
    (ROOT / "tests/project_checks.txt").write_text(report, encoding="utf-8")
    print(report, end="")
    assert all(result for _, result in checks), "Project configuration mismatch"


if __name__ == "__main__":
    main()
