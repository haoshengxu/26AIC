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
    checks.append(("SysTick preserved", "SysTick->" not in source("SYSTEM/delay/delay.c") and "HAL_IncTick();" in source("Core/Src/stm32f4xx_it.c")))
    checks.append(("No remote overwrite after task completion", "Get_RC" not in source("BALANCE/balance.c")))
    checks.append(("M1 is 5000 ms", bool(re.search(r"ACK_TASK1_DURATION_MS\s+5000U", config))))
    checks.append(("CubeMX servo period synchronized", "TIM8.Period=19999" in source("ACKERMANN_YAW.ioc")))
    project = ET.parse(ROOT / "MDK-ARM/ACKERMANN_YAW.uvprojx")
    paths = [element.text for element in project.findall(".//FilePath")]
    checks.append(("All Keil source files exist", all((ROOT / "MDK-ARM" / path.replace("\\", "/")).is_file() for path in paths)))
    checks.append(("Control module in Keil build", any("ackermann_control.c" in path for path in paths)))
    checks.append(("UART priority above TIM7", "HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);" in source("Core/Src/usart.c") and "HAL_NVIC_SetPriority(TIM7_IRQn, 2, 0);" in timer))
    report = "\n".join(f"{'PASS' if result else 'FAIL'}: {name}" for name, result in checks) + "\n"
    (ROOT / "tests/project_checks.txt").write_text(report, encoding="utf-8")
    print(report, end="")
    assert all(result for _, result in checks), "Project configuration mismatch"


if __name__ == "__main__":
    main()
