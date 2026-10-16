# 26AIC — Demo1 阿克曼视觉循迹底盘

STM32F407VET6，CubeMX 6.12.0，Keil ARMCC 5。

已接入由 CubeMX 管理的 FreeRTOS V10.3.1 / CMSIS-RTOS V2：VOFA 和显示分别运行于独立任务，电机控制保留 TIM7 的 100 Hz 中断周期。详见 [FreeRTOS 架构与使用说明](README_FreeRTOS.md)。

- M1：保持启动航向前进 5 秒，0.3 m/s。
- M2：前进转向 90°，0.3 m/s。
- M3：Jetson V2 距离输入，PD＋Stanley 循迹，VOFA 在线调参。
- M4：VOFA 独立调节左右轮速度、转角，同时显示视觉距离。
- 所有模式单轮目标速度限幅 ±0.8 m/s。

## M3 默认参数

| 参数 | 默认值 |
|---|---:|
| 循迹速度 | 0.25 m/s |
| Kp | 5.45 |
| Kd | 0.30 |
| Stanley k | 0.6 |
| 微分滤波时间常数 | 0.08 s |

默认值在 `BALANCE/tracking_config.h`；上电或 `TRESET=1;` 恢复上述数值。VOFA 修改仅保存在 RAM。

## 工程入口

- CubeMX：`Demo1.ioc`
- Keil：`MDK-ARM/Demo1.uvprojx`
- 固件：`firmware/ACKERMANN_YAW.hex`、`.bin`；BIN 烧录地址 `0x08000000`。
- 固件校验：`firmware/SHA256SUMS.txt`
- USART1 PA9/PA10：Jetson 距离输入；USART2 PA2/PA3：IMU；USART3 PC10/PC11：VOFA，均为 115200 8N1。

## 使用说明

- [当前工程概览](README_Demo1.md)
- [M3 VOFA 命令与全部 34 个通道](README_M3_VOFA调参.md)
- [四模式与视觉协议](README_四模式视觉与调试.md)

Keil 编译与实际 C 源码仿真已验证；测试脚本位于 `tests`，需要 Keil ARMCC 5、Python 和 unicorn 2.1.4。测试不代替实车调参、相机标定与串口联调。

CubeMX 已能配置任务并重新生成代码；业务逻辑放在 USER CODE 区，需保留 Keil 自定义源码组及包含路径。历史说明与当前参数冲突时，以本页和源码为准。
