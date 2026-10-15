# Demo1 当前工程

当前为四模式版本：M1 定向行驶 5 秒、M2 转向 90°、M3 视觉 PD＋Stanley 循迹、M4 VOFA 手动调试与距离数据显示。

- M1/M2 速度 0.3 m/s；M3 沿用 C30D 的 0.30 m/s；M4 两轮默认各 0.3 m/s。
- 所有模式的单轮目标速度上限 0.8 m/s。M4 VOFA 命令范围 ±0.8 m/s。
- USART1 PA9/PA10 接 Jetson V2 距离；USART2 PA2/PA3 接 IMU；USART3 PC10/PC11 接 VOFA。
- CubeMX 6.12.0：Demo1.ioc。Keil：MDK-ARM/Demo1.uvprojx。
- 最新固件：firmware/ACKERMANN_YAW.hex / .bin，BIN 地址 0x08000000；校验见 SHA256SUMS.txt。
- 编译日志：MDK-ARM/build_m3_vofa.log。实际 C 源码仿真 580 项断言通过。未实车验证。

完整使用说明见 README_四模式视觉与调试.md；VOFA 基础配置见 README_VOFA调试.md。
原 README_阿克曼使用说明.md 与较早测试日志是历史资料，模式、速度等以当前说明和源码为准。现有代码有手工修改，重新生成外设前应备份并核对启动顺序。
M3 已支持 VOFA 在线调参和 RUN=1; 授权启动，JustFloat 扩展为 34 通道；当前完整配置以 README_M3_VOFA调参.md 为准。
