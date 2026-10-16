# Demo1：使用 CubeMX 配置 FreeRTOS 任务

当前 `Demo1.ioc` 已启用 **FREERTOS / CMSIS_V2**。任务创建代码、任务优先级、静态栈和内核配置由 CubeMX 生成，业务代码保存在 `USER CODE` 区域中。

为匹配电脑上的 CubeMX 6.12.0 和 STM32Cube FW_F4 V1.28.2，内核改用该固件包自带的 **FreeRTOS V10.3.1 + CMSIS-RTOS V2**，替代此前手工接入的 V10.6.2。使用 ARMCC 5 / Cortex-M4F 硬件浮点移植层。

## 在 CubeMX 中修改任务

1. 关闭旧配置窗口，重新打开本目录的 `Demo1.ioc`。
2. 进入 **Middleware and Software Packs → FREERTOS**，Interface 应显示 **CMSIS_V2**。
3. 打开 **Tasks and Queues**，可看到以下两个任务：

| Task Name | Entry Function | Priority | Stack Size | Allocation |
|---|---|---|---:|---|
| vofaTask | StartVofaTask | osPriorityAboveNormal | 768 words（3072 字节） | Static |
| displayTask | StartDisplayTask | osPriorityNormal | 768 words（3072 字节） | Static |

4. 双击任务编辑栈大小、优先级等属性；新任务使用 Add 添加。
5. 保存 `.ioc`，点击 **GENERATE CODE**，然后在 Keil 中重新打开 `MDK-ARM/Demo1.uvprojx` 并 Rebuild。
6. 在 `Core/Src/freertos.c` 对应任务函数的 `USER CODE BEGIN/END` 区域里编写业务逻辑。新增任务必须按需调用 `osDelay()` 或阻塞等待，避免一直占用 CPU。

保留现有两个任务的入口函数名，以便 CubeMX 按用户区名称保留循环内容。任务周期写在任务函数中，不是 Tasks 表格的配置项。不要手工修改生成的 attributes 或 FreeRTOSConfig 宏后期待它们覆盖 `.ioc`；通过 CubeMX 修改相应配置，额外内容写进用户区。

**Project Manager → Code Generator → Keep User Code when re-generating** 已启用，请保持勾选。已有 Keil 自定义源码组和头文件路径也需要保留；正常重新生成已验证能保留这些设置。

## 执行分工

| 功能 | 位置 | 周期/优先级 |
|---|---|---|
| 编码器、M1～M4、轮速 PI、舵机与驱动输出 | TIM7 中断 | 10 ms，NVIC 2 |
| 视觉距离、IMU 接收 | USART1/2 中断 | NVIC 1 |
| VOFA 字节收发 | USART3 中断 | NVIC 3 |
| VOFA 命令解析、34 通道发送 | vofaTask | 每轮处理后 osDelay(2)，AboveNormal |
| OLED 与电压采样 | displayTask | 每轮处理后 osDelay(10)，Normal |
| HAL 毫秒时基 | TIM6 中断 | 1 ms，NVIC 15 |
| FreeRTOS 调度时基 | SysTick | 1 ms，NVIC 15 |

VOFA 与显示函数内部仍保留 50 ms 节流，目标约 20 Hz，实际间隔包含执行和调度耗时。任务优先级与 NVIC 优先级含义不同。

TIM6 为 HAL 独立时基，SysTick 交给 CMSIS-RTOS V2；原 TIM7 100 Hz 控制周期不变。`main.c` 中由 CubeMX 生成唯一的 `HAL_TIM_PeriodElapsedCallback()`，TIM6 推进 HAL 时间，用户区调用 `Balance_TimerCallback()`，后者只处理 TIM7。

现有高优先级串口/控制 ISR 不调用 RTOS API。在 CubeMX NVIC 中已明确关闭这些中断的 Uses FreeRTOS functions，并保留手动优先级；不可向这些 ISR 加入 FromISR API。保留原有 PRIMASK 临界区来保护与它们共享的数据。

M1～M4、串口引脚、VOFA 命令和 34 通道顺序不变。M3 默认速度 0.25 m/s，KP=5.45、KD=0.30、Stanley k=0.6、滤波时间常数 0.08 s；所有模式轮速目标仍限幅 ±0.8 m/s。

## 内存和故障处理

两应用任务使用静态栈；CMSIS-RTOS V2 配置同时支持动态/静态分配，内核动态堆为 16 KiB（heap_4），方便以后通过 CubeMX 添加对象。Idle 与 Timer Service 栈各 256 words；主栈为 4 KiB。栈溢出、内存分配失败、内核断言或任务创建失败都会进入关闭驱动输出的故障处理。

Keil Watch 可观察：

- `g_vofa_stack_free_words`、`g_display_stack_free_words`：运行期间的最小剩余栈空间，单位是 4 字节。
- `g_rtos_assert_file`、`g_rtos_assert_line`：故障来源。

## 验证与固件

已经使用本机 CubeMX 重新生成工程并检查用户代码保留情况。测试入口：

```text
python tests/check_project.py
python tests/run_tests.py --suite control --keil D:/keil
python tests/run_tests.py --suite tracking --keil D:/keil
python tests/run_tests.py --suite vision --keil D:/keil
python tests/run_tests.py --suite vofa --keil D:/keil
python tests/run_tests.py --suite oled --keil D:/keil
python tests/check_rtos.py
```

RTOS 检查读取最新 AXF 并执行入口和任务包装机器码；内核/硬件调用使用测试替身，不能代替实机异常切换与时序验证。烧录后先架空车轮检查 VOFA/OLED、M4 启停、M3 视觉超时停车及剩余栈空间。

Keil 输出仍为 `MDK-ARM/ACKERMANN_YAW/ACKERMANN_YAW.hex`；`firmware/` 保存本次构建的发布副本。以后自己重新生成和编译后，应使用新构建的 HEX 烧录，发布副本不会自动更新。
