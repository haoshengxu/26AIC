# V4 路线信息接收

> M3 已增加 V4 路线感知速度调节和纯追踪增益调参。使用方法、状态、默认值及新增 VOFA 通道见 [README_M3_V4循迹调试.md](README_M3_V4循迹调试.md)。

USART1，PA9/PA10，115200、8N1。字节流入口仍是 `Vision_ProcessByte()`。
兼容 V1/V2/V3 的 14 字节帧，以及 V4 的 22 字节帧；可以分包、粘连或混合版本。

## 帧与校验

`AA 55 04 16 seq[2] flags mode target_x[2] target_y[2] endpoint_x[2] endpoint_y[2] curvature[4] crc[2]`

多字节字段小端。CRC16/CCITT-FALSE：poly=0x1021，init=0xFFFF，无反射，xorout=0；覆盖前20字节。
坐标为 int16，单位0.01 cm，X向右、Y向前，原点为后轴中心。乘0.01得到cm，乘0.0001得到m。
曲率为IEEE754 float32绝对值，单位cm^-1，乘100得到m^-1；无左右方向信息。

`flags`：bit0目标有效，bit1末端坐标有效，bit2末端曲率有效，bit3第二采样点回退。
bit2要求bit1，bit3要求bit0；高四位必须为0。`mode`：0丢线/无新鲜路线，1双边，2左边界，3右边界；mode=0要求flags=0。
有效目标要求Y>0；末端XY保留真实有符号值，允许Y<=0。
有效曲率必须有限且非负，0可表示有效直道。未置有效位的字段忽略并在快照中清零，不沿用旧值。
接收端按字节合成整数及float位模式，用memcpy解码float，不使用未对齐结构体强转。
CRC或格式不合格时滑动1字节重新同步，不更新测量、接收时间和有效目标保持计时。
seq仅用于诊断，重启、重复及回绕不阻断有效帧接收。

## 获取最新路线遥测

```c
VisionTarget route;
Vision_GetTarget(&route); /* 原子快照；最后合法帧超过300ms后清除有效性与测量 */
if (route.link_alive && route.version == 4U) {
    /* route.mode：本帧实际寻线方式，与目标是否有效独立 */
    if (route.valid) {
        float x_m = route.x_centi_cm * 0.0001f;
        float y_m = route.y_centi_cm * 0.0001f;
        /* route.fallback：目标是否来自第二采样点 */
    }
    if (route.endpoint_valid) {
        float end_x_m = route.endpoint_x_centi_cm * 0.0001f;
        float end_y_m = route.endpoint_y_centi_cm * 0.0001f;
    }
    if (route.curvature_valid) {
        float abs_kappa_m_inv = route.endpoint_abs_curvature_cm_inv * 100.0f;
    }
}
```

结构中保留原始`flags`及解码后的布尔有效位。不能使用`flags==1`判断目标有效。
V3帧将version设为3，V4专有元数据全部清零。
收到全无效帧时，通信仍存在；mode和测量立即清零。
收到flags=6的帧时，目标无效但末端坐标和曲率仍可新鲜有效。
300ms诊断超时或UART错误会清除全部路线有效位、mode和测量；版本及计数留作诊断。

## M3控制接口与范围

`Vision_GetControlTarget()`提供最后一个有效目标及其同帧元数据。
V4目标进入纯追踪；直道增益TPGS默认0.95、弯道增益TPGC默认1.40，末端绝对曲率参与速度规划和增益分类，不直接叠加转向。
只有bit0有效的新合法帧更新控制目标、控制接收计数及时间；flags=7或15均可，第二点回退仍是有效目标。
仅末端有效的帧、全无效帧、CRC/格式错误均不刷新控制目标。
最后有效目标可保持不足1000ms，到达1000ms后失效停车；恢复有效数据会更新XY并重新计时。
UART错误及原有本地/远程停止条件仍立即停止。启动M3仍需在本地启动后收到新的有效目标。

注意：控制接口中的末端信息属于被保持的历史有效目标帧，不能当作最新路线。
读取新鲜末端信息必须使用`Vision_GetTarget()`并检查各自有效位。
末端曲率目前用于V4速度规划和双增益分类，不用于转向前馈、盲目加角度或根据末端点接管转向。
VOFA扩展为67通道，原I0～I33保留，新元数据、控制状态和参数追加到I34～I66。
V3继续使用旧X减速参数TXTH/TXK；V4使用曲率减速参数TCTH/TCK，详见调试说明。

## 验证

`tests/test_vision.c`包含文档四组原始完整报文、分包、混合版本、重同步、字段独立有效、负末端坐标、非法标志/模式、NaN/Inf/负曲率、超时与M3保持行为测试。
运行：`F:/PY/Python312/python.exe tests/run_tests.py --suite vision --keil D:/keil`。
仅完成接收代码与离线验证，实机串口联调需使用新编译固件；未自动烧录。
