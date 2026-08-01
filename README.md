# Three-Phase Inverter Demo

基于 STM32F407VETx 的三相离网逆变器控制工程，使用 Keil MDK-ARM、STM32 HAL 和 TIM8 高级定时器实现三相互补 SVPWM。本分支保留 DQ 开环和 DQ 单电压环，不再包含 DQ 电压电流级联双闭环。

> 当前默认模式为 DQ 开环。控制器直接使用电压参考生成 d/q 电压指令，并根据实时母线电压限制最终电压矢量。

## 当前功能

- 20 kHz 边沿对齐三相互补 PWM，包含死区和 5% 至 95% 占空比限制
- 50 Hz 同步旋转坐标系、Clarke/Park 与逆 Park 变换
- 最值公共模注入 SVPWM
- TIM8 CH4 在载波中点触发 ADC1/ADC2 同步采样
- 两相电流采样和第三相电流重构，用于三相软件过流保护
- DQ 开环和 DQ 单电压环两种编译模式
- 输出电压前馈、d/q 电压 PI 和电压矢量限幅
- 电压软启动、软停止、软件过流锁存和人工故障清除
- UART 参数调节和控制状态查询

详细设计、同步采样原理和调试记录见 [`三相逆变器闭环规划.md`](三相逆变器闭环规划.md)。

## 控制模式

控制模式在 [`Component/Dq_control.h`](Component/Dq_control.h) 中通过 `DQ_CONTROL_MODE` 编译期选择：

```c
#define DQ_OPEN_LOOP                 0U
#define DQ_VOLTAGE_LOOP              1U

#ifndef DQ_CONTROL_MODE
#define DQ_CONTROL_MODE              DQ_OPEN_LOOP
#endif
```

| 模式 | 行为 |
| --- | --- |
| `DQ_OPEN_LOOP` | `ud=vd_ref`、`uq=0`，保留软启停、SVPWM和保护，当前默认模式 |
| `DQ_VOLTAGE_LOOP` | 参考电压前馈加 d/q 电压 PI |

模式只能在编译时切换，不支持 UART 运行时切换。

## 单电压环结构

```text
vd_ref / vq_ref
       |
       v
d/q voltage PI + vd_ref feedforward
       |
       v
ud/uq                <= 0.9 * Vdc / sqrt(3)
       |
       v
Inverse Park + SVPWM + TIM8
```

当前控制器输出为：

```c
ud = vd_ref + voltage_pi(vd_ref - vd);
uq = voltage_pi(-vq);
```

工程保留现有离散 PI 形式：

```c
integral_candidate = integral + ki * kp * error;
output = kp * error + integral_candidate;
```

这里的 `Ki` 是工程使用的离散调节系数，不按连续域 `1/s` 参数解释。

## 主要参数

| 参数 | 当前值 |
| --- | ---: |
| MCU | STM32F407VETx |
| 直流母线控制输入 | ADC 实时采样值 |
| PWM / 控制频率 | 20 kHz |
| 输出基波频率 | 50 Hz |
| 默认线电压参考 | 32.0 Vrms |
| 默认相电压峰值参考 | 约 26.128 V |
| 单电压环 Kp / Ki | 0.028 / 0.008 |
| 最终电压矢量限制 | `0.9 * Vdc / sqrt(3)` |
| 软件过流阈值 | 5.0 A，连续 3 次采样 |

直流母线电压由 ADC 实时采样并送入控制器和 SVPWM。母线采样标定、零偏冻结和滤波实现在 `Component/Inverter_sampling.c`。

## 同步采样

TIM8 保持 20 kHz 边沿对齐 PWM，CH4 仅作为内部 ADC 触发源：

```text
TIM8 CNT reaches 4200
        -> CH4 OC4REF rising edge
        -> TIM8 TRGO
        -> ADC1 and ADC2 start one scan sequence
```

采样通道顺序：

```text
ADC1 Rank 1: Iw
ADC1 Rank 2: Uvw

ADC2 Rank 1: Iu
ADC2 Rank 2: Uuv
```

ADC 关闭连续转换，使用 TIM8 TRGO 上升沿启动；DMA 保持循环请求，但关闭 HT/TC 中断。TIM8 在停止发波和故障状态下继续计数，因此 ADC 仍能更新采样和偏置。

## 工程结构

```text
APP/                         Application control and scheduling
Component/Dq_control.c/.h    DQ open-loop and voltage-loop controller
Component/                   ADC, UART, filters and control components
Core/                        STM32CubeMX initialization and interrupts
Drivers/                     STM32 HAL and CMSIS dependencies
MDK-ARM/                     Keil project and build outputs
```

## 构建

1. 使用 Keil MDK-ARM 5 打开 `MDK-ARM/Single_phase_Inverter.uvprojx`。
2. 在 `Component/Dq_control.h` 中选择所需的 `DQ_CONTROL_MODE`。
3. 执行 Rebuild，并使用匹配 STM32F407VETx 的调试器下载固件。

本分支使用 ARMCC 5.06u7 全量构建默认 DQ 单电压环模式，结果为：

```text
0 Error(s), 0 Warning(s)
```

如需通过 STM32CubeMX 重新生成代码，请先确认用户代码区、TIM8 CH4、ADC 外部触发配置以及历史源文件编码不会被覆盖。

## 串口命令

USART1 当前波特率为 `460800`，命令格式为 `name=value`。

```text
wave8=1       Start inverter output
wave8=0       Request soft stop
clrfault=1    Clear a latched software fault
vref=3        Set 3 Vrms line-to-line reference
vp=0.035      Set shared d/q voltage-loop Kp
vi=0.006      Set shared d/q voltage-loop Ki
vstat=1       Print voltage-loop status
jf=1          Enable JustFloat Vdc output
jf=0          Disable JustFloat output
```

## 建议上板顺序

1. 编译 `DQ_OPEN_LOOP`，设置 `vref=3`，确认软启动、相序、SVPWM和保护行为。
2. 编译默认 `DQ_VOLTAGE_LOOP`，继续使用低电压参考和纯电阻负载，通过 `vstat=1` 检查 `vd/vq` 极性和电压指令。
3. 从较小的 `vp/vi` 开始调节单电压环，确认输出稳定后再逐步提高参考和负载。
4. 最后恢复额定参考和负载，检查 5 A 软件保护以及 TIM8 ISR 执行时间。

## 注意事项

- 这是功率电子实验工程。上电前必须确认母线限流、驱动互锁、死区、采样极性和硬件过流保护有效。
- 单电压环的编译和静态检查不能替代上板验证；首次调试必须使用低母线电压或受限电源、小电压参考和小增益。
- 软件过流检测只能作为补充，不能替代硬件比较器和 TIM8 Break 关断。
- 边沿对齐 PWM 的固定中点采样仍可能在约 50% 占空比附近靠近开关沿，需要实测电流噪声。
- 部分历史源文件使用 GBK/CP936 中文注释。新增源代码、代码注释、宏和 UART 文本应只使用英文 ASCII，避免转换旧文件编码。

## License

项目自有代码采用 [MIT License](LICENSE)。`Drivers/` 等第三方组件仍遵循其目录内原有许可证。
