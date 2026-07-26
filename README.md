# Three-Phase Inverter Demo

基于 STM32F407VETx 的三相逆变器控制工程，使用 Keil MDK-ARM 和 STM32 HAL 库开发。

## 当前功能

- 20 kHz 三相互补 PWM 输出，包含死区配置
- 固定 50 Hz 角度发生器
- DQ 电压指令、逆 Park、逆 Clarke 与最值注入 SVPWM
- 电压软启动和软停止
- 两相电流采样与第三相电流重构
- 连续采样过流锁存和人工故障清除
- UART 参数命令和运行控制

DQ 单电压闭环尚未接入当前控制路径，设计与调试步骤记录在 `三相逆变器闭环规划.md` 中。

## 主要参数

| 参数 | 当前值 |
| --- | ---: |
| MCU | STM32F407VETx |
| 直流母线标称电压 | 60 V |
| PWM / 控制频率 | 20 kHz |
| 输出频率 | 50 Hz |
| 目标线电压有效值 | 32 V |
| 目标相电压峰值 | 26.128 V |
| 软件过流阈值 | 4.0 A，连续 3 次采样 |

## 工程结构

```text
APP/        Application control and scheduling
Component/  ADC, UART, filters, PID/PR and peripheral components
Core/       STM32CubeMX generated initialization and interrupt code
Drivers/    STM32 HAL and CMSIS dependencies
MDK-ARM/    Keil project files
```

## 构建

1. 使用 Keil MDK-ARM 5 打开 `MDK-ARM/Single_phase_Inverter.uvprojx`。
2. 选择工程目标并执行 Build。
3. 使用匹配 STM32F407VETx 的调试器下载固件。

如需重新生成外设初始化代码，可使用 STM32CubeMX 打开 `Single_phase_Inverter.ioc`。重新生成前应确认用户代码区和现有编码不会被覆盖。

## 串口命令

命令采用 `name=value` 格式，当前主要命令包括：

```text
wave8=1       Start inverter output
wave8=0       Request soft stop
clrfault=1    Clear a latched software fault
```

其余调试命令可在 `Component/Uart_cmp.c` 的命令表中查看。

## 注意事项

- 这是功率电子实验工程，上电前必须确认母线限流、驱动互锁、死区、采样极性和硬件过流保护有效。
- 当前 ADC 为软件启动的连续转换，尚未与 PWM 中点同步；提高闭环带宽前应先完成同步采样改造。
- 部分历史源文件包含 GBK 中文注释。编辑这些文件时应保持原编码，新增代码和注释建议仅使用 ASCII。
- 软件过流检测只能作为补充，不能替代硬件比较器和 TIM8 Break 关断。

## License

项目自有代码采用 [MIT License](LICENSE)。`Drivers/` 等第三方组件仍遵循其目录内原有许可证。
