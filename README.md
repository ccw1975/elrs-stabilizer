# ExpressLRS 固定翼自稳接收机固件

基于 **ExpressLRS 3.6.4** 定制的 2.4GHz 接收机固件，在**接收机端直接实现固定翼横滚姿态自稳**（无需飞控）。目标硬件为 **ESP32-C3 Zero + MPU6500 + SX1280（E28 2G4M）**。

> 本项目是 ExpressLRS 的衍生作品，遵循 **GPLv3** 许可证（详见 [LICENSE](LICENSE)）。

## 功能特性

- **三档介入模式**（由 CH5 三档开关实时切换，不存档）：
  - **直通（Bypass）**：纯杆量，自稳完全退出（默认安全档）
  - **Rate**：横滚纯角速度阻尼，松杆刹停旋转、不强制回平（更跟手）
  - **Angle**：横滚回平，目标坡度跟随摇杆，收敛柔和
- **俯仰全模式直通**：升降舵纯杆量，不介入（解决俯仰抢杆导致"不跟手"的问题）
- **两档参数完全独立**：Angle 用 `Roll Kp/Kd`，Rate 用 `Rate Gain/Limit/Dead`，各调各的
- **垂直姿态连续处理**：Angle 档按重力参考可靠度对角度项连续软衰减，机头接近竖直时自动退化为纯阻尼（方向永远正确），不会因欧拉角万向节锁定而乱打舵
- **机型支持**：普通布局（1 或 2 个副翼舵机）+ 飞翼（只做横滚差动，PWM1=+roll / PWM2=−roll，两片反向）
- **LUA 菜单调参 + CRSF Attitude 遥测**：遥控器上实时查看姿态角与 CH1 输出
- **三级安全**：低档直通 / 传感器异常旁路 / 信号丢失旁路

## 硬件

| 部件 | 型号 |
|---|---|
| MCU | ESP32-C3 Zero |
| IMU | MPU6500（I2C） |
| 射频 | SX1280（E28 2G4M 模块），2.4GHz |
| 硬件定义 | `hardware/RX/DIY C3 2400 PWM.json` |

## 编译与烧录

使用 **PlatformIO**（建议 6.2.0）：

```powershell
pio run -e Unified_ESP32C3_2400_RX_via_UART
```

产物：`.pio/build/Unified_ESP32C3_2400_RX_via_UART/firmware.bin`

也可用 ELRS Web Flasher 直接烧录该固件。**烧录前请保持机体静止**——陀螺零偏在每次上电时静态校准（约 100ms）。

## 快速上手

1. 烧录后上电，遥控器进入 `ExpressLRS LUA → Stabilizer` 菜单，确认 **IMU = 1**
2. 用三档开关切到 **Angle**（高位），地面手持飞机倾斜：
   - 静观 CH1 输出（遥测 `Yaw` 字段）是否随机体姿态反向变化 → 验证 IMU 修正生效
   - 若"打杆方向正确、自稳方向反了"→ 把 **Roll Dir** 拨到 `Rev`
3. 切到 **Rate**（中档），快速绕机头轴转一下再松手：应感到阻力刹停旋转，但不自动回平
4. 低档始终为直通（含无信号时），任何时候切回它即可完全退出自稳

> 首次飞行务必地面测试修正方向，方向反了会变成正反馈！

## LUA 参数（Stabilizer 菜单，共 11 项）

| 参数 | 说明 | 默认 |
|---|---|---|
| Airframe | Normal / Wing（飞翼只差动介入） | Normal |
| Aileron | 1 / 2 副翼舵机 | 1 Servo |
| Gain Switch | 模式开关通道（CH1~CH12） | CH5 |
| IMU | 传感器状态（只读） | — |
| Roll Kp | **Angle** 横滚比例增益（×0.1） | 10（=1.0） |
| Roll Kd | **Angle** 横滚阻尼增益（×0.01） | 5（=0.05） |
| Rate Gain | **Rate** 阻尼增益（×0.1，us 每 °/s） | 8（=0.8） |
| Rate Limit | **Rate** 修正限幅（us） | 180（≈±18% 行程） |
| Rate Dead | **Rate** 死区（°/s，滤抖舵） | 8 |
| Roll Dir | 横滚修正方向（Normal / Rev） | Normal |
| 6CH Rev | 普通布局双副翼时 CH6 直通反向（做差动） | Normal |

**Rate 档对症调参**：

| 现象 | 处理 |
|---|---|
| 左右晃动、过冲 | 先降 `Rate Gain`（0.8→0.5），再降 `Rate Limit`（180→120） |
| 刹不住、停得慢 | 加 `Rate Gain`（0.8→1.2）或 `Rate Limit` |
| 静止/小动作抖舵 | 加 `Rate Dead`（8→12 °/s） |
| 舵面动作突兀 | 降 `Rate Limit` |

（输出低通 α=0.35 为固件固定值，约 28Hz 截止，不占菜单。）

## 遥测

固件回传 **CRSF Attitude** 帧（10Hz），EdgeTX 端执行 `Discover Sensors` 即可出现 Attitude 传感器：

- `Roll` / `Pitch`：姿态角（**弧度**，读角度请 ×57.3）
- `Yaw`：被复用来回传 **CH1 实际 PWM 输出**（含自稳修正，988~2012us）——用于验证修正是否叠加

此外还有标准 Link Statistics（RSSI / SNR / LQ）。

## 文档

- [Stabilizer_使用文档.txt](Stabilizer_使用文档.txt) — 完整使用说明与调参建议
- [MODIFICATION_LOG_20260825.md](MODIFICATION_LOG_20260825.md) — 历次修改记录（修改点 22/23/24）
- [STABILIZER_DESIGN.md](STABILIZER_DESIGN.md) — 设计说明

## 许可证

```
Copyright (C) 2007 Free Software Foundation, Inc.
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
```

本项目基于 [ExpressLRS](https://github.com/ExpressLRS/ExpressLRS) 3.6.4 定制，完整条款见 [LICENSE](LICENSE)。
