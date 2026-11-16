# 修改日志：LUA 只读状态参数（IMU 调试回传）

- 日期：2026-08-25 11:13
- 目的：将 IMU 工作状态与实时姿态角通过 LUA 参数回传遥控器，无需 USB 线即可在遥控器 `Stabilizer` 菜单下判断 MPU6050/6500 是否工作
- 涉及文件：仅 1 个 —— `lib/LUA/rx_devLUA.cpp`

---

## 修改点 1：引入头文件

**文件**：`lib/LUA/rx_devLUA.cpp`（第 7 行，`#include` 区）

```cpp
// 原：无这行
#include "deferred.h"
// 新增 ↓
#include "stabilizer.h"
```

**说明**：需要引用 `stabilizer` 全局实例（其声明在 `stabilizer.h`）。

**回退**：删除第 7 行 `#include "stabilizer.h"`。

---

## 修改点 2：新增 3 个只读参数定义

**文件**：`lib/LUA/rx_devLUA.cpp`（第 270~305 行，`luaStabPitchKd` 定义之后、`luaparamStabAirframe` 函数之前）

```cpp
// ---- 调试状态参数（只读，event() 中实时刷新，供遥控器 LUA 菜单查看）----
static struct luaItem_int8 luaStabImuStat = {
  {"IMU", CRSF_UINT8},
  {
    {
      (uint8_t)0,       // value: 0=未就绪 1=正常
      (uint8_t)0,       // min
      (uint8_t)1,       // max
    }
  },
  STR_EMPTYSPACE
};

static struct luaItem_int8 luaStabRoll = {
  {"Roll", CRSF_INT8},
  {
    {
      (uint8_t)0,       // value（度，补码）
      (uint8_t)129,     // min = -127（0x81 补码，遥控器按 INT8 解释）
      (uint8_t)127,     // max = +127
    }
  },
  "deg"
};

static struct luaItem_int8 luaStabPitch = {
  {"Pitch", CRSF_INT8},
  {
    {
      (uint8_t)0,       // value（度，补码）
      (uint8_t)129,     // min = -127（0x81 补码，遥控器按 INT8 解释）
      (uint8_t)127,     // max = +127
    }
  },
  "deg"
};
```

> **注意（编译修复，11:36）**：Roll/Pitch 最初写的是 `(int8_t)0/-127/127`，但 `luaItem_int8.properties` 的匿名 union 首个成员是 `u`（uint8_t）视图，GCC 报 `-Wnarrowing` 编译失败。由于 `luaInt8StructToArray()` 是整块 memcpy 原始字节、遥控器按 `CRSF_INT8` 解释为有符号，改为用 `u` 视图 + 补码（`0x81` = -127）即可，功能等价。已编译通过（RAM 18.6% / Flash 56.0%）。

**回退**：删除从 `// ---- 调试状态参数` 注释到 `luaStabPitch` 定义结束（即 `};`）之间的全部内容。

---

## 修改点 3：注册参数（不带回调 = 只读）

**文件**：`lib/LUA/rx_devLUA.cpp`（`registerLuaParameters()` 内，Stabilizer 注册区，第 769~772 行）

```cpp
    registerLUAParameter(&luaStabPitchKd, &luaparamStabPitchKd, luaStabilizerFolder.common.id);
    // 调试状态参数：不注册回调 → 只读（遥控器侧修改会被忽略）   ← 新增注释
    registerLUAParameter(&luaStabImuStat, nullptr, luaStabilizerFolder.common.id);  // ← 新增
    registerLUAParameter(&luaStabRoll, nullptr, luaStabilizerFolder.common.id);     // ← 新增
    registerLUAParameter(&luaStabPitch, nullptr, luaStabilizerFolder.common.id);    // ← 新增
```

**回退**：删除注释行和 3 行 `registerLUAParameter`。

---

## 修改点 4：`timeout()` 中周期性刷新参数值

**文件**：`lib/LUA/rx_devLUA.cpp`（`timeout()` 函数内、`luaHandleUpdateParameter()` 之前，第 889~902 行）

```cpp
static int timeout()
{
#if defined(STABILIZER_ENABLE) && defined(GPIO_PIN_PWM_OUTPUTS)
  if (OPT_HAS_SERVO_OUTPUT)
  {
    // 调试状态：IMU 健康标志 + 实时姿态角（未初始化时保持 0）
    // event() 只在连接状态变化等事件时触发，动态值必须在这里周期性刷新
    // （连接时 interval/250 ms，未连接 1s），遥控器 LUA 菜单读取时即为最新值
    setLuaUint8Value(&luaStabImuStat, stabilizer.imuOk() ? 1 : 0);                   // ← 新增
    setLuaInt8Value(&luaStabRoll, (int8_t)constrain((int32_t)stabilizer.roll(), (int32_t)-127, (int32_t)127));  // ← 新增
    setLuaInt8Value(&luaStabPitch, (int8_t)constrain((int32_t)stabilizer.pitch(), (int32_t)-127, (int32_t)127)); // ← 新增
  }
#endif
  luaHandleUpdateParameter();
  ...
}
```

> **注意（11:41 修正）**：最初把刷新放在 `event()` 里，但排查 `lib/DEVICE/device.cpp` 后发现 `event()` 只在「eventFired 置位 / 连接状态变化 / model match 变化」时触发（`start()` 时仅一次），**不是周期性调用**，陀螺仪读数会变成"事件时刻快照"而不更新。已挪到周期性运行的 `timeout()`（连接时每 `interval/250` ms 刷新一次），并置于 `luaHandleUpdateParameter()` 之前保证遥控器读取到最新值。已编译通过。

**回退**：删除 `timeout()` 中 `#if defined(STABILIZER_ENABLE)...#endif` 包裹的 3 行 `setLua*Value`。

---

## 修改点 5：IMU 驱动 I2C 地址回退（0x68 ↔ 0x69）

**文件**：`lib/STABILIZER/imu_mpu6500.cpp`（`begin()` 函数，第 32~52 行）

**背景**：MPU6050/MPU6500 只有 `0x68` 与 `0x69` 两个 I2C 地址，由模块 AD0 引脚电平决定（接地=0x68，接高=0x69）。原实现只试默认 `0x68`，若模块 AD0 被拉高则检测失败、`imuOk()` 恒为 false（遥控器 `IMU` 参数显示 0）。

**修改**：`begin()` 内先试传入地址（默认 0x68），WHO_AM_I 校验失败则回退到另一地址（`address ^= 0x01`，0x68↔0x69），两次都失败才返回 false：

```cpp
    bool detected = false;
    for (uint8_t attempt = 0; attempt < 2; ++attempt)
    {
        m_address = address;
        uint8_t whoami = 0;
        if (readBytes(MPU_REG_WHO_AM_I, &whoami, 1) &&
            (whoami == MPU6050_WHO_AM_I_VAL || whoami == MPU6500_WHO_AM_I_VAL))
        {
            m_whoami = whoami;
            detected = true;
            break;
        }
        address ^= 0x01; // 0x68 <-> 0x69
    }

    if (!detected)
    {
        return false;
    }
```

**说明**：若 `begin()` 被显式传入 `0x69`，同样会先试 0x69、失败再回退 0x68，对称兼容。已编译通过（13:40）。

**回退**：恢复 `begin()` 为原单地址逻辑（只设 `m_address` 后直接读 WHO_AM_I，失败即返回）。

---

## 修改点 6：姿态环防卡顿降频（未连接/传感器异常时）

**文件**：`lib/STABILIZER/stabilizer.cpp`（`devTimeout()` 函数，第 285~300 行）

**背景**：`devTimeout()` 返回 `DURATION_IMMEDIATELY`（=0），device 框架每主循环调用一次；内部 `update()` 原本无条件按 2ms 节流跑 I2C 读取。隐患：ESP32 Wire 在 I2C 总线挂死（如 SDA 被拉低）时一次传输最多阻塞 ~50ms 超时。若传感器故障仍每 2ms 强读，主循环会被拖慢到 50ms/轮 → 无线收包超时断连 → 而 `isActive()` 要求 `connectionState == connected`，**断连后自稳反而不介入**（"卡住让 IMU 不能介入"的路径）。

**修改**：仅「已连接 + 传感器健康」才跑 500Hz；否则降频到 100ms 一次（保持 healthy 检测与姿态跟踪）：

```cpp
    uint32_t interval = STAB_LOOP_INTERVAL_MS;
    if (connectionState != connected || !stabilizer.imuOk())
    {
        interval = 100;
    }
    if (now - lastUpdate >= interval)
    {
        lastUpdate = now;
        stabilizer.update();
    }
```

**说明**：未连接时姿态环本就无输出（`isActive()` 恒 false），100ms 足够维持陀螺仪零点跟踪；连接瞬间 Mahony 用真实 dt（上限 0.1s）继续，精度损失可忽略。已编译通过（13:58）。

**回退**：恢复 `devTimeout()` 为无条件 2ms 节流的原版。

---

## 修改点 7（关键修复）：`Stabilizer_device` 补 `.start` —— 姿态环从未运行的根因

**文件**：`lib/STABILIZER/stabilizer.cpp`（`Stabilizer_device` 定义，第 300~312 行）

**现象**（用户实测反馈）：烧录后遥控器 `IMU` 显示 1，但 `Roll/Pitch` 恒为 0，晃动模块数值不动。

**根因**：`Stabilizer_device` 注册时 `.start = nullptr`。查看 `lib/DEVICE/device.cpp`：

- `devicesStart()`（第 86~96 行）：`deviceTimeout[i] = 0xFFFFFFFF;`，仅当 `.start` 非空才调用并写入实际时间
- `_devicesUpdate()`（第 153~157 行）：`if (now >= deviceTimeout[i])` 才调用 `timeout()`

`.start` 为空 → `deviceTimeout[i]` 恒为 `0xFFFFFFFF` → `now >= 0xFFFFFFFF` 永远为假（除非 millis 回绕）→ **`devTimeout()` 从未被调用 → `update()` 从不执行**。而 `m_imu.healthy()` 在 `begin()` 成功即置 true 且无 `readRaw` 失败将其清零 → IMU 显示 1，`m_roll/m_pitch` 保持初始 0。所有其他 ELRS device（ADC/AnalogVbat/Baro/Button 等）均带 `.start`，唯独本模块缺失。

**修复**：

```cpp
static int devStart()
{
    return DURATION_IMMEDIATELY;   // 立即进入 timeout 调度（每 loop 一次）
}

device_t Stabilizer_device = {
    .initialize = devInit,
    .start = devStart,             // ← 新增，此前为 nullptr
    .event = nullptr,
    .timeout = devTimeout,
};
```

修复后 `deviceTimeout[i] = now + 0`，`timeout()` 每 loop 被调，`devTimeout()` 内 2ms 节流 → 500Hz 姿态环正常运转。已编译通过（08:47）。

**回退**：把 `.start = devStart` 恢复为 `.start = nullptr` 并删除 `devStart()`。

---

## 修改点 8：IMU 数据"有显示但不动"的排查与防御性修复

**现象**（用户实测反馈）：烧录修改点 7 后，遥控器 `Stabilizer` 菜单已能显示 `Roll`/`Pitch`（单位 deg），但晃动模块数值始终不变。

**根因分析**：`Mahony` 的 `mahonyStep()` 开头对加速度计归一化 `norm < 1e-6f` 时**整个函数直接 return**——连陀螺仪积分也一并跳过。若 I2C 读到全 0 数据（如传感器未真正退出睡眠/配置寄存器写入失败），`ax=ay=az=0 → norm=0` → 四元数永久冻结在初始值 → `Roll/Pitch` 恒 0 且不动。而 `begin()` 中所有 `writeByte()` 返回值被忽略——WHO_AM_I 是 ROM 寄存器，睡眠状态下仍可读（IMU=1），但数据寄存器输出全 0，造成"IMU 正常但数据不动"的假象。

**修改内容**（3 处）：

1. **`lib/STABILIZER/imu_mpu6500.cpp`** `begin()`：逐个检查 `writeByte()` 返回值，任一关键配置（复位/退睡眠/DLPF/量程）写入失败立即返回 `false`。若传感器未真正唤醒，IMU 参数会明确显示 0，不再"假健康"。

2. **`lib/STABILIZER/stabilizer.cpp`** `mahonyStep()`：重构为「加速度计有有效输出时才做加速度修正，陀螺仪积分无条件执行」——即使加速度计数据全 0 或自由落体，姿态仍跟随陀螺仪，不会永久冻结（标准 Mahony 实现方式）。

3. **诊断参数**（`lib/STABILIZER/stabilizer.h/.cpp` + `lib/LUA/rx_devLUA.cpp`）：`Stabilizer` 保存最近一次原始读数（`m_rawGx/m_rawGy/m_rawGz/m_rawAz`）并暴露 getter；LUA `Stabilizer` 菜单新增 4 个只读 INT16 参数：
   - `GyroX` / `GyroY` / `GyroZ`（dps，±2000）
   - `AccZ`（mg，±16000）
   
   晃动模块时观察这些值即可一锤定音：GyroX 等全为 0 → 传感器数据无效（硬件/唤醒问题）；Gyro 有值但 Roll/Pitch 不动 → 解算问题（可回退本修改点 2 排查）。

**说明**：已编译通过（09:06）。烧录后优先看 `GyroX/GyroY/GyroZ/AccZ`——静止时 `AccZ` 应约 +1000（1g），晃动模块 `GyroX/GyroY/GyroZ` 应有明显变化。

**回退**：
- 修改 1：恢复 `begin()` 不检查 `writeByte()` 返回值。
- 修改 2：恢复 `mahonyStep()` 开头 `norm<1e-6` 直接 return 的原版。
- 修改 3：删除 stabilizer.h/.cpp 中 `m_raw*` 成员与 getter；删除 rx_devLUA.cpp 中 4 个 `luaStabGyro*/luaStabAccZ` 定义、注册与 `setLuaInt16Value` 上报。

---

## 修改点 9：姿态环/读取链路诊断计数器（区分"update 没跑 / readRaw 失败 / 数据真不变"）

**现象**（用户实测反馈）：修改点 8 烧录后 `AccZ=996`（≈1g，静止正常），`GyroX/Y/Z` 也都有数据，但晃动模块时**所有值都不变**。这排除了"传感器数据全 0 冻结"（修改点 8 的根因），说明 `update()` 至少成功跑过一次（`m_rawAz=996` 只在 readRaw 成功时写入），但之后数据链路疑似断掉——可能是 update() 不再被调度、或 readRaw 持续失败、或 I2C 读到静态数据。

**排查结论**（代码审查）：调度链（`devicesUpdate` → `devTimeout` 每 loop 返回 `DURATION_IMMEDIATELY` → 内部 2ms/100ms 节流调 `update()`）、LUA 上报链（`timeout()` → `setLuaInt16Value` → `luaHandleUpdateParameter()` 在遥控器请求时发送）代码均正确，无法静态定位，需运行时证据。

**修改内容**（3 处，纯新增诊断、不影响功能）：

1. **`lib/STABILIZER/stabilizer.h`**：新增成员 `m_updateCount`（update 调用累计）、`m_readErrCount`（readRaw 失败累计）及 getter `updateCount()`/`readErrCount()`。

2. **`lib/STABILIZER/stabilizer.cpp`** `update()`：函数入口 `++m_updateCount`（无论成败都计数）；`readRaw` 失败分支 `++m_readErrCount`。

3. **`lib/LUA/rx_devLUA.cpp`**：`Stabilizer` 菜单新增 2 个只读 INT16 参数：
   - `UpdCnt`：update() 调用次数
   - `RdErr`：readRaw 累计失败次数

**烧录后判读**（打开 `Stabilizer` 菜单，晃动模块观察 30 秒）：

| UpdCnt | RdErr | Gyro/AccZ | 结论与下一步 |
|:--|:--|:--|:--|
| 持续增长 | 0 | 不变 | update 在跑、readRaw 成功但读到静态数据 → I2C/传感器硬件冻结（总线时序、模块供电、传感器损坏），查硬件 |
| 持续增长 | 增长 | 不变 | readRaw 持续失败 → I2C 总线异常（上拉电阻、接线、地址冲突），查硬件 |
| 不变（停在小值） | 0 | 不变 | **update() 未被调度** → 回到代码侧：查 `devicesStart`/`devTimeout` 调用链 |
| 不变 | 增长 | 不变 | update 不被调度 + readRaw 失败 → 双重问题 |

**说明**：已编译通过（09:27，RAM 18.6% / Flash 56.0%）。这是诊断版本，拿到 UpdCnt/RdErr 表现后即可定位根因；定位后此修改点可整体回退（删除 2 个 LUA 参数与 2 个计数器即可）。

**回退**：删除 stabilizer.h 中 `m_updateCount/m_readErrCount` 成员与 getter；删除 stabilizer.cpp `update()` 中两处计数；删除 rx_devLUA.cpp 中 `luaStabUpdCnt/luaStabRdErr` 定义、注册与上报。

---

## 修改点 10：实时姿态遥测（CRSF ATTITUDE，10Hz 推送遥控器）

**结论**（修改点 9 诊断闭环）：用户实测 `UpdCnt=9653`（持续增长）、`RdErr=0`（readRaw 100% 成功），且"编辑参数退出后所有 LUA 数据都会更新、晃动时数值变化"——证明**传感器、读取链路、姿态环全部正常**。之前所有"数据不变"均为 **ELRS LUA 参数页的拉模式显示机制**：`elrsV3.lua` 的字段读取请求只在进入页面/滚动/编辑退出时发送（`loadQ` 非空才 push），静止页面不主动刷新，看到的都是快照。

**需求**：调试稳定器需要实时看姿态角，LUA 快照不适合。方案：接收机通过 OTA 遥测向遥控器推送 **CRSF ATTITUDE 帧**（0x1E），EdgeTX 自动识别 "Attitude" 传感器实时显示 Roll/Pitch（可加姿态仪 widget），不依赖 LUA 拉模式。

**修改内容**（仅 `lib/STABILIZER/stabilizer.cpp`）：

1. include `telemetry.h`；`extern Telemetry telemetry;`（rx_main.cpp 的全局遥测队列）。
2. 新增 `sendAttitudeTelemetry()`：构造 `CRSF_MK_FRAME_T(crsf_sensor_attitude_t)` 帧（roll/pitch = 弧度×10000，大端字节序，与 MAVLink.cpp 的 htobe16 一致；yaw 置 0），`CRSF::SetHeaderAndCrc` 打包后 `telemetry.AppendTelemetryPackage()` 入队。队列内已有 ATTITUDE 帧会覆盖不堆积。
3. `devTimeout()` 中：`connectionState == connected` 时每 100ms（10Hz）发送一次；断开自动停发。

**烧录后查看**：遥控器 → 遥测设置 → **Discover Sensors**（或手动添加 `Attitude` 传感器），打开遥测显示：晃动模块，`Roll`/`Pitch` 实时变化（静止归零）；也可在遥测屏添加 **"姿态指示器" widget** 可视化飞机姿态。若数值静止不动，检查 1:2 遥测率是否被其他占用。

**说明**：已编译通过（09:39，RAM 18.6% / Flash 56.1%）。修改点 9 的诊断参数保留（Stabilizer LUA 菜单仍可看 UpdCnt/RdErr），不需要可整体删除。

**回退**：删除 stabilizer.cpp 中 `#include "telemetry.h"`、`extern Telemetry telemetry;`、`sendAttitudeTelemetry()` 及 `devTimeout()` 中 10Hz 发送块。

---

## 修改点 11：Stabilizer LUA 菜单精简 + Yaw 说明

**需求**：姿态已改走实时遥测（修改点 10，用户实测 Roll/Pitch 实时变化），LUA 菜单里的显示/诊断项已无意义。精简菜单：只保留 **IMU 状态 + 4 个 PDI 参数**（Roll/Pitch 的 Kp/Kd）+ **机身布局/开关通道配置**，删除其余。Yaw 不关心（实时显示恒 0），去除相关处理。

**修改内容**：

1. `lib/LUA/rx_devLUA.cpp`：
   - 保留（可配置）：`luaStabAirframe`（普通/飞翼）、`luaStabAileron`（单/双副翼）、`luaStabGainCh`（增稳开关通道 CH1-CH12）。**注意：这三项先删后恢复（09:59-10:06），最终全部保留，与删除前行为一致**。
   - 删除：`luaStabRoll`/`luaStabPitch`（姿态快照，已被实时遥测替代）、`luaStabGyroX/Y/Z`、`luaStabAccZ`（原始数据诊断，问题已闭环）、`luaStabUpdCnt`/`luaStabRdErr`（修改点 9 诊断计数）。
   - 保留：`luaStabImuStat`（IMU 健康 0/1，timeout() 周期刷新）+ `luaStabRollKp/RollKd/PitchKp/PitchKd`。
   - 最终菜单 8 项：`Airframe`、`Aileron`、`Gain Switch`、`IMU`、`Roll Kp`、`Roll Kd`、`Pitch Kp`、`Pitch Kd`。

2. `lib/STABILIZER/stabilizer.cpp`：`sendAttitudeTelemetry()` 中 yaw 恒置 0 保持不变（**CRSF ATTITUDE 帧格式固定含 roll/pitch/yaw 三字段，协议层无法删除 yaw**），更新注释说明：EdgeTX 端只需在遥测屏添加 Roll/Pitch 传感器项、不添加 Yaw 即不显示。

**说明**：已编译通过（09:59，RAM 18.6% / Flash 56.0%）。config.h 中 stab 字段与 Set/Get 函数保留（EEPROM 布局稳定 + stabilize 代码仍使用 GetStabAirframe/GainCh 等），仅 LUA 侧移除。

**回退**：git 方式查看 `git diff lib/LUA/rx_devLUA.cpp lib/STABILIZER/stabilizer.cpp` 后手工恢复被删参数项即可（定义+注册+event 回读+timeout 刷新）。

---

## 修改点 12：Yaw 字段复用为 CH1 输出回传（验证 IMU 参与通道调整）

**需求**：用户不需要 yaw（航向恒 0，不关心），想看 IMU 是否真实参与了通道数据调整。CRSF ATTITUDE 帧格式固定含 roll/pitch/yaw 三字段无法删除 → 将 **yaw 字段复用为 CH1（PWM1）实际输出 PWM 回传**（us，988-2012，已含自稳修正）。

**修改内容**：

1. `lib/ServoOutput/devServoOutput.h/.cpp`：新增 `uint16_t servoOutputGetValue(uint8_t ch)`，返回 `pwmChannelValues[ch]`（servoWrite 写入的最终输出，已含 `stabilizerApply` 修正；未初始化/无效时回退 1500 中位，避免遥测出现 65535）。

2. `lib/STABILIZER/stabilizer.cpp`：`sendAttitudeTelemetry()` 中 `crsfatt.p.yaw` 由恒 0 改为 `servoOutputGetValue(0)`（大端字节序），并更新注释。

**验证方法**：EdgeTX 遥测屏添加 Yaw 传感器项（它现在显示 CH1 输出 PWM）。摇杆固定不动，机体转个角度：
- 开关通道拨到高位（自稳介入）：Yaw 值随机体姿态反向变化 → IMU 修正真实叠加到 CH1 输出；
- 开关拨到低位/旁路：Yaw 值恒等于摇杆对应 PWM（不动）。
- 不添加 Yaw 传感器项即完全不显示（等同"取消 yaw"）。

**说明**：已编译通过（10:15，RAM 18.6% / Flash 56.0%）。

**回退**：将 `crsfatt.p.yaw` 改回 `0`，删除 `servoOutputGetValue` 及 `#include "devServoOutput.h"` 即可。

---

## 修改点 13：Mahony 收敛慢修复（晃动停手后 10s 才稳定）

**现象**：用户晃动传感器后停止，Roll/Pitch 要 ~10 秒才稳定。

**根因**：`STAB_MAHONY_KP=1.0`（时间常数≈1s）。晃动时加速度计被线加速度污染，姿态被拉偏几十度；停手后需 3~5 个时间常数（5~10s）才完全回拉。Ki=0 无积分，无法加速收敛。

**修改内容**：

1. `lib/STABILIZER/stabilizer.h`：`STAB_MAHONY_KP` 1.0 → **3.0**（静止满权重，时间常数≈0.33s，1~2s 内稳定）。
2. `lib/STABILIZER/stabilizer.cpp` `mahonyStep()`：新增**运动检测调制加速度修正权重**：
   - 角速度（补偿前，rad/s）`>15°/s` → 权重 10%（晃动/机动时加速度计不可信，姿态靠陀螺仪积分，不被拉偏）；
   - `≤2°/s` → 权重 100%（静止时加速度计满强度拉回，收敛快）；
   - 之间线性过渡。飞行中震动也不会把姿态噪声串入（权重自动压低）。

**说明**：已编译通过（10:37，RAM 18.6% / Flash 56.0%）。预期效果：晃动停手后 1~2s 内姿态稳定归位；晃动过程中姿态更平滑（不再被线加速度拉偏）。

**回退**：`STAB_MAHONY_KP` 改回 1.0，删除 mahonyStep 中 accelWeight/gyroMag 相关 5 行，`gx/gy/gz +=` 恢复直接使用 `STAB_MAHONY_KP`。

---

## 修改点 14：Roll Dir / Pitch Dir 轴方向开关（独立于 Invert）

**需求**：用户担心"遥控器方向都调好了、但自稳叠加方向反了"时无法单独修正。讨论结论：Invert 是"直通+修正"联动翻转，做不到只翻修正；传感器装反（180°/上下颠倒）是"摇杆对、自稳反"的最常见根源，且无 IMU 安装方向补偿 → 新增**按轴的 IMU 修正方向开关**（按轴而非按通道：传感器装反是整轴现象，且差动副翼/飞翼混控的结构符号由代码自动保持）。

**修改内容**：

1. `lib/CONFIG/config.h`：位域 `stabUnused:2` 复用于 `stabRollDir:1` / `stabPitchDir:1`（0=正向 1=反向，EEPROM 字节布局不变，旧配置兼容，默认 0 即正向）；新增 `GetStabRollDir/GetStabPitchDir`、`SetStabRollDir/SetStabPitchDir`。
2. `lib/CONFIG/config.cpp`：`SetStabilizerDefaults()` 补 `stabRollDir=0`、`stabPitchDir=0`；新增两个 setter。
3. `lib/STABILIZER/stabilizer.cpp` `computeCorrections()`：算出 `m_corrRollUs/m_corrPitchUs` 后按轴开关取反。**翻转发生在逻辑轴源头** → 普通布局 CH1(+roll)/CH6(−roll 差动)、飞翼 CH1/CH2(pitch±roll 混控) 全部自动继承，结构符号不受影响。
4. `lib/LUA/rx_devLUA.cpp`：新增 `Roll Dir` / `Pitch Dir` 两个下拉项（Normal/Rev），含注册回调与 event 回读刷新，位于 Pitch Kd 之后、IMU 状态之前。

**使用场景**：
- 传感器上下颠倒（Roll+Pitch 同时反）→ 两个都拨 Rev；
- 只有单轴反（如侧装）→ 只拨对应轴；
- 与 Invert 互不干扰：Invert 治舵机接线，轴开关治传感器装姿。

**说明**：已编译通过（12:10，RAM 18.6% / Flash 56.0%——两 bit 复用原有 unused，资源占用不变）。

**回退**：`config.h` 位域恢复 `stabUnused:2`，删除 getter/setter，`stabilizer.cpp` 删除 computeCorrections 末尾两处取反块，`rx_devLUA.cpp` 删除两个菜单项及注册/回读。

---

## 修改点 15：姿态遥测震荡修复（三级低通降噪）

**现象**：用户反馈遥测 Roll/Pitch 数值震荡（来回摆动）；重新烧录修改前版本同样震荡 → 非修改点 14 引入，根源在修改点 13。

**根因分析**：
1. **accelWeight 下限 0.1 太狠**：震动/晃动时加速度计受线加速度污染最严重（可达 0.1~0.5g，等效姿态误差 6°~27°），但旧逻辑仍保留 10% 修正权重（kp=0.3），把线加速度当成姿态误差拉偏 → 与 PD 控制环互相作用产生来回摆动；
2. **KP=3.0 放大加速度计噪声**：静止满权重 kp=3.0，加速度计 ±16g 量程噪声经 Mahony 环路增益进姿态角；
3. **陀螺仪高频噪声积分**：DLPF 92Hz 带宽、500Hz 采样，高频噪声直接积分进姿态角，并串入 PD 的 Kd 项；
4. **权重阈值临界抖振**：gyroMag 在 2°/s~15°/s 阈值带内时 accelWeight 剧烈变化（0.1↔1.0），姿态修正强度忽强忽弱。

**修改内容**（`lib/STABILIZER/stabilizer.h` + `stabilizer.cpp`）：
1. `STAB_MAHONY_KP` 3.0 → **2.0**（静止收敛≈0.5s，对加速度噪声增益适中，回拉仍比 1.0 快一倍）；
2. accelWeight 下限 0.1 → **0.0**：晃动/震动时完全信任陀螺仪积分，加速度计零介入，杜绝污染拉偏；
3. accelWeight 做**一阶低通平滑**（`STAB_WEIGHT_LPF_A=0.10`，约 8Hz）：防阈值附近反复切换抖振；
4. **陀螺仪一阶低通**（`STAB_GYRO_LPF_A=0.25`，500Hz 下截止约 26Hz）：降积分噪声 + PD Kd 项噪声；真实机动 <10Hz 不受影响；
5. **遥测姿态角一阶低通**（`STAB_ATT_LPF_A=0.20`，约 16Hz）：`roll()/pitch()` 返回低通平滑值，**仅影响显示，PD 控制环仍用原始角**（无相位延迟损失）。

**说明**：已编译通过（13:40，RAM 18.6% / Flash 56.1%，4 个 float 状态变量 +6B RAM 级）。

> ⚠ **实测失败（13:47 用户反馈）**：比修改前更震荡、"稳定不住"（修改前约 7~8s 能稳定）。根因：**accelWeight 下限 0.0** 使晃动/微振时加速度计零介入，陀螺零偏漂移无人拉回，且只要 gyroMag 未低于 2°/s 阈值权重就一直为 0 → 姿态永不收敛；陀螺低通又使 gyroMag 恢复滞后、权重回升更慢。已由修改点 16 回退并换方案。

**回退**：`STAB_MAHONY_KP` 改回 3.0，accelWeight 下限改回 0.1 并删除低通平滑（`kp` 直接乘 `accelWeight`），删除 update() 中陀螺低通与姿态角低通 5 行，`roll()/pitch()` 恢复返回 `m_roll/m_pitch`，`stabilizer.h` 删除三个滤波系数宏及三个成员变量。

---

## 修改点 16：回退修改点 15 激进项 + 启用 Ki 积分收敛（实测震荡修复）

**背景**：修改点 15 实测更震荡（"稳定不住"）。本次恢复修改点 15 之前的稳定基线，仅保留"遥测显示低通"，并新增标准 Mahony **Ki 积分项**治本（消除陀螺零偏漂移、加速静止收敛）。

**修改内容**：
1. `lib/STABILIZER/stabilizer.h`：
   - `STAB_MAHONY_KP` 恢复 **3.0**；删除 `STAB_GYRO_LPF_A`、`STAB_WEIGHT_LPF_A` 两个宏；
   - `STAB_MAHONY_KI` 0.0 → **0.1**（静止时积分消除陀螺零偏引起的长期漂移，加速最终收敛；运动时随权重缩放）；
   - 保留 `STAB_ATT_LPF_A=0.20` 遥测显示低通与 `m_attLpf[2]`，删除 `m_gyroLpf[3]`、`m_accelWeightLpf` 两个成员；
   - `roll()/pitch()` 仍返回低通平滑值（仅显示）。
2. `lib/STABILIZER/stabilizer.cpp`：
   - `update()` 删除陀螺一阶低通 3 行（低通使 gyroMag 恢复滞后 → 权重回升慢，加剧"稳定不住"）；
   - `mahonyStep()`：accelWeight 下限恢复 **0.1**（晃动时保留少量修正锚住姿态）；删除权重一阶低通；**积分项移到 accelWeight 计算之后**并改为 `ki = STAB_MAHONY_KI × accelWeight`（运动时积分几乎冻结，避免大机动错误累积）；
   - `calibrateGyro()` 删除 m_gyroLpf 初始化。

**原理**：震荡本质是"晃动停手后姿态被拉偏、靠加速度计慢慢拉回（7~8s）"——kp 项负责拉回但对抗的是陀螺零偏造成的持续漂移；Ki=0.1 用积分直接消除零偏项，静止收敛更快且不放大高频噪声；运动时 Ki 随权重缩小到 1/10，不会把线加速度污染积进去。

**说明**：已编译通过（13:50，RAM 18.6% / Flash 56.1%）。

**回退**：`STAB_MAHONY_KI` 改回 0.0，`m_integralFB` 恢复不参与（或删除积分块），`STAB_ATT_LPF_A` 宏与 `m_attLpf` 成员、`roll()/pitch()` 低通一并删除即回到修改点 14 状态。

---

## 修改点 17：姿态遥测读数现象排查结论（无代码改动，仅文档修正）

**现象**（用户实测，14:21）：转动传感器后报告——roll 翻 180° 数值 0→-3→跳到 3→回 0；pitch 水平转垂直 0→1.5→回 0（反向 0→-1.4→回 0）；pitch 接近 90° 时 roll 快速涨到 ±3。用户质疑是否故障。

**排查结论**：全部为**正常行为，无代码 bug**，未改动稳定器代码：
1. **单位是弧度**：CRSF ATTITUDE 协议（EdgeTX `crossfire.cpp`：`CS(ATTITUDE_ID,...,UNIT_RADIANS,3)`，`value/10` 后按 `rad×1000` 精度 3 显示）固定显示弧度。当前发送 `rad×10000` 符合协议。90°=1.57、180°=3.14 → 用户看到的 1.5/3 就是弧度，×57.3 即角度。文档原先写"（度）"与实际不符，已修正并补充读数说明。
2. **roll ±180° 回绕**：`atan2` 定义域 [-180°,+180°]，转 180° 时从 -3.14 跳到 +3.14 是边界回绕，正常。
3. **pitch ±90° 反射**：`asin` 定义域仅 ±90°，超过 90° 读数反射回落，正常。
4. **万向节锁定（Gimbal Lock）**：pitch 接近 ±90° 时 roll 读数爆炸到 ±180°（`roll=atan2(2(qw·qx+qy·qz),...)` 分子/分母退化），是欧拉角表示法的固有特性，所有欧拉角飞控都有。自稳包线 ±30°，正常飞行到不了 90°，不影响控制。

**改动**：仅更新 `Stabilizer_使用文档.txt` 第三节（遥测单位=弧度、回绕/反射/万向节锁定的正常现象说明、×57.3 换算、姿态仪 widget 正确）。

**后续可选**（未实施，待用户决定）：若需数值显示为角度，CRSF 协议层无法改（EdgeTX 单位固定为弧度）；可在 LUA 菜单恢复快照显示（但 int8 仅 ±127，且修改点 11 已精简移除）；另可在 `computeCorrections` 加"|pitch|>85° 时 roll 修正置 0"防御（防万向节锁定时 roll 修正饱和硬打舵），属包线外保护，风险低但非必需。

---

## 修改点 18：万向节锁定防御（B 方案实施，|pitch|≥85° 时 roll 修正置 0）

**背景**：修改点 17 排查确认 pitch 接近 ±90° 时 roll 读数爆到 ±180° 是欧拉角万向节锁定的固有现象（显示无碍）。但用户实测确认：此时控制环的 roll 修正同样会爆到饱和（±30° 限幅硬打舵）。虽然正常飞行到不了 90°，作为包线外保护，用户选择实施 B 方案。

**修改内容**：
1. `lib/STABILIZER/stabilizer.h`：新增宏 `STAB_GIMBAL_LOCK_GUARD_DEG  85.0f`（阈值远在自稳包线 ±30° 外，正常飞行永不触发）。
2. `lib/STABILIZER/stabilizer.cpp` `computeCorrections()`：PD 计算后、限幅前加入——
   ```cpp
   if (fabsf(m_pitch) >= STAB_GIMBAL_LOCK_GUARD_DEG) { corrRollDeg = 0.0f; }
   ```
   - 使用**控制环原始角 `m_pitch`**（非遥测低通值），与 PD 同一姿态源；
   - 只清零 roll 修正；pitch 由 `asin` 在 ±90° 内仍正确，俯仰自稳保留；
   - 普通布局与飞翼布局均生效（roll 修正清零后，飞翼 mix `pitch±roll` 只留 pitch）。

**行为**：|pitch| ≥ 85° 时 roll 轴自稳失效（修正 0，直通摇杆），回落到阈值内自动恢复。属极端姿态保护，不影响正常飞行包线内的自稳性能。

**说明**：已编译通过（14:36，lint 无报错）。

**回退**：删除 `stabilizer.h` 的 `STAB_GIMBAL_LOCK_GUARD_DEG` 宏与 `computeCorrections()` 中的 `if (fabsf(m_pitch) >= ...)` 3 行即回到修改点 17 状态。

---

## 修改点 19：倒扣段自稳整体冻结（档位2，筋斗保护）

**背景**：修改点 18 的 85° 阈值只保护奇点两侧窄带（物理 85°~95° / 265°~275°），对筋斗的主体——倒扣段（物理俯仰 90°~270°）无效：该段欧拉 pitch 读数反射回 85° 以内、roll 读数恒 ±180°，roll 修正照常饱和打舵、pitch 修正方向相反。用户实测确认并选择"档位 2"：倒扣段整个自稳修正清零。

**关键认识**（用户讨论后确认的数学事实）：
- 欧拉角提取的 roll 在 |pitch|<90° 时**精确等于真实横滚角**（`atan2(sinφ·cosθ, cosφ·cosθ)=φ`），90° 是唯一奇点，不存在渐进影响；
- "倒扣态"在欧拉角表示里**从物理 90° 翻过瞬间即成立**（物理 95° 时欧拉 roll 已跳 ±180°），持续到 270°，不是 180° 那一点；
- 机体 z 轴重力分量 `gravZ=q0²−q1²−q2²+q3²` 的符号在 **90°/270° 处过零**，与万向节锁定区间精确对齐，是可靠的"机腹朝上"判据（`gzE<0`），不依赖欧拉角。

**修改内容**：
1. `lib/STABILIZER/stabilizer.cpp` `computeCorrections()`：在 85° 阈值判断之后新增——
   ```cpp
   float gravZ = m_q[0]*m_q[0] - m_q[1]*m_q[1] - m_q[2]*m_q[2] + m_q[3]*m_q[3];
   if (gravZ < 0.0f)
   {
       corrRollDeg = 0.0f;
       corrPitchDeg = 0.0f;
   }
   ```
   - `gravZ` 与 `mahonyStep()` 中的 `gzE` 同式，直接从四元数算，500Hz 下开销可忽略；
   - 保留修改点 18 的 85° 阈值（只清 roll），两层叠加覆盖物理俯仰 85°~275° 全程无空窗：
     - 85°~90°（已接近奇点、未倒扣）：85° 阈值清 roll；
     - 90°~95°（已倒扣+奇点窄带）：两层同时触发；
     - 95°~265°（倒扣主体）：`gravZ<0` 清 roll+pitch；
     - 265°~275°（奇点窄带+将出倒扣）：两层同时触发；
     - 275° 之后：欧拉 |pitch| 仍 ≥85° 期间由 85° 阈值继续清 roll 直至远离奇点。
2. `lib/STABILIZER/stabilizer.h`：更新 `STAB_GIMBAL_LOCK_GUARD_DEG` 注释，说明两层防御。

**行为**：物理俯仰进入 85°~275° 区间后 roll 修正始终为 0；进入倒扣段（机腹朝上）后 roll、pitch 修正全部为 0，自稳完全退出、纯手动直通（筋斗等特技动作不受干扰），回落到正常区间自动恢复。正常飞行包线 ±30° 内行为与之前完全一致。

**说明**：lint 无报错；已编译验证通过（15:00，RAM 18.6% / Flash 56.1%）。

**回退**：删除 `computeCorrections()` 中 `gravZ` 计算行与 `if (gravZ < 0.0f) {...}` 块即回到修改点 18 状态（保留 85° 阈值）。

---

## 修改点 20：大仰角 roll 自稳提前让路（阈值 85°→70°，逐步测试中）

**背景**：修改点 19 的倒扣防御实测生效（垂直后自稳退出），但用户反馈"接近垂直之前 roll 仍有介入，这是不希望的"，希望预防措施更早生效；具体角度用户无法精确读出。经讨论确认这是**理念选择**（大仰角区 roll 让路，避免接近垂直时横滚自稳介入），非噪声乱动问题（60°~80° 欧拉角仍良态、读数可靠，放弃纯属主动取舍）。用户采用**逐步测试法**：先定 70° 实测，效果不佳再改 60°，故当前值为 70°。

**修改**：
1. `lib/STABILIZER/stabilizer.h`：`STAB_GIMBAL_LOCK_GUARD_DEG` **85.0 → 70.0**（宏名保留），注释同步更新；
2. `lib/STABILIZER/stabilizer.cpp`：`computeCorrections()` 逻辑不变（用宏），注释更新为 70° 描述。

**覆盖区间**（物理俯仰角）：
- 0°~70°：自稳完全正常（包线 ±30° + 修正 ±30°，日常飞行到不了 70°）；
- 70°~90°：\|pitch\|≥70° → roll 修正清 0（pitch 修正保留，继续压机头防竖立）；
- 90°~270°：倒扣段（gravZ<0）→ roll+pitch 全清；
- 270°~290°：\|pitch\|≥70° → roll 修正清 0；
- 290°~360°：自稳正常。

**取舍**：70° 时噪声放大 ×2.9，读数仍可靠；提前放弃 70°~85° 区间的 roll 自稳（含部分螺旋抑制能力），换取用户明确期望的"大仰角 roll 让路"。70° 远高于正常机动范围，日常飞行不受影响。

**调整备选**：若 70° 实测仍嫌"介入偏晚"，宏改 **60.0**（放大 ×2，覆盖 60°~300°，让路更早）；若想尽量保留大仰角自稳，可改 75.0~80.0（放大 ×3.9~×5.8）。仅改宏一个值，倒扣段逻辑不受影响。

**说明**：lint 无报错（06:38）。

**回退**：宏改回 85.0 即回到修改点 19 状态。

---

## 修改点 21：6CH Rev —— 普通布局双副翼 CH6 直通反向（**已恢复/二次实现，2026-08-30 16:0x**）

**背景**：用户调试实测发现，普通布局 + 2 副翼舵机（Aileron=2 Servo）时，CH1/CH6 两个副翼舵面同方向（未差动）。用户需要"在此模式下给 6 通道反向"。通道映射页 Invert 是"直通+修正一起翻"（Invert 后直通差动但修正由 `-roll` 变 `+roll` 同向，自稳错），不能满足"直通差动 + 修正差动同时成立"。故新增"只翻直通、修正保持差动"的独立开关。

**第一次实现（15:12 编译）实测无效 → 已回退（15:3x）→ 本次（16:0x）按方案 A 重新实现并排查。**

**修改（当前代码，与第一次实现相同）**：
1. `lib/CONFIG/config.h`：
   - `rx_config_t` 末尾追加 `stab6chRev`（uint8_t，0=正向 1=反向）；
   - 自稳配置标记升级：新增 `STABILIZER_CONFIG_MAGIC_V2(0xA6)`，保留 V1(0xA5) 用于升级识别；
   - 新增 `GetStab6chRev()` / `SetStab6chRev()`。
2. `lib/CONFIG/config.cpp`：
   - `SetStabilizerDefaults()`：magic 改 V2，`stab6chRev=0`；
   - `Load()`：magic==V1 时只补 `stab6chRev=0` 并升到 V2，保留其余自稳参数不重置；其他不匹配则整段重置；
   - 实现 `SetStab6chRev()`。
3. `lib/LUA/rx_devLUA.cpp`：新增菜单项 `6CH Rev`（Normal/Rev，默认 Normal），注册 + event 刷新。
4. `lib/ServoOutput/devServoOutput.cpp` `servosUpdate()`：在通道 Invert 之后、自稳修正叠加之前——
   ```cpp
   if (ch == 5 && config.GetStabAirframe() == STAB_AIRFRAME_NORMAL &&
       config.GetStabAileron() == STAB_AILERON_2SERVO && config.GetStab6chRev())
   {
       us = 3000U - us;   // 仅直通绕中位反向
   }
   ```
   条件限普通布局+2 舵机+开关开启，其他情况完全无影响；仅作用于 PWM6(CH6)，不碰 CH1。
5. `Stabilizer_使用文档.txt`：菜单恢复 11 项，新增 [11] 6CH Rev 条目（含生效条件与互斥说明）。

**行为**：Aileron=2 Servo 且 6CH Rev=Rev 时，CH6 直通信号绕中位反向，左右副翼差动（摇杆向右 → CH1 一方向、CH6 反方向）；自稳修正仍为 CH6=-roll 差动，方向与直通一致。与 Roll Dir/Pitch Dir（只翻修正）、通道 Invert（直通+修正一起翻）三者在逻辑上独立。

**第一次实测无效的排查结论（恢复后务必核对）**：
1. **烧错固件（最可能）**：6CH Rev 是 15:12 才编译的，构建目录里 8/26 的 `V2.bin` 根本没有此功能。若当时烧的是旧固件自然无反应。本次必须烧**最新编译产物**。
2. **LUA 三条件未齐**：必须 `Airframe=Normal` + `Aileron=2 Servo` + `6CH Rev=Rev` 同时成立；Aileron 是 1 Servo 时 CH6 无差动修正，开关无意义。
3. **通道位置**：左副翼舵机必须插在 PWM6 输出口（ch==5，硬件 json 确认 GPIO21）。
4. **与映射页 Invert 互斥**：若 CH6 在映射页勾了 Invert，则 Invert 先把直通翻一次、6CH Rev 再翻一次 → 双重翻转抵消（等于没翻）；同时 Invert 使修正由 `-roll` 变 `+roll` → 修正也错。两者混用 = 开关完全失效。用 6CH Rev 时 CH6 的 Invert 必须保持 Normal。
5. **烧录后配置重置**：新固件 flash-discriminator 变化会重置 UID/绑定/自稳配置，烧完需重新设置 LUA 参数。

**当前状态**：修改点 21 功能已重新实现，与第一次实现代码一致；配置标记 V2(0xA6)；编译验证待执行。

---

## 修改点 22：取消姿态越界防御——自稳全程持续介入（2026-09-02）

**背景**：修改点 18/19/20 依次加入的"万向节锁定防御 + 倒扣段自稳冻结 + 大仰角 roll 提前让路"(覆盖物理俯仰 70°~290°)是按用户此前实测要求实施的。用户现在改主意:取消倒扣不介入,让自稳在任意姿态下都一直介入。

**修改内容**：
1. `lib/STABILIZER/stabilizer.cpp` `computeCorrections()`：删除 `gravZ` 计算与两段清零——
   ```cpp
   // 已删除（修改点 22）：
   // float gravZ = ...;
   // if (fabsf(m_pitch) >= STAB_GIMBAL_LOCK_GUARD_DEG) { corrRollDeg = 0.0f; }
   // if (gravZ < 0.0f) { corrRollDeg = 0.0f; corrPitchDeg = 0.0f; }
   ```
   替换为说明注释。PD 修正现在只在后续限幅处截断,不再按姿态越界退出。
2. `lib/STABILIZER/stabilizer.h`：删除宏 `STAB_GIMBAL_LOCK_GUARD_DEG` 与相关注释（无其他引用点，已全仓确认）。
3. `Stabilizer_使用文档.txt`：同步删除"两层包线外防御"描述,改为"已取消、全程介入"。

**行为**：|pitch| 与机腹朝向不再影响修正。接近 ±90° 垂直时欧拉 roll 读数 ±180°、pitch 反射,倒扣段两轴 PD 修正按正飞目标域(roll 目标 0°)计算——**方向与饱和行为可能不符合倒扣物理直觉**(这正是修改点 18~20 加防御的原因),需低空实测验证;GainCh 开关可随时切回直通。

**回退**：恢复修改点 20 状态 = 在 PD 限幅前加回
```cpp
float gravZ = m_q[0]*m_q[0] - m_q[1]*m_q[1] - m_q[2]*m_q[2] + m_q[3]*m_q[3];
if (fabsf(m_pitch) >= 70.0f) { corrRollDeg = 0.0f; }
if (gravZ < 0.0f) { corrRollDeg = 0.0f; corrPitchDeg = 0.0f; }
```
并在头文件补回宏即可。

**说明**：lint 待查、编译待验证（本次未编译）。

---

## 修改点 23：三模式（直通/Rate/Angle）+ 俯仰全模式直通 + 仅横滚介入（2026-09-06）

**背景**：试飞反馈——俯仰介入导致飞机不跟手（升降舵被自稳抢杆），横滚介入对平衡有用。用户要求：
1. 只保留横滚轴介入，俯仰全模式直通（升降舵纯杆量）；
2. 介入方式分三档（CH5 模式开关实时判定、不存档）：直通 / Rate（纯阻尼、不强制回平、更跟手）/ Angle（回平）；
3. 支持普通布局（1/2 副翼舵机）与飞翼（只差动介入、两片反向）。

**重要纠错**：原讨论曾设想"Angle 改用重力投影角 `atan2(gyE,gzE)` 消除万向节锁定"。实现时核对公式发现：欧拉 roll 分子恰为 gyE、分母归一化后恰为 gzE，二者**数学恒等**；竖立爆炸是物理极限（机头竖直时机体绕机头轴旋转不改变重力投影，绝对横滚不可观测），非公式选择问题。故不换角度，改用"重力参考可靠度衰减"。

**修改内容**：
1. `lib/STABILIZER/stabilizer.h`：
   - 新增 `StabMode_e` 枚举（BYPASS/RATE/ANGLE）与三档阈值宏（`STAB_MODE_LOW_BOUND=660`、`STAB_MODE_HIGH_BOUND=1320`）；
   - Angle 收敛系数 `STAB_ANGLE_SOFTEN=0.7`（回平更柔和、不抢杆）；
   - 重力参考可靠度下限 `STAB_GRAV_REF_FLOOR=0.30`（约 |pitch|72° 起角度项让位）；
   - Rate 增益 `STAB_RATE_GAIN=2.0`（200°/s≈满修正）与限幅 `STAB_RATE_CORR_LIMIT_US=410`（±40% 满行程）；
   - 新增 `getMode()`，`computeCorrections()` 签名改单参（去掉 gyroY）。
2. `lib/STABILIZER/stabilizer.cpp`：
   - `isActive()`：低档（含无信号）不介入；`getMode()` 按 GainCh 三档实时判定（不存档）；
   - `computeCorrections()` 三分支：直通=0；Rate=纯阻尼 `−ω×2.0` 限 ±410us；Angle=PD 但角度项乘可靠度 `constrain((|gzE|−0.30)/0.70, 0, 1)`（阻尼项不受影响，竖直段自动退化纯阻尼）；
   - 俯仰修正恒 0（`m_corrPitchUs` 清 0），`applyCorrection()` 不变：普通 CH1=+roll、CH6=-roll（2 舵）；飞翼 PWM1=+roll、PWM2=-roll（俯仰分量恒 0 → 纯差动）；
   - 注释同步（含"重力投影角恒等于欧拉"的纠错记录）。
3. `lib/LUA/rx_devLUA.cpp`：Pitch Kp/Kd/Pitch Dir 从菜单隐藏（定义与回调注释保留，config 字段不删避免动 EEPROM 布局）；菜单说明更新为三模式。
4. `lib/CONFIG/config.h`：注释同步（stabGainCh=模式开关三档；stabRollKp/Kd 语义注明 Angle/Rate 用途；stabPitchKp/Kd/stabPitchDir 保留字段不再使用）。
5. `Stabilizer_使用文档.txt`：全文同步（三模式、菜单 8 项、Rate/Angle 验证方法、可靠度衰减说明、版本号）。

**行为**：
- 模式开关低=直通（默认安全档）、中=Rate、高=Angle，实时切换不存档；
- 俯仰任何模式都不介入 → 升降舵跟手；
- Rate 档任何姿态方向正确（纯陀螺阻尼），竖直段无 gimbal lock 问题；
- Angle 档平飞/正常坡度手感收敛柔和；接近竖直时角度项连续衰减，不因欧拉 roll 爆炸而乱打舵（阻尼保留，方向永远对）。

**已知限制**：Angle 档在倒扣平飞（机腹朝上）时，目标角域仍按正飞定义（roll 目标 0°），若进入倒扣维持状态修正方向与直觉相反（同修改点 22 说明）；竖直段 Angle 只剩阻尼无回平（物理极限）。如需"倒扣目标域映射"另行实现。

**回退**：恢复修改点 22 后状态 = 撤回本点 5 处文件改动（stabilizer.h/.cpp 的枚举与三分支、LUA 隐藏项恢复注册、config.h 注释、使用文档）；无需动 EEPROM 布局。

**说明**：lint 已查 0 错误；编译验证通过（2026-09-06 11:25，env `Unified_ESP32C3_2400_RX_via_UART`，RAM 18.6% / Flash 56.1%，firmware.bin 1148.4KB）。警告均为第三方库既有告警（AsyncJson 弃用、POWERMGNT 未用变量），本次改动文件无告警。

---

## 修改点 24：Rate 档参数独立可调 + 降增益/限幅 + 加死区与输出低通（2026-09-22）

**背景**：试飞反馈——
1. Rate 档反应很快、飞机左右晃动、舵面角度比较大；
2. Angle 档没问题，但两档参数应分开，各有自己的 Kp / Kd（用户确认"ki"为 **Kd** 笔误；代码为 PD 控制，无积分项 I）。

**原因分析**（基于修改点 23 的实现数值）：
- Rate 原为**固定宏**：增益 `2.0 us/(°/s)`、限幅 `±410us`（±40% 满行程）；而 Angle 档实际最大仅 `±105us`（±10%）→ **Rate 舵量是 Angle 的 4 倍**，这是"舵面角度大"的直接原因；
- **无死区**：陀螺噪声与零偏残差（几 °/s）也被乘 2.0 直接出舵 → 静止/手抖时舵面持续微动；
- **无输出滤波**：修正量瞬时跳变，在舵机延迟 + 机体惯性下形成"过冲→反向→再过冲"的极限环（wag），即"反应很快 + 左右晃动"。

**修改内容**：
1. `lib/STABILIZER/stabilizer.h`：
   - Rate 三项参数改由 config（LUA 可调）提供，原宏改名为默认值常量：`STAB_RATE_GAIN_DEFAULT 0.8f`、`STAB_RATE_CORR_LIMIT_US_DEFAULT 180`、`STAB_RATE_DEADBAND_DPS_DEFAULT 8`；
   - 新增输出低通宏 `STAB_RATE_LPF_A 0.35f`（写死，不占 LUA 菜单；@500Hz ≈ 28Hz 截止）；
   - 新增成员 `m_rateCorrLpf`（Rate 修正低通状态）。
2. `lib/STABILIZER/stabilizer.cpp`（`computeCorrections()` Rate 分支）：
   - 读 config 三项：增益 `×0.1`（默认 0.8）、限幅 us（默认 180）、死区 °/s（默认 8）；
   - **死区**：`|ω|` 低于死区不出舵，超出后按 `(|ω|−死区)` 线性输出（边界跳变仅 ≈6us，可忽略）；
   - **低通**：对角速度项本身不滤波（避免相位滞后），只对合成后的修正指令做一阶低通；
   - Bypass / Angle 分支均清零 `m_rateCorrLpf`，避免切档瞬间带出上一档修正残量。
3. `lib/CONFIG/config.h` / `config.cpp`：
   - `rx_config_t` **末尾**追加 `stabRateGain`(×0.1，默认 8)、`stabRateLimit`(us，默认 180)、`stabRateDeadband`(°/s，默认 8) —— 不改动既有字段布局；
   - Magic 新增 `STABILIZER_CONFIG_MAGIC_V3 = 0xA7`；`Load()` 增加 V2→V3 迁移：**只补三项新参数默认值，用户已调好的 Kp/Kd/Airframe 等全部保留**（V1→V3 链式同理）；
   - 新增三个 getter / setter；
   - 修正 `stabRollKd` 注释错误（原写"Angle/Rate 共用"，实际 **Rate 档从不使用它**）。
4. `lib/LUA/rx_devLUA.cpp`：新增 `Rate Gain`(0~30，×0.1)、`Rate Limit`(0~255 us)、`Rate Dead`(0~30 °/s) 三项，菜单 8 项 → **11 项**；定义、回调、注册、刷新四处同步。
5. `Stabilizer_使用文档.txt`：菜单项重编号并新增 [7][8][9] 三项说明；补 Rate 档"对症调参表"（晃动→降 Gain→降 Limit；刹不住→加 Gain/Limit；抖舵→加 Dead）；概述限幅数据修正（Angle 实际 ±105us、Rate ±180us）。

**行为**：
- 两档参数彻底解耦：Angle 档用 `Roll Kp` + `Roll Kd`，Rate 档用 `Rate Gain` + `Rate Limit` + `Rate Dead`，互不影响；
- Rate 默认舵量降到原来的约 1/5（增益 40% × 限幅 44%），并滤除死区内噪声 → 静止不抖舵、晃动与过冲大幅减弱；
- Angle 档控制律与默认值完全未动（用户确认"Angle 没问题"）。

**调参指引**（写入使用文档第 6 条）：
| 现象 | 处理 |
|---|---|
| 仍左右晃动 / 过冲 | 先降 `Rate Gain`(0.8→0.5)，再降 `Rate Limit`(180→120) |
| 刹不住、停得慢 | 加 `Rate Gain`(0.8→1.2) 或 `Rate Limit` |
| 静止/小动作抖舵 | 加 `Rate Dead`(8→12 °/s) |
| 舵面动作突兀 | 降 `Rate Limit` |

**回退**：恢复修改点 23 行为 = 把 config 三项设为 `Gain 20 / Limit 255(或 250) / Dead 0` 即可等效原固定宏手感（限幅上限 255 略低于原 410，如需完全等价需改宏）；代码回退则撤销本点 5 处文件改动，EEPROM 新增字段可留空不影响旧配置加载（magic 不匹配时走 V3 分支补默认值）。

**说明**：编译验证通过（env `Unified_ESP32C3_2400_RX_via_UART`，exit=0，2026-09-22）；lint 0 错误。

---

## 回退总指引

- 本文件在 git 仓库中，可用 `git diff lib/LUA/rx_devLUA.cpp` 查看本次及历史全部改动。
- **注意**：`rx_devLUA.cpp` 在本次修改前已包含之前会话的修改（Stabilizer 菜单、配置项等），**请勿用 `git restore` / `git checkout` 整文件回退**，否则会一并丢失之前的自稳功能修改。
- 正确回退方式：按上面 4 个修改点手动删除新增代码（或先备份当前文件再对照本日志还原）。

---

## 使用说明

1. 编译烧录固件（环境 `Unified_ESP32C3_2400_RX_via_UART`）
2. 遥控器进入 `ExpressLRS LUA → Stabilizer` 菜单，可见新增三项：
   - `IMU`：1 = 传感器正常，0 = 未初始化/失败
   - `Roll` / `Pitch`：实时姿态角（度），晃动模块数值随之变化
3. 三项均为只读，遥控器侧无法修改
