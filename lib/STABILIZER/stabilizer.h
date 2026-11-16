#pragma once

#include <Arduino.h>
#include "imu_mpu6500.h"
#include "device.h"

// ============================================================
// 固定翼姿态自稳模块（STABILIZER_ENABLE 编译开关）
//
// 架构：姿态修正层插入在 RC 通道数据与 PWM 输出之间。
//   - 修正量在逻辑轴（Roll/Pitch）上由 PD 控制律算出
//   - 在舵机空间叠加到物理通道（不反解遥控器混控，保留用户使用习惯）
//   - 三级安全：默认直通 / 传感器异常旁路 / 断连旁路
// ============================================================

// 姿态环频率
#define STAB_LOOP_INTERVAL_MS   2     // 500 Hz

// 摇杆意图轴限幅（度）
#define STAB_ROLL_LIMIT_DEG     60.0f
#define STAB_PITCH_LIMIT_DEG    30.0f

// 模式通道三档判定阈值（CRSF 通道值 172..1811，中位 992）
// 低(< 1/3 行程) = 直通；中 = Rate；高(> 2/3 行程) = Angle
#define STAB_MODE_LOW_BOUND     660   // 低于此值视为低档
#define STAB_MODE_HIGH_BOUND    1320  // 高于此值视为高档

// Angle 修正量限幅：±30 度 × 5us/度 ≈ ±150us ≈ 满行程(1024us)的 15%
#define STAB_CORR_LIMIT_DEG     30.0f
#define STAB_CORR_SCALE_US_DEG  5.0f

// Angle 模式收敛系数（0~1）：对 PD 修正整体打折，使回平更柔和、不抢杆。
// 1.0 = 原手感；0.7 = 收敛 30%（用户确认收敛些）。
#define STAB_ANGLE_SOFTEN       0.7f

// Angle 重力参考可靠度下限：|gzE| 低于此值（机头接近竖直）时角度项权重降为 0。
// 0.30 ≈ |pitch|≈72° 起角度项开始让位，到 90° 完全退化为纯阻尼。
// 这是连续软衰减（非硬清零防御），正常飞行包线内可靠度≈1，不影响手感。
#define STAB_GRAV_REF_FLOOR     0.30f

// Rate 模式：纯角速度阻尼。增益 = 修正量(us) / 角速度(°/s)。
// 修改点 24：增益 / 限幅 / 死区改为 LUA 可调（存于 config，见 stabRateGain 等），
// 此处仅为烧写默认值与代码内参考。原值 2.0 / ±410us（±40% 行程）实测过冲晃动、
// 舵量过大（是 Angle 档 ±105us 的 4 倍），默认下调为 0.8 / ±180us（≈±18%），
// 并新增死区 + 输出低通抑制抖舵与极限环振荡（wag）。
#define STAB_RATE_GAIN_DEFAULT          0.8f   // ×0.1 存 config，默认 8 = 0.8
#define STAB_RATE_CORR_LIMIT_US_DEFAULT 180    // us（≈±18% 满行程）
#define STAB_RATE_DEADBAND_DPS_DEFAULT  8      // °/s（角速度低于此值不介入）

// Rate 修正输出一阶低通（写死为宏，不占 LUA 菜单）：软化"反应很快"，
// 滤除陀螺噪声被增益放大后引起的舵面高频抖动。
// α=0.35 @500Hz ≈ 28Hz 截止，高于阻尼环有效带宽，不影响正常刹停手感。
#define STAB_RATE_LPF_A                 0.35f

// Mahony 滤波器增益
// KP 为「静止满权重」值（mahonyStep 中按角速度动态调制，晃动时自动降低）。
// KP=1.0 时静止收敛时间常数≈1s，晃动停手后需 5~10s 才完全稳定（被线加速度
// 污染拉偏后回拉慢）；提到 3.0 后时间常数≈0.33s，1~2s 内稳定。
#define STAB_MAHONY_KP          3.0f
// KI=0.1：静止时通过积分消除陀螺零偏引起的长期漂移，加速最终收敛；
// 运动时随权重缩放（Ki×accelWeight），避免大机动/晃动时错误积分累积。
#define STAB_MAHONY_KI          0.1f

// 遥测姿态角低通（α=0.20，约 16Hz）：只影响回传显示，不参与控制环（PD 用原始角）。
#define STAB_ATT_LPF_A          0.20f

// 机型布局（与 LUA "Airframe" 选项顺序一致）
enum StabAirframe_e : uint8_t
{
    STAB_AIRFRAME_NORMAL = 0,   // 普通布局
    STAB_AIRFRAME_WING = 1,     // 飞翼布局
};

// 副翼舵机数（与 LUA "Aileron" 选项顺序一致）
enum StabAileron_e : uint8_t
{
    STAB_AILERON_1SERVO = 0,    // 1 个副翼舵机（含 Y 线/机械联动）
    STAB_AILERON_2SERVO = 1,    // 2 个独立副翼舵机
};

// 介入模式（由 GainCh 三档开关实时判定，不存档；与 LUA/遥测显示一致）
enum StabMode_e : uint8_t
{
    STAB_MODE_BYPASS = 0,   // 低档：直通（纯杆量，自稳完全退出）
    STAB_MODE_RATE = 1,     // 中档：Rate（纯角速度阻尼，不强制回平）
    STAB_MODE_ANGLE = 2,    // 高档：Angle（横滚回平）
};

class Stabilizer
{
public:
    void init();        // 初始化 IMU + 陀螺仪静态校准（约 100ms）
    void update();      // 500Hz 姿态环：读 IMU → Mahony 解算 → 修正
    bool isActive() const;  // 开关非直通档 + 传感器健康 + 已连接
    StabMode_e getMode() const;  // 当前介入模式（读 GainCh 三档）

    // 对某 PWM 输出通道的舵机值叠加修正（未启用/异常时原样返回）
    uint16_t applyCorrection(uint8_t pwmCh, uint16_t us) const;

    // 状态查询（调试/遥测用）：返回低通平滑后的姿态角（仅显示，不参与控制环）
    float roll() const { return m_attLpf[0]; }
    float pitch() const { return m_attLpf[1]; }
    bool imuOk() const { return m_imu.healthy(); }

    // 最近一次原始读数（LUA 诊断用：判断传感器数据是否有效/是否全 0）
    int16_t rawGx() const { return m_rawGx; }
    int16_t rawGy() const { return m_rawGy; }
    int16_t rawGz() const { return m_rawGz; }
    int16_t rawAz() const { return m_rawAz; }

    // 诊断计数（LUA 判断姿态环/读取链路是否在跑）：
    // - updateCount() 持续增长 → update() 被调度器正常调用
    // - readErrCount() 增长   → readRaw 持续失败（I2C/传感器异常）
    uint16_t updateCount() const { return m_updateCount; }
    uint16_t readErrCount() const { return m_readErrCount; }

private:
    void mahonyStep(float gx, float gy, float gz, float ax, float ay, float az, float dt);
    void computeCorrections(float gyroXDps);
    void calibrateGyro();

    IMU_MPU6500 m_imu;
    float m_q[4] = {1.0f, 0.0f, 0.0f, 0.0f};   // 四元数 w,x,y,z
    float m_roll = 0.0f;       // 度（欧拉横滚角，Angle 控制用；竖直段靠可靠度衰减兜底）
    float m_pitch = 0.0f;      // 度
    int16_t m_corrRollUs = 0;  // 横滚修正（us，舵机空间）
    int16_t m_corrPitchUs = 0; // 俯仰修正（us，舵机空间，恒 0 = 全模式直通）
    float m_gyroBiasDps[3] = {0.0f, 0.0f, 0.0f};
    float m_rateCorrLpf = 0.0f;    // Rate 档修正输出低通状态（us）；退出 Rate 时清零
    float m_attLpf[2] = {0.0f, 0.0f};               // 遥测姿态角低通（roll, pitch）
    float m_integralFB[3] = {0.0f, 0.0f, 0.0f};
    uint32_t m_lastUpdateMs = 0;
    int16_t m_rawGx = 0, m_rawGy = 0, m_rawGz = 0, m_rawAz = 0;
    uint16_t m_updateCount = 0;   // 诊断：update() 累计调用次数
    uint16_t m_readErrCount = 0;  // 诊断：readRaw 累计失败次数
};

// 全局实例
extern Stabilizer stabilizer;

// 供 PWM 输出链调用的叠加函数（未启用时原样返回）
inline uint16_t stabilizerApply(uint8_t pwmCh, uint16_t us)
{
    return stabilizer.applyCorrection(pwmCh, us);
}

// ELRS device 注册对象（rx_main.cpp 使用）
#if defined(GPIO_PIN_PWM_OUTPUTS)
extern device_t Stabilizer_device;
#endif
