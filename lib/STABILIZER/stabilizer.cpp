#if defined(STABILIZER_ENABLE) && defined(GPIO_PIN_PWM_OUTPUTS)

#include "stabilizer.h"
#include "device.h"
#include "config.h"
#include "common.h"
#include "crsf_protocol.h"
#include "telemetry.h"
#include "devServoOutput.h"

Stabilizer stabilizer;

// 全局遥测队列（rx_main.cpp 定义）：把 CRSF 帧塞进去即可经 OTA 转发到遥控器
extern Telemetry telemetry;

// 实时姿态遥测：CRSF ATTITUDE 帧（pitch/roll/yaw，弧度×10000，大端字节序）
// EdgeTX 遥测页自动识别 "Attitude" 传感器，实时显示姿态角（可加姿态仪 widget），
// 不依赖 LUA 参数页的拉模式快照。
static void sendAttitudeTelemetry()
{
    CRSF_MK_FRAME_T(crsf_sensor_attitude_t)
    crsfatt = {0};

    int16_t rollRad10000  = (int16_t)(stabilizer.roll()  * (DEG_TO_RAD * 10000.0f));
    int16_t pitchRad10000 = (int16_t)(stabilizer.pitch() * (DEG_TO_RAD * 10000.0f));
    // CRSF ATTITUDE 字段为大端（兼容 Betaflight/EdgeTX），ESP32 为小端需字节交换
    crsfatt.p.roll  = (int16_t)(((uint16_t)rollRad10000 >> 8) | ((uint16_t)rollRad10000 << 8));
    crsfatt.p.pitch = (int16_t)(((uint16_t)pitchRad10000 >> 8) | ((uint16_t)pitchRad10000 << 8));
    // yaw 不关心航向，帧格式固定含 3 轴字段无法删除 → 复用该字段回传 CH1 实际输出
    // PWM（us，988-2012，已含自稳修正）。验证方法：摇杆固定，机体转个角度，
    // 若该值随机体姿态反向变化，说明 IMU 修正真实叠加到了 CH1 输出。
    // EdgeTX 端：不添加 Yaw 传感器项即不显示；想观察 CH1 输出就添加 Yaw 项。
    uint16_t ch1us = servoOutputGetValue(0);
    crsfatt.p.yaw = (int16_t)(((uint16_t)ch1us >> 8) | ((uint16_t)ch1us << 8));

    CRSF::SetHeaderAndCrc((uint8_t *)&crsfatt, CRSF_FRAMETYPE_ATTITUDE,
                          CRSF_FRAME_SIZE(sizeof(crsf_sensor_attitude_t)),
                          CRSF_ADDRESS_CRSF_TRANSMITTER);
    telemetry.AppendTelemetryPackage((uint8_t *)&crsfatt);
}

// 摇杆值（CRSF 172..1811，中位 992）→ 目标角（度），并受轴限幅
static float stickToTarget(int32_t val, float limitDeg)
{
    float frac = (val - CRSF_CHANNEL_VALUE_MID) /
                 (float)(CRSF_CHANNEL_VALUE_MAX - CRSF_CHANNEL_VALUE_MID);
    frac = constrain(frac, -1.0f, 1.0f);
    return frac * limitDeg;
}

void Stabilizer::init()
{
    m_lastUpdateMs = millis();
    if (!m_imu.begin())
    {
        // 传感器缺失/异常：保持旁路（三级安全 2）
        return;
    }
    calibrateGyro();
}

void Stabilizer::calibrateGyro()
{
    // 上电静态校准：平均约 100ms 陀螺仪零点（飞机上电时静止）
    float sum[3] = {0.0f, 0.0f, 0.0f};
    int count = 0;
    for (int i = 0; i < 50; ++i)
    {
        int16_t ax, ay, az, gx, gy, gz;
        if (m_imu.readRaw(ax, ay, az, gx, gy, gz))
        {
            sum[0] += gx;
            sum[1] += gy;
            sum[2] += gz;
            ++count;
        }
        delay(2);
    }
    if (count > 0)
    {
        m_gyroBiasDps[0] = sum[0] / count / MPU_GYRO_SCALE_LSB;
        m_gyroBiasDps[1] = sum[1] / count / MPU_GYRO_SCALE_LSB;
        m_gyroBiasDps[2] = sum[2] / count / MPU_GYRO_SCALE_LSB;
    }
}

StabMode_e Stabilizer::getMode() const
{
    // GainCh 三档实时判定（不存档）：低=直通、中=Rate、高=Angle
    uint8_t gainCh = config.GetStabGainCh();
    uint32_t val = ChannelData[gainCh];
    if (val < STAB_MODE_LOW_BOUND)   // 含未收到(0)，上电默认安全档
    {
        return STAB_MODE_BYPASS;
    }
    if (val < STAB_MODE_HIGH_BOUND)
    {
        return STAB_MODE_RATE;
    }
    return STAB_MODE_ANGLE;
}

bool Stabilizer::isActive() const
{
    // 三级安全：
    // 1) 上电默认直通 —— 开关通道低档(直通)/未收到(0)时不介入
    // 2) 传感器健康 —— 连续读失败达阈值自动旁路
    // 3) 断连/失联旁路 —— 信号丢失时交给原有 failsafe
    if (!m_imu.healthy())
    {
        return false;
    }
    if (connectionState != connected)
    {
        return false;
    }
    return getMode() != STAB_MODE_BYPASS;
}

void Stabilizer::update()
{
    // 诊断：update() 被调用即计数（LUA UpdCnt 观察姿态环是否在跑）
    ++m_updateCount;

    uint32_t now = millis();
    float dt = (now - m_lastUpdateMs) * 0.001f;
    m_lastUpdateMs = now;
    if (dt <= 0.0f)
    {
        dt = 0.002f;
    }
    else if (dt > 0.1f)
    {
        dt = 0.1f;   // 上电首次 / 长时间停顿限幅
    }

    int16_t ax, ay, az, gx, gy, gz;
    if (!m_imu.readRaw(ax, ay, az, gx, gy, gz))
    {
        // 诊断：readRaw 失败累计（LUA RdErr 观察 I2C 是否异常）
        ++m_readErrCount;
        // 读取失败：立即清零修正直通输出，避免用陈旧修正值叠加
        // （健康监控在驱动内部处理，连续失败达阈值后 isActive 自动关闭）
        m_corrRollUs = 0;
        m_corrPitchUs = 0;
        return;
    }

    // 保存原始读数（LUA 诊断：判断数据是否有效/全 0）
    m_rawGx = gx;
    m_rawGy = gy;
    m_rawGz = gz;
    m_rawAz = az;

    // 去陀螺仪零点偏置（dps）
    float gxDps = gx / MPU_GYRO_SCALE_LSB - m_gyroBiasDps[0];
    float gyDps = gy / MPU_GYRO_SCALE_LSB - m_gyroBiasDps[1];
    float gzDps = gz / MPU_GYRO_SCALE_LSB - m_gyroBiasDps[2];

    // Mahony 互补滤波（陀螺仪用 rad/s）
    mahonyStep(gxDps * DEG_TO_RAD, gyDps * DEG_TO_RAD, gzDps * DEG_TO_RAD,
               ax / MPU_ACCEL_SCALE_LSB, ay / MPU_ACCEL_SCALE_LSB, az / MPU_ACCEL_SCALE_LSB,
               dt);

    // 提取姿态角（度）
    float qw = m_q[0], qx = m_q[1], qy = m_q[2], qz = m_q[3];
    m_roll = atan2f(2.0f * (qw * qx + qy * qz), 1.0f - 2.0f * (qx * qx + qy * qy)) * RAD_TO_DEG;
    m_pitch = asinf(constrain(2.0f * (qw * qy - qz * qx), -1.0f, 1.0f)) * RAD_TO_DEG;

    // 注：横滚角沿用上方欧拉 m_roll。曾尝试改用"重力投影角 atan2(gyE,gzE)"，
    // 发现其与欧拉 roll 数学恒等（分子 gyE、分母 gzE 归一化后即欧拉分母），
    // 竖立爆炸是物理极限（机头竖直时机体绕机头轴旋转不改变重力投影，横滚
    // 不可观测），非公式选择问题。真正的对策在 computeCorrections() 的 Angle
    // 分支：角度项按重力参考可靠度 |gzE| 连续衰减，竖直段自动退化为纯阻尼。

    // 遥测姿态角低通（仅显示平滑，PD 修正仍用原始角，无相位损失）
    m_attLpf[0] += STAB_ATT_LPF_A * (m_roll - m_attLpf[0]);
    m_attLpf[1] += STAB_ATT_LPF_A * (m_pitch - m_attLpf[1]);

    computeCorrections(gxDps);
}

void Stabilizer::mahonyStep(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    float q0 = m_q[0], q1 = m_q[1], q2 = m_q[2], q3 = m_q[3];

    // 加速度计修正（仅在加速度计有有效输出时）：
    // 自由落体 / 数据全 0 / 归一化失败时跳过加速度修正，但陀螺仪积分必须继续，
    // 否则一旦加速度计异常（如传感器未退睡眠输出全 0），四元数永久冻结，Roll/Pitch 恒 0 不动。
    float ex = 0.0f, ey = 0.0f, ez = 0.0f;
    float norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm > 1e-6f)
    {
        ax /= norm;
        ay /= norm;
        az /= norm;

        // 四元数预测的重力方向
        float gxE = 2.0f * (q1 * q3 - q0 * q2);
        float gyE = 2.0f * (q0 * q1 + q2 * q3);
        float gzE = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        // 误差向量 = 加速度计 × 预测重力（叉积）
        ex = ay * gzE - az * gyE;
        ey = az * gxE - ax * gzE;
        ez = ax * gyE - ay * gxE;
    }

    // 运动检测调制加速度修正权重（防晃动拉偏 + 飞行震动串入）：
    // 角速度大时（晃动/机动）加速度计受线加速度污染，参考方向不可信 → 权重
    // 降为 10%，姿态主要靠陀螺仪积分（短时准确）；角速度趋零（静止/停手）→
    // 权重恢复 100%，加速度计把长期漂移与积分误差迅速拉回。
    // 阈值：>15°/s 视为运动（0.26 rad/s），≤2°/s 视为静止（0.035 rad/s）
    // 下限 0.1：晃动时保留少量修正把姿态锚住（下限 0 会让陀螺零偏漂移
    // 无人拉回，停手后反而"稳定不住"，实测更震荡，已回退）。
    float gyroMag = sqrtf(gx * gx + gy * gy + gz * gz);   // rad/s
    float accelWeight = constrain((0.26f - gyroMag) / 0.23f, 0.1f, 1.0f);

    // 积分项：消除陀螺零偏引起的长期漂移，加速静止收敛（KI>0 时生效）。
    // 权重缩放：运动时（权重小）积分几乎冻结，避免大机动/晃动时错误累积。
    if (STAB_MAHONY_KI > 0.0f)
    {
        float ki = STAB_MAHONY_KI * accelWeight;
        m_integralFB[0] += ki * ex * dt;
        m_integralFB[1] += ki * ey * dt;
        m_integralFB[2] += ki * ez * dt;
    }

    // 陀螺仪漂移补偿
    float kp = STAB_MAHONY_KP * accelWeight;
    gx += kp * ex + m_integralFB[0];
    gy += kp * ey + m_integralFB[1];
    gz += kp * ez + m_integralFB[2];

    // 四元数微分（一阶）
    float gx2 = 0.5f * gx * dt, gy2 = 0.5f * gy * dt, gz2 = 0.5f * gz * dt;
    float n0 = q0 + (-q1 * gx2 - q2 * gy2 - q3 * gz2);
    float n1 = q1 + (q0 * gx2 + q2 * gz2 - q3 * gy2);
    float n2 = q2 + (q0 * gy2 - q1 * gz2 + q3 * gx2);
    float n3 = q3 + (q0 * gz2 + q1 * gy2 - q2 * gx2);

    // 归一化
    norm = sqrtf(n0 * n0 + n1 * n1 + n2 * n2 + n3 * n3);
    if (norm > 1e-9f)
    {
        m_q[0] = n0 / norm;
        m_q[1] = n1 / norm;
        m_q[2] = n2 / norm;
        m_q[3] = n3 / norm;
    }
}

void Stabilizer::computeCorrections(float gyroXDps)
{
    // 模式三分支：直通 / Rate（纯阻尼）/ Angle（横滚回平）。
    // 俯仰全模式直通：m_corrPitchUs 恒 0（升降舵纯杆量，跟手）。

    StabMode_e mode = getMode();
    m_corrRollUs = 0;
    m_corrPitchUs = 0;
    if (mode == STAB_MODE_BYPASS)
    {
        // 直通档清零 Rate 低通状态，避免切档瞬间把上一次的修正残量带出来
        m_rateCorrLpf = 0.0f;
        return;
    }

    if (mode == STAB_MODE_RATE)
    {
        // Rate：纯角速度阻尼。只用陀螺，不依赖姿态参考 —— 任何姿态方向都正确、
        // 无万向节锁定；松杆即刹停旋转，但不强制回平（跟手）。
        //
        // 修改点 24：增益 / 限幅 / 死区改为 LUA 可调（原为固定宏 2.0、±410us），
        // 并新增输出低通。针对实测反馈「反应很快 / 左右晃动 / 舵面角度大」：
        //   - 增益 2.0 → 0.8：单位角速度舵量减到 40%
        //   - 限幅 ±410us(±40%) → ±180us(≈±18%)：与 Angle 档(±105us)量级拉齐
        //   - 死区 8°/s：静止/手抖时陀螺噪声与零偏残差不再被放大成抖舵
        //   - 输出低通 α=0.35：软化瞬间跳变，抑制与舵机延迟形成的极限环
        float gain = config.GetStabRateGain() * 0.1f;          // ×0.1，默认 8 = 0.8
        float limitUs = (float)config.GetStabRateLimit();      // us，默认 180
        float deadband = (float)config.GetStabRateDeadband();  // °/s，默认 8

        // 死区：扣除死区量后线性输出（越界处跳变仅 gain×deadband ≈ 6us，可忽略）
        float mag = fabsf(gyroXDps);
        float wEff = 0.0f;
        if (mag > deadband)
        {
            float magEff = mag - deadband;
            wEff = (gyroXDps >= 0.0f) ? magEff : -magEff;
        }

        float corr = -wEff * gain;
        corr = constrain(corr, -limitUs, limitUs);

        // 一阶低通：平滑修正指令（对角速度项本身不加滤波，避免相位滞后）
        m_rateCorrLpf += STAB_RATE_LPF_A * (corr - m_rateCorrLpf);
        m_corrRollUs = (int16_t)m_rateCorrLpf;

        if (config.GetStabRollDir())
        {
            m_corrRollUs = -m_corrRollUs;
        }
        return;
    }

    // ---- Angle（剩余分支）：横滚回平 ----
    // 切到 Angle 时同样清零 Rate 低通（两档修正独立，互不残留）
    m_rateCorrLpf = 0.0f;
    // 摇杆意图：读取 PWM1/PWM2 的输入通道（尊重 Output Mapping 配置）
    // PWM1 = 副翼（普通）/ 左 elevon（飞翼）；PWM2 = 升降 / 右 elevon（飞翼）
    uint8_t aileronCh = config.GetPwmChannel(0)->val.inputChannel;
    uint8_t elevatorCh = config.GetPwmChannel(1)->val.inputChannel;
    int32_t aileronVal = ChannelData[aileronCh];
    int32_t elevatorVal = ChannelData[elevatorCh];

    float targetRoll;
    if (config.GetStabAirframe() == STAB_AIRFRAME_WING)
    {
        // 飞翼：遥控器已做 elevon 混控，从混合信号反解副翼摇杆意图
        // 左 elevon = 升降 + 副翼，右 elevon = 升降 − 副翼 → 副翼 = (左−右)/2
        targetRoll = stickToTarget((aileronVal - elevatorVal) / 2, STAB_ROLL_LIMIT_DEG);
    }
    else
    {
        targetRoll = stickToTarget(aileronVal, STAB_ROLL_LIMIT_DEG);
    }

    // LUA 定点缩放：Kp ×0.1、Kd ×0.01
    float kpRoll = config.GetStabRollKp() * 0.1f;
    float kdRoll = config.GetStabRollKd() * 0.01f;

    // 重力参考可靠度（0~1）：机体 z 轴上的重力分量 gzE（= qw²−qx²−qy²+qz²）。
    // 平飞时 |gzE|≈1，机头竖直时 →0。物理上机头竖直时绕机头轴旋转不改变重力
    // 投影，绝对横滚不可观测 → 角度项必须随之让位，否则欧拉 roll 在 ±90° 邻域
    // 读数爆炸、修正乱打。这里做连续软衰减（非硬清零防御）：
    //   角度项 × reliability，可靠度从 1 平滑降到 0；
    //   阻尼项（−ω×Kd）不受影响 —— 竖直段自动退化为纯阻尼，方向永远正确。
    float qw = m_q[0], qx = m_q[1], qy = m_q[2], qz = m_q[3];
    float gzE = qw * qw - qx * qx - qy * qy + qz * qz;
    float reliability = constrain((fabsf(gzE) - STAB_GRAV_REF_FLOOR) / (1.0f - STAB_GRAV_REF_FLOOR), 0.0f, 1.0f);

    // PD 控制律（无积分）：
    // 修正 = (目标角 − 实测角) × Kp × reliability − 角速度 × Kd
    float corrRollDeg = (targetRoll - m_roll) * kpRoll * reliability - gyroXDps * kdRoll;

    // Angle 修正输出限幅（±15% 满行程 ≈ ±30 度）
    corrRollDeg = constrain(corrRollDeg, -STAB_CORR_LIMIT_DEG, STAB_CORR_LIMIT_DEG);
    // 收敛系数：整体打折使回平更柔和、不抢杆（1.0=原手感，0.7=收敛 30%）
    m_corrRollUs = (int16_t)(corrRollDeg * STAB_CORR_SCALE_US_DEG * STAB_ANGLE_SOFTEN);

    // 轴方向开关（LUA: Roll Dir）：只翻转 IMU 修正方向，独立于 Invert
    // （Invert 是"直通+修正"一起翻）。用于传感器装反等"摇杆方向正确、自稳方向反"场景。
    // 翻转发生在逻辑轴源头，差动副翼（CH6=-roll）与飞翼混控（roll 差动）的
    // 结构符号自动保持正确，无需逐通道设置。
    if (config.GetStabRollDir())
    {
        m_corrRollUs = -m_corrRollUs;
    }
}

uint16_t Stabilizer::applyCorrection(uint8_t pwmCh, uint16_t us) const
{
    if (!isActive())
    {
        return us;
    }

    // 注意：m_corrPitchUs 恒为 0（修改点 23：俯仰全模式直通，升降舵纯杆量）。
    // 因此普通布局 CH2 不叠加任何修正；飞翼布局 PWM1/PWM2 自动退化为
    // 纯 roll 差动（+roll / −roll，两片反向），即"只差动介入，不同向"。
    int16_t corr = 0;
    if (config.GetStabAirframe() == STAB_AIRFRAME_NORMAL)
    {
        switch (pwmCh)
        {
        case 0:  corr = m_corrRollUs; break;                          // PWM1(CH1) 副翼（1舵 Y 线或右舵）
        case 1:  corr = m_corrPitchUs; break;                         // PWM2(CH2) 升降（恒 0 = 直通）
        case 5:
            if (config.GetStabAileron() == STAB_AILERON_2SERVO)
            {
                corr = -m_corrRollUs;                                 // PWM6(CH6) 左副翼（差动）
            }
            break;
        default: break;
        }
    }
    else // STAB_AIRFRAME_WING
    {
        switch (pwmCh)
        {
        case 0:  corr = m_corrPitchUs + m_corrRollUs; break;          // PWM1 左升降副翼（= +roll）
        case 1:  corr = m_corrPitchUs - m_corrRollUs; break;          // PWM2 右升降副翼（= −roll）
        default: break;
        }
    }

    if (corr == 0)
    {
        return us;
    }

    // 通道 inverted 时舵面动作方向与通道值方向相反，修正需反向，
    // 否则增稳方向反了变成正反馈（servosUpdate 中 us 已先翻转）
    if (config.GetPwmChannel(pwmCh)->val.inverted)
    {
        corr = -corr;
    }

    // 输出限幅 ±PWM 满行程（修正叠加后不允许超出 988..2012us）
    int32_t out = (int32_t)us + corr;
    out = constrain(out, (int32_t)CRSF_to_US(CRSF_CHANNEL_VALUE_MIN),
                    (int32_t)CRSF_to_US(CRSF_CHANNEL_VALUE_MAX));
    return (uint16_t)out;
}

// ============================================================
// ELRS device 集成：initialize 中初始化 IMU，timeout 中 500Hz 姿态环
// ============================================================
static void devInit()
{
    stabilizer.init();
}

static int devTimeout()
{
    static uint32_t lastUpdate;
    uint32_t now = millis();
    // 防卡顿降频：仅「已连接 + 传感器健康」时才跑 500Hz 姿态环。
    // 未连接 / 传感器异常时降到 100ms 一次（保持 healthy 检测与姿态跟踪即可）。
    // 原因：ESP32 Wire 在 I2C 总线挂死（如 SDA 被拉低）时一次传输最多阻塞 ~50ms，
    // 若传感器故障仍每 2ms 强读一次，会把主循环拖慢 50ms/轮 → 无线收包超时断连，
    // 而 isActive() 要求 connected，断连后自稳反而不介入。
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

    // 实时姿态遥测（10Hz，仅连接时占用遥测带宽）。
    // 队列内已有 ATTITUDE 帧会直接覆盖（不会堆积），断开后自动停发。
    if (connectionState == connected)
    {
        static uint32_t lastTelem;
        if (now - lastTelem >= 100)
        {
            lastTelem = now;
            sendAttitudeTelemetry();
        }
    }
    return DURATION_IMMEDIATELY;
}

static int devStart()
{
    // 必须提供 start：devicesStart() 对 start==nullptr 的 device 将 deviceTimeout 置为
    // 0xFFFFFFFF，_devicesUpdate() 中 `now >= deviceTimeout` 永远为假 → timeout 永不被调用，
    // 姿态环不会运行（现象：IMU 显示 1 但 Roll/Pitch 恒 0）。
    return DURATION_IMMEDIATELY;
}

device_t Stabilizer_device = {
    .initialize = devInit,
    .start = devStart,
    .event = nullptr,
    .timeout = devTimeout,
};

#endif // STABILIZER_ENABLE && GPIO_PIN_PWM_OUTPUTS
