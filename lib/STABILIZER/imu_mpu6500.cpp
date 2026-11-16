#if defined(STABILIZER_ENABLE)

#include "imu_mpu6500.h"

bool IMU_MPU6500::writeByte(uint8_t reg, uint8_t val)
{
    Wire.beginTransmission(m_address);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

bool IMU_MPU6500::readBytes(uint8_t reg, uint8_t *data, uint8_t len)
{
    Wire.beginTransmission(m_address);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0)
    {
        return false;
    }
    if (Wire.requestFrom((int)m_address, (int)len) != len)
    {
        return false;
    }
    for (uint8_t i = 0; i < len; ++i)
    {
        data[i] = Wire.read();
    }
    return true;
}

bool IMU_MPU6500::begin(uint8_t address)
{
    m_healthy = false;
    m_consecutiveErrors = 0;

    // 地址回退：MPU6050/MPU6500 只有 0x68 与 0x69 两个地址（由 AD0 引脚电平决定）。
    // 优先使用传入地址，WHO_AM_I 校验失败则回退到另一个（0x68 <-> 0x69），
    // 兼容模块 AD0 接高/接地的两种硬件接法。
    bool detected = false;
    for (uint8_t attempt = 0; attempt < 2; ++attempt)
    {
        m_address = address;
        uint8_t whoami = 0;
        // WHO_AM_I 校验：MPU6050 返回 0x68，MPU6500 返回 0x70
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

    // 复位。关键：必须检查每个配置写入是否成功！
    // 若任一写入失败（I2C 异常/传感器未就绪），传感器可能停留在睡眠状态，
    // 数据寄存器输出全 0 —— WHO_AM_I（ROM 寄存器）仍可读，IMU 会显示 1，
    // 但姿态环 Mahony 因加速度计 norm≈0 而冻结，Roll/Pitch 恒 0 不动。
    if (!writeByte(MPU_REG_PWR_MGMT_1, 0x80))
    {
        return false;   // 复位失败
    }
    delay(20);
    if (!writeByte(MPU_REG_PWR_MGMT_1, 0x01))   // 退出睡眠，选择陀螺仪 PLL 时钟源
    {
        return false;
    }
    delay(10);
    if (!writeByte(MPU_REG_CONFIG, 0x02))       // DLPF 低通滤波：加速度 94Hz / 陀螺仪 98Hz
    {
        return false;
    }
    if (!writeByte(MPU_REG_GYRO_CONFIG, 0x18))  // 陀螺仪量程 ±2000dps（16.4 LSB/dps）
    {
        return false;
    }
    if (!writeByte(MPU_REG_ACCEL_CONFIG, 0x18)) // 加速度计量程 ±16g（2048 LSB/g）
    {
        return false;
    }

    m_healthy = true;
    return true;
}

bool IMU_MPU6500::readRaw(int16_t &ax, int16_t &ay, int16_t &az,
                          int16_t &gx, int16_t &gy, int16_t &gz)
{
    uint8_t buf[14];
    if (!readBytes(MPU_REG_ACCEL_XOUT_H, buf, 14))
    {
        if (m_consecutiveErrors < 255)
        {
            ++m_consecutiveErrors;
        }
        if (m_consecutiveErrors >= IMU_ERROR_THRESHOLD)
        {
            m_healthy = false;
        }
        return false;
    }

    m_consecutiveErrors = 0;
    m_healthy = true;

    ax = (int16_t)((buf[0] << 8) | buf[1]);
    ay = (int16_t)((buf[2] << 8) | buf[3]);
    az = (int16_t)((buf[4] << 8) | buf[5]);
    gx = (int16_t)((buf[8] << 8) | buf[9]);
    gy = (int16_t)((buf[10] << 8) | buf[11]);
    gz = (int16_t)((buf[12] << 8) | buf[13]);
    return true;
}

#endif // STABILIZER_ENABLE
