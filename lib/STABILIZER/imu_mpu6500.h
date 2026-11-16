#pragma once

#include <Arduino.h>
#include <Wire.h>

// ============================================================
// MPU6050 / MPU6500 I2C 驱动（两芯片寄存器完全兼容）
// WHO_AM_I：MPU6050 = 0x68，MPU6500 = 0x70
// I2C 总线复用 ELRS 现有 Wire（GPIO3/GPIO7，400kHz）
// ============================================================

// 共用寄存器
#define MPU_REG_PWR_MGMT_1   0x6B
#define MPU_REG_PWR_MGMT_2   0x6C
#define MPU_REG_CONFIG       0x1A
#define MPU_REG_GYRO_CONFIG  0x1B
#define MPU_REG_ACCEL_CONFIG 0x1C
#define MPU_REG_ACCEL_XOUT_H 0x3B
#define MPU_REG_GYRO_XOUT_H  0x43
#define MPU_REG_WHO_AM_I     0x75

#define MPU6050_WHO_AM_I_VAL 0x68
#define MPU6500_WHO_AM_I_VAL 0x70

// 量程换算：陀螺仪 ±2000dps、加速度计 ±16g
#define MPU_GYRO_SCALE_LSB   16.4f
#define MPU_ACCEL_SCALE_LSB  2048.0f

// 连续读失败多少次判定传感器故障（健康监控）
#define IMU_ERROR_THRESHOLD  10

class IMU_MPU6500
{
public:
    // 初始化：WHO_AM_I 校验（兼容 0x68/0x70）+ 复位 + 退睡眠 + 量程配置
    bool begin(uint8_t address = 0x68);

    // 一次读取 14 字节原始数据（加速度 6 字节 + 温度 2 字节跳过 + 陀螺 6 字节）
    bool readRaw(int16_t &ax, int16_t &ay, int16_t &az,
                 int16_t &gx, int16_t &gy, int16_t &gz);

    bool healthy() const { return m_healthy; }
    uint8_t whoami() const { return m_whoami; }

private:
    bool readBytes(uint8_t reg, uint8_t *data, uint8_t len);
    bool writeByte(uint8_t reg, uint8_t val);

    uint8_t m_address = 0x68;
    uint8_t m_whoami = 0;
    bool m_healthy = false;
    uint8_t m_consecutiveErrors = 0;
};
