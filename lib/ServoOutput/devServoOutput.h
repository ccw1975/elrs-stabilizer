#pragma once
#if defined(GPIO_PIN_PWM_OUTPUTS)

#include "device.h"
#include "common.h"

#if (defined(PLATFORM_ESP32))
#include "DShotRMT.h"
#endif

extern device_t ServoOut_device;
#define HAS_SERVO_OUTPUT
#define OPT_HAS_SERVO_OUTPUT (GPIO_PIN_PWM_OUTPUTS_COUNT > 0)

// Notify this unit that new channel data has arrived
void servoNewChannelsAvailable();

// 读取某 PWM 通道当前实际输出值（us，已含自稳修正；未初始化/无有效输出返回 1500 中位）
uint16_t servoOutputGetValue(uint8_t ch);
#else
inline void servoNewChannelsAvailable(){};
inline uint16_t servoOutputGetValue(uint8_t ch){ return 1500; }
#endif
