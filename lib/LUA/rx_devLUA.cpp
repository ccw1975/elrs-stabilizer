#ifdef TARGET_RX

#include "rxtx_devLua.h"
#include "helpers.h"
#include "devServoOutput.h"
#include "deferred.h"
#include "stabilizer.h"

#define RX_HAS_SERIAL1 (GPIO_PIN_SERIAL1_TX != UNDEF_PIN || OPT_HAS_SERVO_OUTPUT)

extern void reconfigureSerial();
#if defined(PLATFORM_ESP32)
extern void reconfigureSerial1();
#endif
extern bool BindingModeRequest;

static char modelString[] = "000";
#if defined(GPIO_PIN_PWM_OUTPUTS)
static char pwmModes[] = "50Hz;60Hz;100Hz;160Hz;333Hz;400Hz;10kHzDuty;On/Off;DShot;Serial RX;Serial TX;I2C SCL;I2C SDA;Serial2 RX;Serial2 TX";
#endif

static struct luaItem_selection luaSerialProtocol = {
    {"Protocol", CRSF_TEXT_SELECTION},
    0, // value
#if defined(TARGET_RX_GHOST_ATTO_V1)
    "CRSF;Inverted CRSF;SBUS;Inverted SBUS",
#elif defined(PLATFORM_STM32)
    "CRSF;Inverted CRSF;SBUS;Inverted SBUS;SUMD;DJI RS Pro",
#else
    "CRSF;Inverted CRSF;SBUS;Inverted SBUS;SUMD;DJI RS Pro;HoTT Telemetry;MAVLink",
#endif
    STR_EMPTYSPACE
};

#if defined(PLATFORM_ESP32)
static struct luaItem_selection luaSerial1Protocol = {
    {"Protocol2", CRSF_TEXT_SELECTION},
    0, // value
    "Off;CRSF;Inverted CRSF;SBUS;Inverted SBUS;SUMD;DJI RS Pro;HoTT Telemetry;Tramp;SmartAudio",
    STR_EMPTYSPACE
};
#endif

static struct luaItem_selection luaSBUSFailsafeMode = {
    {"SBUS failsafe", CRSF_TEXT_SELECTION},
    0, // value
    "No Pulses;Last Pos",
    STR_EMPTYSPACE
};

#if !defined(PLATFORM_STM32)
static struct luaItem_int8 luaTargetSysId = {
  {"Target SysID", CRSF_UINT8},
  {
    {
      (uint8_t)1,       // value - default to 1
      (uint8_t)1,       // min
      (uint8_t)255,     // max
    }
  },
  STR_EMPTYSPACE
};
static struct luaItem_int8 luaSourceSysId = {
  {"Source SysID", CRSF_UINT8},
  {
    {
      (uint8_t)255,       // value - default to 255
      (uint8_t)1,         // min
      (uint8_t)255,       // max
    }
  },
  STR_EMPTYSPACE
};
#endif

#if defined(POWER_OUTPUT_VALUES)
static struct luaItem_selection luaTlmPower = {
    {"Tlm Power", CRSF_TEXT_SELECTION},
    0, // value
    strPowerLevels,
    "mW"
};
#endif

#if defined(GPIO_PIN_ANT_CTRL)
static struct luaItem_selection luaAntennaMode = {
    {"Ant. Mode", CRSF_TEXT_SELECTION},
    0, // value
    "Antenna A;Antenna B;Diversity",
    STR_EMPTYSPACE
};
#endif

// Gemini Mode
#if defined(GPIO_PIN_NSS_2)
static struct luaItem_selection luaDiversityMode = {
    {"Rx Mode", CRSF_TEXT_SELECTION},
    0, // value
    "Diversity;Gemini",
    STR_EMPTYSPACE
};
#endif

static struct luaItem_folder luaTeamraceFolder = {
    {"Team Race", CRSF_FOLDER},
};

static struct luaItem_selection luaTeamraceChannel = {
    {"Channel", CRSF_TEXT_SELECTION},
    0, // value
    "AUX2;AUX3;AUX4;AUX5;AUX6;AUX7;AUX8;AUX9;AUX10;AUX11;AUX12",
    STR_EMPTYSPACE
};

static struct luaItem_selection luaTeamracePosition = {
    {"Position", CRSF_TEXT_SELECTION},
    0, // value
    "Disabled;1/Low;2;3;Mid;4;5;6/High",
    STR_EMPTYSPACE
};

//----------------------------Info-----------------------------------

static struct luaItem_string luaModelNumber = {
    {"Model Id", CRSF_INFO},
    modelString
};

static struct luaItem_string luaELRSversion = {
    {version, CRSF_INFO},
    commit
};

//----------------------------Info-----------------------------------

//---------------------------- WiFi -----------------------------


//---------------------------- WiFi -----------------------------

//---------------------------- Output Mapping -----------------------------

#if defined(GPIO_PIN_PWM_OUTPUTS)
static struct luaItem_folder luaMappingFolder = {
    {"Output Mapping", CRSF_FOLDER},
};

static struct luaItem_int8 luaMappingChannelOut = {
  {"Output Ch", CRSF_UINT8},
  {
    {
      (uint8_t)5,       // value - start on AUX1, value is 1-16, not zero-based
      1,                // min
      PWM_MAX_CHANNELS, // max
    }
  },
  STR_EMPTYSPACE
};

static struct luaItem_int8 luaMappingChannelIn = {
  {"Input Ch", CRSF_UINT8},
  {
    {
      0,                 // value
      1,                 // min
      CRSF_NUM_CHANNELS, // max
    }
  },
  STR_EMPTYSPACE
};

static struct luaItem_selection luaMappingOutputMode = {
    {"Output Mode", CRSF_TEXT_SELECTION},
    0, // value
    pwmModes,
    STR_EMPTYSPACE
};

static struct luaItem_selection luaMappingInverted = {
    {"Invert", CRSF_TEXT_SELECTION},
    0, // value
    "Off;On",
    STR_EMPTYSPACE
};

static struct luaItem_command luaSetFailsafe = {
    {"Set Failsafe Pos", CRSF_COMMAND},
    lcsIdle, // step
    STR_EMPTYSPACE
};

#endif // GPIO_PIN_PWM_OUTPUTS

//---------------------------- Stabilizer -----------------------------

#if defined(STABILIZER_ENABLE) && defined(GPIO_PIN_PWM_OUTPUTS)
static struct luaItem_folder luaStabilizerFolder = {
    {"Stabilizer", CRSF_FOLDER},
};

// 菜单说明：
// 2026-08-26 精简：保留 IMU 状态 + PDI 参数 + 机身布局/副翼/模式开关通道配置；
//   移除快照显示/原始数据/诊断计数（走 CRSF ATTITUDE 遥测与 LUA UpdCnt 等）。
// 2026-08-26 增：Roll Dir / Pitch Dir 轴方向开关（独立于 Invert，治传感器装反）。
// 2026-09-06（修改点 23）：三档模式（低=直通 / 中=Rate / 高=Angle），俯仰全模式
//   直通 → Pitch Kp/Kd/Pitch Dir 从菜单隐藏（config 字段保留）；仅横滚介入。
static struct luaItem_selection luaStabAirframe = {
    {"Airframe", CRSF_TEXT_SELECTION},
    0, // value
    "Normal;Wing",
    STR_EMPTYSPACE
};

static struct luaItem_selection luaStabAileron = {
    {"Aileron", CRSF_TEXT_SELECTION},
    0, // value
    "1 Servo;2 Servo",
    STR_EMPTYSPACE
};

static struct luaItem_selection luaStabGainCh = {
    {"Gain Switch", CRSF_TEXT_SELECTION},
    0, // value
    "CH1;CH2;CH3;CH4;CH5;CH6;CH7;CH8;CH9;CH10;CH11;CH12",
    STR_EMPTYSPACE
};

static struct luaItem_int8 luaStabRollKp = {
  {"Roll Kp", CRSF_UINT8},
  {
    {
      (uint8_t)10,      // value (×0.1 = 1.0)
      (uint8_t)0,       // min
      (uint8_t)25,      // max (2.5)
    }
  },
  STR_EMPTYSPACE
};

static struct luaItem_int8 luaStabRollKd = {
  {"Roll Kd", CRSF_UINT8},
  {
    {
      (uint8_t)5,       // value (×0.01 = 0.05)
      (uint8_t)0,       // min
      (uint8_t)50,      // max (0.50)
    }
  },
  STR_EMPTYSPACE
};

// 修改点 24：Rate 档独立参数（与 Angle 档 Roll Kp/Kd 完全解耦，各自可调）。
// 背景：Rate 档原为固定宏（增益 2.0、限幅 ±410us），实测反应过快、左右晃动、
// 舵面角度过大（是 Angle 档 ±105us 的 4 倍）。现改为可调并下调默认值，
// 另加死区抑制静止/手抖时的抖舵（输出低通为写死宏，不占菜单）。
static struct luaItem_int8 luaStabRateGain = {
  {"Rate Gain", CRSF_UINT8},
  {
    {
      (uint8_t)8,       // value (×0.1 = 0.8)
      (uint8_t)0,       // min
      (uint8_t)30,      // max (3.0)
    }
  },
  STR_EMPTYSPACE
};

static struct luaItem_int8 luaStabRateLimit = {
  {"Rate Limit", CRSF_UINT8},
  {
    {
      (uint8_t)180,     // value (us，≈±18% 满行程)
      (uint8_t)0,       // min
      (uint8_t)255,     // max
    }
  },
  STR_EMPTYSPACE
};

static struct luaItem_int8 luaStabRateDead = {
  {"Rate Dead", CRSF_UINT8},
  {
    {
      (uint8_t)8,       // value (°/s 死区)
      (uint8_t)0,       // min
      (uint8_t)30,      // max
    }
  },
  STR_EMPTYSPACE
};

// 修改点 23：俯仰全模式直通，Pitch Kp/Kd/Pitch Dir 已无作用，从菜单隐藏。
// （config 字段保留不删，避免动 EEPROM magic；恢复俯仰介入时取消注册即可。）
// static struct luaItem_int8 luaStabPitchKp = { ... };
// static struct luaItem_int8 luaStabPitchKd = { ... };
// static struct luaItem_selection luaStabPitchDir = { ... };

// 轴方向开关：只翻 IMU 修正方向（独立于通道映射页的 Invert）。
// 用于传感器装反等"摇杆方向正确、自稳方向反"的场景。
static struct luaItem_selection luaStabRollDir = {
    {"Roll Dir", CRSF_TEXT_SELECTION},
    0, // value
    "Normal;Rev",
    STR_EMPTYSPACE
};

// CH6 反向：普通布局 + 2 副翼舵机时，左副翼(CH6)直通信号绕中位反向，
// 使左右舵面差动（等效遥控器双副翼左通道反向）。只翻直通信号，
// 自稳修正仍为 -roll 差动不受影响；与通道映射页 Invert(直通+修正一起翻)完全独立。
static struct luaItem_selection luaStab6chRev = {
    {"6CH Rev", CRSF_TEXT_SELECTION},
    0, // value
    "Normal;Rev",
    STR_EMPTYSPACE
};

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

static void luaparamStabAirframe(struct luaPropertiesCommon *item, uint8_t arg)
{
    UNUSED(item);
    config.SetStabAirframe(arg);
}

static void luaparamStabAileron(struct luaPropertiesCommon *item, uint8_t arg)
{
    UNUSED(item);
    config.SetStabAileron(arg);
}

static void luaparamStabGainCh(struct luaPropertiesCommon *item, uint8_t arg)
{
    UNUSED(item);
    config.SetStabGainCh(arg);
}

static void luaparamStabRollKp(struct luaPropertiesCommon *item, uint8_t arg)
{
    UNUSED(item);
    config.SetStabRollKp(arg);
}

static void luaparamStabRollKd(struct luaPropertiesCommon *item, uint8_t arg)
{
    UNUSED(item);
    config.SetStabRollKd(arg);
}

static void luaparamStabRateGain(struct luaPropertiesCommon *item, uint8_t arg)
{
    UNUSED(item);
    config.SetStabRateGain(arg);
}

static void luaparamStabRateLimit(struct luaPropertiesCommon *item, uint8_t arg)
{
    UNUSED(item);
    config.SetStabRateLimit(arg);
}

static void luaparamStabRateDead(struct luaPropertiesCommon *item, uint8_t arg)
{
    UNUSED(item);
    config.SetStabRateDeadband(arg);
}

// 修改点 23：随 luaStabPitchKp/Kd/PitchDir 一并隐藏（见上方定义处注释）。
// static void luaparamStabPitchKp(...) { config.SetStabPitchKp(arg); }
// static void luaparamStabPitchKd(...) { config.SetStabPitchKd(arg); }
// static void luaparamStabPitchDir(...) { config.SetStabPitchDir(arg); }

static void luaparamStabRollDir(struct luaPropertiesCommon *item, uint8_t arg)
{
    UNUSED(item);
    config.SetStabRollDir(arg);
}

static void luaparamStab6chRev(struct luaPropertiesCommon *item, uint8_t arg)
{
    UNUSED(item);
    config.SetStab6chRev(arg);
}

#endif // STABILIZER_ENABLE && GPIO_PIN_PWM_OUTPUTS

//---------------------------- Stabilizer -----------------------------

static struct luaItem_selection luaBindStorage = {
    {"Bind Storage", CRSF_TEXT_SELECTION},
    0, // value
    "Persistent;Volatile;Returnable;Administered",
    STR_EMPTYSPACE
};

static struct luaItem_command luaBindMode = {
    {STR_EMPTYSPACE, CRSF_COMMAND},
    lcsIdle, // step
    STR_EMPTYSPACE
};

#if defined(GPIO_PIN_PWM_OUTPUTS)
static void luaparamMappingChannelOut(struct luaPropertiesCommon *item, uint8_t arg)
{
    bool sclAssigned = false;
    bool sdaAssigned = false;
#if defined(PLATFORM_ESP32)
    bool serial1rxAssigned = false;
    bool serial1txAssigned = false;
#endif

    const char *no1Option    = ";";
    const char *no2Options   = ";;";
    const char *dshot        = ";DShot";
    const char *serial_RX    = ";Serial RX";
    const char *serial_TX    = ";Serial TX";
    const char *i2c_SCL      = ";I2C SCL;";
    const char *i2c_SDA      = ";;I2C SDA";
    const char *i2c_BOTH     = ";I2C SCL;I2C SDA";
#if defined(PLATFORM_ESP32)
    const char *serial1_RX   = ";Serial2 RX;";
    const char *serial1_TX   = ";;Serial2 TX";
    const char *serial1_BOTH = ";Serial2 RX;Serial2 TX";
#endif

    const char *pModeString;


    // find out if use once only modes have already been assigned
    for (uint8_t ch = 0; ch < GPIO_PIN_PWM_OUTPUTS_COUNT; ch++)
    {
      if (ch == (arg -1))
        continue;

      eServoOutputMode mode = (eServoOutputMode)config.GetPwmChannel(ch)->val.mode;

      if (mode == somSCL)
        sclAssigned = true;

      if (mode == somSDA)
        sdaAssigned = true;

#if defined(PLATFORM_ESP32)
      if (mode == somSerial1RX)
        serial1rxAssigned = true;

      if (mode == somSerial1TX)
        serial1txAssigned = true;
#endif
    }

    setLuaUint8Value(&luaMappingChannelOut, arg);

    // When the selected output channel changes, update the available PWM modes for that pin
    // Truncate the select options before the ; following On/Off
    pwmModes[50] = '\0';

#if defined(PLATFORM_ESP32)
    // DShot output (1 option)
    // ;DShot
    // ESP8266 enum skips this, so it is never present
    if (GPIO_PIN_PWM_OUTPUTS[arg-1] != 0)   // DShot doesn't work with GPIO0, exclude it
    {
        pModeString = dshot;
    }
    else
#endif
    {
        pModeString = no1Option;
    }
    strcat(pwmModes, pModeString);

    // SerialIO outputs (1 option)
    // ;[Serial RX] | [Serial TX]
    if (GPIO_PIN_PWM_OUTPUTS[arg-1] == U0RXD_GPIO_NUM)
    {
        pModeString = serial_RX;
    }
    else if (GPIO_PIN_PWM_OUTPUTS[arg-1] == U0TXD_GPIO_NUM)
    {
        pModeString = serial_TX;
    }
    else
    {
        pModeString = no1Option;
    }
    strcat(pwmModes, pModeString);

    // I2C pins (2 options)
    // ;[I2C SCL] ;[I2C SDA]
    if (GPIO_PIN_SCL != UNDEF_PIN || GPIO_PIN_SDA != UNDEF_PIN)
    {
        // If the target defines SCL/SDA then those pins MUST be used
        if (GPIO_PIN_PWM_OUTPUTS[arg-1] == GPIO_PIN_SCL)
        {
            pModeString = i2c_SCL;
        }
        else if (GPIO_PIN_PWM_OUTPUTS[arg-1] == GPIO_PIN_SDA)
        {
            pModeString = i2c_SDA;
        }
        else
        {
            pModeString = no2Options;
        }
    }
    else
    {
        // otherwise allow any pin to be either SCL or SDA but only once
        if (sclAssigned && !sdaAssigned)
        {
            pModeString = i2c_SDA;
        }
        else if (sdaAssigned && !sclAssigned)
        {
            pModeString = i2c_SCL;
        }
        else if (!sclAssigned && !sdaAssigned)
        {
            pModeString = i2c_BOTH;
        }
        else
        {
            pModeString = no2Options;
        }
    }
    strcat(pwmModes, pModeString);

    // nothing to do for unsupported somPwm mode
    strcat(pwmModes, no1Option);

#if defined(PLATFORM_ESP32)
    // secondary Serial pins (2 options)
    // ;[SERIAL2 RX] ;[SERIAL2_TX]
    if (GPIO_PIN_SERIAL1_RX != UNDEF_PIN || GPIO_PIN_SERIAL1_TX != UNDEF_PIN)
    {
        // If the target defines Serial2 RX/TX then those pins MUST be used
        if (GPIO_PIN_PWM_OUTPUTS[arg-1] == GPIO_PIN_SERIAL1_RX)
        {
            pModeString = serial1_RX;
        }
        else if (GPIO_PIN_PWM_OUTPUTS[arg-1] == GPIO_PIN_SERIAL1_TX)
        {
            pModeString = serial1_TX;
        }
        else
        { 
            pModeString = no2Options;
        }
    } 
    else
    {   // otherwise allow any pin to be either RX or TX but only once
        if (serial1txAssigned && !serial1rxAssigned)
        {
            pModeString = serial1_RX;
        }        
        else if (serial1rxAssigned && !serial1txAssigned)
        {
            pModeString = serial1_TX;
        }

        else if (!serial1rxAssigned && !serial1txAssigned)
        {
            pModeString = serial1_BOTH;
        } 
        else
        {
            pModeString = no2Options;
        }
    }
    strcat(pwmModes, pModeString);
#endif

    // trim off trailing semicolons (assumes pwmModes has at least 1 non-semicolon)
    for (auto lastPos = strlen(pwmModes)-1; pwmModes[lastPos] == ';'; lastPos--)
    {
        pwmModes[lastPos] = '\0';
    }

    // Trigger an event to update the related fields to represent the selected channel
    devicesTriggerEvent();
}

static void luaparamMappingChannelIn(struct luaPropertiesCommon *item, uint8_t arg)
{
  const uint8_t ch = luaMappingChannelOut.properties.u.value - 1;
  rx_config_pwm_t newPwmCh;
  newPwmCh.raw = config.GetPwmChannel(ch)->raw;
  newPwmCh.val.inputChannel = arg - 1; // convert 1-16 -> 0-15

  config.SetPwmChannelRaw(ch, newPwmCh.raw);
}

static void configureSerialPin(uint8_t sibling, uint8_t oldMode, uint8_t newMode)
{
  for (int ch=0 ; ch<GPIO_PIN_PWM_OUTPUTS_COUNT ; ch++)
  {
    if (GPIO_PIN_PWM_OUTPUTS[ch] == sibling)
    {
      // Retain as much of the sibling's current config as possible
      rx_config_pwm_t siblingPinConfig;
      siblingPinConfig.raw = config.GetPwmChannel(ch)->raw;

      // If the new mode is serial, the sibling is also forced to serial
      if (newMode == somSerial)
      {
        siblingPinConfig.val.mode = somSerial;
      }
      // If the new mode is not serial, and the sibling is serial, set the sibling to PWM (50Hz)
      else if (siblingPinConfig.val.mode == somSerial)
      {
        siblingPinConfig.val.mode = som50Hz;
      }

      config.SetPwmChannelRaw(ch, siblingPinConfig.raw);
      break;
    }
  }

  if (oldMode != newMode)
  {
    deferExecutionMillis(100, [](){
      reconfigureSerial();
    });
  }
}

static void luaparamMappingOutputMode(struct luaPropertiesCommon *item, uint8_t arg)
{
  UNUSED(item);
  const uint8_t ch = luaMappingChannelOut.properties.u.value - 1;
  rx_config_pwm_t newPwmCh;
  newPwmCh.raw = config.GetPwmChannel(ch)->raw;
  uint8_t oldMode = newPwmCh.val.mode;
  newPwmCh.val.mode = arg;

  // Check if pin == 1/3 and do other pin adjustment accordingly
  if (GPIO_PIN_PWM_OUTPUTS[ch] == 1)
  {
    configureSerialPin(3, oldMode, newPwmCh.val.mode);
  }
  else if (GPIO_PIN_PWM_OUTPUTS[ch] == 3)
  {
    configureSerialPin(1, oldMode, newPwmCh.val.mode);
  }
  config.SetPwmChannelRaw(ch, newPwmCh.raw);
}

static void luaparamMappingInverted(struct luaPropertiesCommon *item, uint8_t arg)
{
  UNUSED(item);
  const uint8_t ch = luaMappingChannelOut.properties.u.value - 1;
  rx_config_pwm_t newPwmCh;
  newPwmCh.raw = config.GetPwmChannel(ch)->raw;
  newPwmCh.val.inverted = arg;

  config.SetPwmChannelRaw(ch, newPwmCh.raw);
}

static void luaparamSetFailsafe(struct luaPropertiesCommon *item, uint8_t arg)
{
  luaCmdStep_e newStep;
  const char *msg;
  if (arg == lcsClick)
  {
    newStep = lcsAskConfirm;
    msg = "Set failsafe to curr?";
  }
  else if (arg == lcsConfirmed)
  {
    // This is generally not seen by the user, since we'll disconnect to commit config
    // and the handset will send another lcdQuery that will overwrite it with idle
    newStep = lcsExecuting;
    msg = "Setting failsafe";

    for (int ch=0; ch<GPIO_PIN_PWM_OUTPUTS_COUNT; ++ch)
    {
      rx_config_pwm_t newPwmCh;
      // The value must fit into the 10 bit range of the failsafe
      newPwmCh.raw = config.GetPwmChannel(ch)->raw;
      newPwmCh.val.failsafe = CRSF_to_UINT10(constrain(ChannelData[config.GetPwmChannel(ch)->val.inputChannel], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX));
      //DBGLN("FSCH(%u) crsf=%u us=%u", ch, ChannelData[ch], newPwmCh.val.failsafe+988U);
      config.SetPwmChannelRaw(ch, newPwmCh.raw);
    }
  }
  else
  {
    newStep = lcsIdle;
    msg = STR_EMPTYSPACE;
  }

  sendLuaCommandResponse((struct luaItem_command *)item, newStep, msg);
}

#endif // GPIO_PIN_PWM_OUTPUTS

#if defined(POWER_OUTPUT_VALUES)

static void luaparamSetPower(struct luaPropertiesCommon* item, uint8_t arg)
{
  UNUSED(item);
  uint8_t newPower = arg + POWERMGNT::getMinPower();
  if (newPower > POWERMGNT::getMaxPower())
  {
    newPower = PWR_MATCH_TX;
  }

  config.SetPower(newPower);
  // POWERMGNT::setPower() will be called in updatePower() in the main loop
}

#endif // POWER_OUTPUT_VALUES

static void registerLuaParameters()
{
  registerLUAParameter(&luaSerialProtocol, [](struct luaPropertiesCommon* item, uint8_t arg){
    config.SetSerialProtocol((eSerialProtocol)arg);
    if (config.IsModified()) {
      deferExecutionMillis(100, [](){
        reconfigureSerial();
      });
    }
  });

#if defined(PLATFORM_ESP32)
  if (RX_HAS_SERIAL1)
  {
    registerLUAParameter(&luaSerial1Protocol, [](struct luaPropertiesCommon* item, uint8_t arg){
      config.SetSerial1Protocol((eSerial1Protocol)arg);
      if (config.IsModified()) {
        deferExecutionMillis(100, [](){
          reconfigureSerial1();
        });
      }
    });
  }
#endif

  registerLUAParameter(&luaSBUSFailsafeMode, [](struct luaPropertiesCommon* item, uint8_t arg){
    config.SetFailsafeMode((eFailsafeMode)arg);
  });

#if !defined(PLATFORM_STM32)
  registerLUAParameter(&luaTargetSysId, [](struct luaPropertiesCommon* item, uint8_t arg){
    config.SetTargetSysId((uint8_t)arg);
  });
  registerLUAParameter(&luaSourceSysId, [](struct luaPropertiesCommon* item, uint8_t arg){
    config.SetSourceSysId((uint8_t)arg);
  });
#endif

  if (GPIO_PIN_ANT_CTRL != UNDEF_PIN)
  {
    registerLUAParameter(&luaAntennaMode, [](struct luaPropertiesCommon* item, uint8_t arg){
      config.SetAntennaMode(arg);
    });
  }

  // Gemini Mode
  if (isDualRadio())
  {
    registerLUAParameter(&luaDiversityMode, [](struct luaPropertiesCommon* item, uint8_t arg){
      config.SetAntennaMode(arg); // Reusing SetAntennaMode since both GPIO_PIN_ANTENNA_SELECT and GPIO_PIN_NSS_2 will not be defined together.
    });
  }

#if defined(POWER_OUTPUT_VALUES)
  luadevGeneratePowerOpts(&luaTlmPower);
  registerLUAParameter(&luaTlmPower, &luaparamSetPower);
#endif

  // Teamrace
  registerLUAParameter(&luaTeamraceFolder);
  registerLUAParameter(&luaTeamraceChannel, [](struct luaPropertiesCommon* item, uint8_t arg) {
    config.SetTeamraceChannel(arg + AUX2);
  }, luaTeamraceFolder.common.id);
  registerLUAParameter(&luaTeamracePosition, [](struct luaPropertiesCommon* item, uint8_t arg) {
    config.SetTeamracePosition(arg);
  }, luaTeamraceFolder.common.id);

#if defined(GPIO_PIN_PWM_OUTPUTS)
  if (OPT_HAS_SERVO_OUTPUT)
  {
    luaparamMappingChannelOut(&luaMappingOutputMode.common, luaMappingChannelOut.properties.u.value);
    registerLUAParameter(&luaMappingFolder);
    registerLUAParameter(&luaMappingChannelOut, &luaparamMappingChannelOut, luaMappingFolder.common.id);
    registerLUAParameter(&luaMappingChannelIn, &luaparamMappingChannelIn, luaMappingFolder.common.id);
    registerLUAParameter(&luaMappingOutputMode, &luaparamMappingOutputMode, luaMappingFolder.common.id);
    registerLUAParameter(&luaMappingInverted, &luaparamMappingInverted, luaMappingFolder.common.id);
    registerLUAParameter(&luaSetFailsafe, &luaparamSetFailsafe);
  }
#endif

#if defined(STABILIZER_ENABLE) && defined(GPIO_PIN_PWM_OUTPUTS)
  if (OPT_HAS_SERVO_OUTPUT)
  {
    registerLUAParameter(&luaStabilizerFolder);
    registerLUAParameter(&luaStabAirframe, &luaparamStabAirframe, luaStabilizerFolder.common.id);
    registerLUAParameter(&luaStabAileron, &luaparamStabAileron, luaStabilizerFolder.common.id);
    registerLUAParameter(&luaStabGainCh, &luaparamStabGainCh, luaStabilizerFolder.common.id);
    registerLUAParameter(&luaStabRollKp, &luaparamStabRollKp, luaStabilizerFolder.common.id);
    registerLUAParameter(&luaStabRollKd, &luaparamStabRollKd, luaStabilizerFolder.common.id);
    registerLUAParameter(&luaStabRateGain, &luaparamStabRateGain, luaStabilizerFolder.common.id);
    registerLUAParameter(&luaStabRateLimit, &luaparamStabRateLimit, luaStabilizerFolder.common.id);
    registerLUAParameter(&luaStabRateDead, &luaparamStabRateDead, luaStabilizerFolder.common.id);
    registerLUAParameter(&luaStabRollDir, &luaparamStabRollDir, luaStabilizerFolder.common.id);
    registerLUAParameter(&luaStab6chRev, &luaparamStab6chRev, luaStabilizerFolder.common.id);
    // 调试状态参数：不注册回调 → 只读（遥控器侧修改会被忽略）
    registerLUAParameter(&luaStabImuStat, nullptr, luaStabilizerFolder.common.id);
  }
#endif

  registerLUAParameter(&luaBindStorage, [](struct luaPropertiesCommon* item, uint8_t arg) {
    config.SetBindStorage((rx_config_bindstorage_t)arg);
  });
  registerLUAParameter(&luaBindMode, [](struct luaPropertiesCommon* item, uint8_t arg){
    // Complete when TX polls for status i.e. going back to idle, because we're going to lose connection
    if (arg == lcsQuery) {
      deferExecutionMillis(200, EnterBindingModeSafely);
    }
    sendLuaCommandResponse(&luaBindMode, arg < 5 ? lcsExecuting : lcsIdle, arg < 5 ? "Entering..." : "");
  });

  registerLUAParameter(&luaModelNumber);
  registerLUAParameter(&luaELRSversion);
}

static void updateBindModeLabel()
{
  if (config.IsOnLoan())
    luaBindMode.common.name = "Return Model";
  else
    luaBindMode.common.name = "Enter Bind Mode";
}

static int event()
{
  setLuaTextSelectionValue(&luaSerialProtocol, config.GetSerialProtocol());
#if defined(PLATFORM_ESP32)
  if (RX_HAS_SERIAL1)
  {
    setLuaTextSelectionValue(&luaSerial1Protocol, config.GetSerial1Protocol());
  }
#endif
  
  setLuaTextSelectionValue(&luaSBUSFailsafeMode, config.GetFailsafeMode());

  if (GPIO_PIN_ANT_CTRL != UNDEF_PIN)
  {
    setLuaTextSelectionValue(&luaAntennaMode, config.GetAntennaMode());
  }

  // Gemini Mode
  if (isDualRadio())
  {
    setLuaTextSelectionValue(&luaDiversityMode, config.GetAntennaMode()); // Reusing SetAntennaMode since both GPIO_PIN_ANTENNA_SELECT and GPIO_PIN_NSS_2 will not be defined together.
  }

#if defined(POWER_OUTPUT_VALUES)
  // The last item (for MatchTX) will be MaxPower - MinPower + 1
  uint8_t luaPwrVal = (config.GetPower() == PWR_MATCH_TX) ? POWERMGNT::getMaxPower() + 1 : config.GetPower();
  setLuaTextSelectionValue(&luaTlmPower, luaPwrVal - POWERMGNT::getMinPower());
#endif

  // Teamrace
  setLuaTextSelectionValue(&luaTeamraceChannel, config.GetTeamraceChannel() - AUX2);
  setLuaTextSelectionValue(&luaTeamracePosition, config.GetTeamracePosition());

#if defined(GPIO_PIN_PWM_OUTPUTS)
  if (OPT_HAS_SERVO_OUTPUT)
  {
    const rx_config_pwm_t *pwmCh = config.GetPwmChannel(luaMappingChannelOut.properties.u.value - 1);
    setLuaUint8Value(&luaMappingChannelIn, pwmCh->val.inputChannel + 1);
    setLuaTextSelectionValue(&luaMappingOutputMode, pwmCh->val.mode);
    setLuaTextSelectionValue(&luaMappingInverted, pwmCh->val.inverted);
  }
#endif

#if defined(STABILIZER_ENABLE) && defined(GPIO_PIN_PWM_OUTPUTS)
  if (OPT_HAS_SERVO_OUTPUT)
  {
    setLuaTextSelectionValue(&luaStabAirframe, config.GetStabAirframe());
    setLuaTextSelectionValue(&luaStabAileron, config.GetStabAileron());
    setLuaTextSelectionValue(&luaStabGainCh, config.GetStabGainCh());
    setLuaUint8Value(&luaStabRollKp, config.GetStabRollKp());
    setLuaUint8Value(&luaStabRollKd, config.GetStabRollKd());
    setLuaUint8Value(&luaStabRateGain, config.GetStabRateGain());
    setLuaUint8Value(&luaStabRateLimit, config.GetStabRateLimit());
    setLuaUint8Value(&luaStabRateDead, config.GetStabRateDeadband());
    setLuaTextSelectionValue(&luaStabRollDir, config.GetStabRollDir());
    setLuaTextSelectionValue(&luaStab6chRev, config.GetStab6chRev());
  }
#endif

  if (config.GetModelId() == 255)
  {
    setLuaStringValue(&luaModelNumber, "Off");
  }
  else
  {
    itoa(config.GetModelId(), modelString, 10);
    setLuaStringValue(&luaModelNumber, modelString);
  }
  setLuaTextSelectionValue(&luaBindStorage, config.GetBindStorage());
  updateBindModeLabel();

#if !defined(PLATFORM_STM32)
  if (config.GetSerialProtocol() == PROTOCOL_MAVLINK)
  {
    setLuaUint8Value(&luaSourceSysId, config.GetSourceSysId() == 0 ? 255 : config.GetSourceSysId());  //display Source sysID if 0 display 255 to mimic logic in SerialMavlink.cpp
    setLuaUint8Value(&luaTargetSysId, config.GetTargetSysId() == 0 ? 1 : config.GetTargetSysId());  //display Target sysID if 0 display 1 to mimic logic in SerialMavlink.cpp
    LUA_FIELD_SHOW(luaSourceSysId)
    LUA_FIELD_SHOW(luaTargetSysId)
  }
  else
  {
    LUA_FIELD_HIDE(luaSourceSysId)
    LUA_FIELD_HIDE(luaTargetSysId)
  }
#endif

  return DURATION_IMMEDIATELY;
}

static int timeout()
{
#if defined(STABILIZER_ENABLE) && defined(GPIO_PIN_PWM_OUTPUTS)
  if (OPT_HAS_SERVO_OUTPUT)
  {
    // 调试状态：IMU 健康标志（未初始化时保持 0）
    // event() 只在连接状态变化等事件时触发，动态值必须在这里周期性刷新
    // （连接时 interval/250 ms，未连接 1s），遥控器 LUA 菜单读取时即为最新值
    setLuaUint8Value(&luaStabImuStat, stabilizer.imuOk() ? 1 : 0);
  }
#endif
  luaHandleUpdateParameter();
  // Receivers can only `UpdateParamReq == true` every 4th packet due to the transmitter cadence in 1:2
  // Channels, Downlink Telemetry Slot, Uplink Telemetry (the write command), Downlink Telemetry Slot...
  // (interval * 4 / 1000) or 1 second if not connected
  return (connectionState == connected) ? ExpressLRS_currAirRate_Modparams->interval / 250 : 1000;
}

static int start()
{
  registerLuaParameters();
  event();
  return DURATION_IMMEDIATELY;
}

device_t LUA_device = {
  .initialize = nullptr,
  .start = start,
  .event = event,
  .timeout = timeout
};

#endif
