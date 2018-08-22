#pragma once

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
                                                            COMMAND NAMES SECTION 
*/

// Need to add command macro with sequental number
#define CMD_ZBX_NOPE                                            (0x00)
#define CMD_ZBX_AGENT_PING                                      (0x01)
#define CMD_ZBX_AGENT_HOSTNAME                                  (0x02)
#define CMD_ZBX_AGENT_VERSION                                   (0x03)
#define CMD_SYSTEM_UPTIME                                       (0x04)
                                                                
#define CMD_ARDUINO_ANALOGWRITE                                 (0x05)
#define CMD_ARDUINO_ANALOGREAD                                  (0x06)
#define CMD_ARDUINO_ANALOGREFERENCE                             (0x07)
#define CMD_ARDUINO_DELAY                                       (0x08)
#define CMD_ARDUINO_DIGITALWRITE                                (0x09)
#define CMD_ARDUINO_DIGITALREAD                                 (0x0A)
#define CMD_ARDUINO_TONE                                        (0x0B)
#define CMD_ARDUINO_NOTONE                                      (0x0C)
#define CMD_ARDUINO_RANDOMSEED                                  (0x0D)
#define CMD_ARDUINO_RANDOM                                      (0x0E)

#define CMD_SET_HOSTNAME                                        (0x0F)
#define CMD_SET_NETWORK                                         (0x10)
#define CMD_SET_PASSWORD                                        (0x11)
#define CMD_SET_SYSPROTECT                                      (0x12)

#define CMD_SET_LOCALTIME                                       (0x13)
                                                                    
#define CMD_SYS_PORTWRITE                                       (0x14)
#define CMD_SYS_SHIFTOUT                                        (0x15)
#define CMD_SYS_REBOOT                                          (0x16)
                                                                    
#define CMD_SYSTEM_LOCALTIME                                    (0x17)
#define CMD_SYSTEM_RUN                                          (0x18)

#define CMD_SYSTEM_HW_CHASSIS                                   (0x19)
#define CMD_SYSTEM_HW_CPU                                       (0x1A)
#define CMD_NET_PHY_NAME                                        (0x1B)
#define CMD_NET_PHY_REINITS                                     (0x1C)
                                                                    
#define CMD_SYS_CMD_COUNT                                       (0x1D)
#define CMD_SYS_CMD_TIMEMAX                                     (0x1E)
#define CMD_SYS_CMD_TIMEMAX_N                                   (0x1F)
                                                                    
#define CMD_SYS_RAM_FREE                                        (0x20)
#define CMD_SYS_RAM_FREEMIN                                     (0x21)

#define CMD_SYS_VCC                                             (0x22)
#define CMD_SYS_VCCMIN                                          (0x23)
#define CMD_SYS_VCCMAX                                          (0x24)
                                                                    
#define CMD_EXTINT_COUNT                                        (0x25)
#define CMD_INCENC_VALUE                                        (0x26)
                                                                    
#define CMD_OW_SCAN                                             (0x27)
                                                                    
#define CMD_I2C_SCAN                                            (0x28)
#define CMD_I2C_WRITE                                           (0x29)
#define CMD_I2C_READ                                            (0x2A)
#define CMD_I2C_BITWRITE                                        (0x2B)
#define CMD_I2C_BITREAD                                         (0x2C)
                                                                    
#define CMD_DS18X20_TEMPERATURE                                 (0x2D)
                                                                    
#define CMD_DHT_HUMIDITY                                        (0x2E)
#define CMD_DHT_TEMPERATURE                                     (0x2F)
                                                                    
#define CMD_BMP_PRESSURE                                        (0x30) 
#define CMD_BMP_TEMPERATURE                                     (0x31)
#define CMD_BME_HUMIDITY                                        (0x32)
                                                                    
#define CMD_BH1750_LIGHT                                        (0x33)

#define CMD_MAX7219_WRITE                                       (0x34)
                                                                    
#define CMD_PCF8574_LCDPRINT                                    (0x35)
                                                                    
#define CMD_SHT2X_HUMIDITY                                      (0x36)
#define CMD_SHT2X_TEMPERATURE                                   (0x37)
                                                                    
#define CMD_ACS7XX_ZC                                           (0x38)
#define CMD_ACS7XX_AC                                           (0x39)
#define CMD_ACS7XX_DC                                           (0x3A)

#define CMD_ULTRASONIC_DISTANCE                                 (0x3B)
                                                                    
#define CMD_IR_SEND                                             (0x3C)
#define CMD_IR_SENDRAW                                          (0x3D)
                                                                    
#define CMD_WS2812_SENDRAW                                      (0x3E)
                                                                    
#define CMD_PZEM004_CURRENT                                     (0x3F) 
#define CMD_PZEM004_VOLTAGE                                     (0x40) 
#define CMD_PZEM004_POWER                                       (0x41)  
#define CMD_PZEM004_ENERGY                                      (0x42)
#define CMD_PZEM004_SETADDR                                     (0x43)
                                                                    
#define CMD_UPS_APCSMART                                        (0x44)
#define CMD_UPS_MEGATEC                                         (0x45)
                                                                    
#define CMD_INA219_BUSVOLTAGE                                   (0x46)
#define CMD_INA219_CURRENT                                      (0x47)
#define CMD_INA219_POWER                                        (0x48)
                                                                    
#define CMD_AT24CXX_WRITE                                       (0x49)
#define CMD_AT24CXX_READ                                        (0x4A)

#define CMD_MAX44009_LIGHT                                      (0x4B)

#define CMD_MHZXX_PWM_CO2                                       (0x4C)
#define CMD_MHZXX_UART_CO2                                      (0x4D)

#define CMD_USER_RUN                                            (0x4E)

#define CMD_VEML6070_UV                                         (0x4F)

#define CMD_SERVO_TURN                                          (0x50)

// add new command as "const char command_<COMMAND_MACRO> PROGMEM". Only 'const' push string to PROGMEM. Tanx, Arduino & AVR.
// command_* values must be in lower case due analyze sub convert all chars to lower
const char command_CMD_ZBX_NOPE[]                               PROGMEM = "\1";
const char command_CMD_ZBX_AGENT_PING[]                         PROGMEM = "agent.ping";
const char command_CMD_ZBX_AGENT_HOSTNAME[]                     PROGMEM = "agent.hostname";
const char command_CMD_ZBX_AGENT_VERSION[]                      PROGMEM = "agent.version";
const char command_CMD_SYSTEM_UPTIME[]                          PROGMEM = "system.uptime";

const char command_CMD_ARDUINO_ANALOGWRITE[]                    PROGMEM = "analogwrite";         
const char command_CMD_ARDUINO_ANALOGREAD[]                     PROGMEM = "analogread";
const char command_CMD_ARDUINO_ANALOGREFERENCE[]                PROGMEM = "analogreference";
const char command_CMD_ARDUINO_DELAY[]                          PROGMEM = "delay";
const char command_CMD_ARDUINO_DIGITALWRITE[]                   PROGMEM = "digitalwrite";
const char command_CMD_ARDUINO_DIGITALREAD[]                    PROGMEM = "digitalread"; 

const char command_CMD_ARDUINO_TONE[]                           PROGMEM = "tone";
const char command_CMD_ARDUINO_NOTONE[]                         PROGMEM = "notone";

const char command_CMD_ARDUINO_RANDOMSEED[]                     PROGMEM = "randomseed";
const char command_CMD_ARDUINO_RANDOM[]                         PROGMEM = "random";

const char command_CMD_SET_HOSTNAME[]                           PROGMEM = "set.hostname";
const char command_CMD_SET_NETWORK[]                            PROGMEM = "set.network";
const char command_CMD_SET_PASSWORD[]                           PROGMEM = "set.password";
const char command_CMD_SET_SYSPROTECT[]                         PROGMEM = "set.sysprotect";
const char command_CMD_SET_LOCALTIME[]                          PROGMEM = "set.localtime";

const char command_CMD_SYS_PORTWRITE[]                          PROGMEM = "portwrite";

const char command_CMD_SYS_SHIFTOUT[]                           PROGMEM = "shiftout";

const char command_CMD_SYS_REBOOT[]                             PROGMEM = "reboot";              

const char command_CMD_SYSTEM_LOCALTIME[]                       PROGMEM = "system.localtime";
const char command_CMD_SYSTEM_RUN[]                             PROGMEM = "system.run";

const char command_CMD_SYSTEM_HW_CHASSIS[]                      PROGMEM = "system.hw.chassis";
const char command_CMD_SYSTEM_HW_CPU[]                          PROGMEM = "system.hw.cpu";

const char command_CMD_NET_PHY_NAME[]                           PROGMEM = "net.phy.name";
const char command_CMD_NET_PHY_REINITS[]                        PROGMEM = "net.phy.reinits";

const char command_CMD_SYS_CMD_COUNT[]                          PROGMEM = "sys.cmd.count";
const char command_CMD_SYS_CMD_TIMEMAX[]                        PROGMEM = "sys.cmd.timemax";
const char command_CMD_SYS_CMD_TIMEMAX_N[]                      PROGMEM = "sys.cmd.timemax.n";

const char command_CMD_SYS_RAM_FREE[]                           PROGMEM = "sys.ram.free";
const char command_CMD_SYS_RAM_FREEMIN[]                        PROGMEM = "sys.ram.freemin";

const char command_CMD_SYS_VCC[]                                PROGMEM = "sys.vcc";
const char command_CMD_SYS_VCCMIN[]                             PROGMEM = "sys.vccmin";
const char command_CMD_SYS_VCCMAX[]                             PROGMEM = "sys.vccmax";

const char command_CMD_EXTINT_COUNT[]                           PROGMEM = "extint.count";
const char command_CMD_INCENC_VALUE[]                           PROGMEM = "incenc.value";

const char command_CMD_OW_SCAN[]                                PROGMEM = "ow.scan";

const char command_CMD_I2C_SCAN[]                               PROGMEM = "i2c.scan";
const char command_CMD_I2C_WRITE[]                              PROGMEM = "i2c.write";
const char command_CMD_I2C_READ[]                               PROGMEM = "i2c.read";
const char command_CMD_I2C_BITWRITE[]                           PROGMEM = "i2c.bitwrite";
const char command_CMD_I2C_BITREAD[]                            PROGMEM = "i2c.bitread";

const char command_CMD_DS18X20_TEMPERATURE[]                    PROGMEM = "ds18x20.temperature";

const char command_CMD_DHT_HUMIDITY[]                           PROGMEM = "dht.humidity";
const char command_CMD_DHT_TEMPERATURE[]                        PROGMEM = "dht.temperature";

const char command_CMD_BMP_PRESSURE[]                           PROGMEM = "bmp.pressure";
const char command_CMD_BMP_TEMPERATURE[]                        PROGMEM = "bmp.temperature";
const char command_CMD_BME_HUMIDITY[]                           PROGMEM = "bme.humidity";

const char command_CMD_BH1750_LIGHT[]                           PROGMEM = "bh1750.light";

const char command_CMD_MAX7219_WRITE[]                          PROGMEM = "max7219.write";

const char command_CMD_PCF8574_LCDPRINT[]                       PROGMEM = "pcf8574.lcdprint";

const char command_CMD_SHT2X_HUMIDITY[]                         PROGMEM = "sht2x.humidity";
const char command_CMD_SHT2X_TEMPERATURE[]                      PROGMEM = "sht2x.temperature";

const char command_CMD_ACS7XX_ZC[]                              PROGMEM = "acs7xx.zc";
const char command_CMD_ACS7XX_AC[]                              PROGMEM = "acs7xx.ac";
const char command_CMD_ACS7XX_DC[]                              PROGMEM = "acs7xx.dc";

const char command_CMD_ULTRASONIC_DISTANCE[]                    PROGMEM = "ultrasonic.distance";
const char command_CMD_IR_SEND[]                                PROGMEM = "ir.send";
const char command_CMD_IR_SENDRAW[]                             PROGMEM = "ir.sendraw";

const char command_CMD_WS2812_SENDRAW[]                         PROGMEM = "ws2812.sendraw";

const char command_CMD_PZEM004_CURRENT[]                        PROGMEM = "pzem004.current";
const char command_CMD_PZEM004_VOLTAGE[]                        PROGMEM = "pzem004.voltage";
const char command_CMD_PZEM004_POWER[]                          PROGMEM = "pzem004.power";
const char command_CMD_PZEM004_ENERGY[]                         PROGMEM = "pzem004.energy";
const char command_CMD_PZEM004_SETADDR[]                        PROGMEM = "pzem004.setaddr";

const char command_CMD_UPS_APCSMART[]                           PROGMEM = "ups.apcsmart";
const char command_CMD_UPS_MEGATEC[]                            PROGMEM = "ups.megatec";

const char command_CMD_INA219_BUSVOLTAGE[]                      PROGMEM = "ina219.busvoltage";
const char command_CMD_INA219_CURRENT[]                         PROGMEM = "ina219.current";
const char command_CMD_INA219_POWER[]                           PROGMEM = "ina219.power";

const char command_CMD_AT24CXX_WRITE[]                          PROGMEM = "at24cxx.write";
const char command_CMD_AT24CXX_READ[]                           PROGMEM = "at24cxx.read";

const char command_CMD_MAX44009_LIGHT[]                         PROGMEM = "max44009.light";

const char command_CMD_MHZXX_PWM_CO2[]                          PROGMEM = "mhzxx.pwm.co2";
const char command_CMD_MHZXX_UART_CO2[]                         PROGMEM = "mhzxx.uart.co2";

const char command_CMD_USER_RUN[]                               PROGMEM = "user.run";

const char command_CMD_VEML6070_UV[]                            PROGMEM = "veml6070.uv";

const char command_CMD_SERVO_TURN[]                             PROGMEM = "servo.turn";

// do not insert new command to any position without syncing indexes. Tanx, Arduino and AVR, for this method of string array pushing to PROGMEM
// ~300 bytes of PROGMEM space can be saved with crazy "#ifdef-#else-#endif" dance
const char* const commands[] PROGMEM = {
  command_CMD_ZBX_NOPE,

  command_CMD_ZBX_AGENT_PING,
  command_CMD_ZBX_AGENT_HOSTNAME,
  command_CMD_ZBX_AGENT_VERSION,
  command_CMD_SYSTEM_UPTIME,

  command_CMD_ARDUINO_ANALOGWRITE,
  command_CMD_ARDUINO_ANALOGREAD,

#ifdef FEATURE_AREF_ENABLE
  command_CMD_ARDUINO_ANALOGREFERENCE,
#else
  command_CMD_ZBX_NOPE,
#endif

  command_CMD_ARDUINO_DELAY,
  command_CMD_ARDUINO_DIGITALWRITE,
  command_CMD_ARDUINO_DIGITALREAD,

#ifdef FEATURE_TONE_ENABLE
  command_CMD_ARDUINO_TONE,
  command_CMD_ARDUINO_NOTONE,
#else
  command_CMD_ZBX_NOPE,
  command_CMD_ZBX_NOPE,
#endif

#ifdef FEATURE_RANDOM_ENABLE
  command_CMD_ARDUINO_RANDOMSEED,
  command_CMD_ARDUINO_RANDOM,
#else
  command_CMD_ZBX_NOPE,
  command_CMD_ZBX_NOPE,
#endif

#ifdef FEATURE_EEPROM_ENABLE
  command_CMD_SET_HOSTNAME,
  command_CMD_SET_NETWORK,
  command_CMD_SET_PASSWORD,
  command_CMD_SET_SYSPROTECT,
#else
  command_CMD_ZBX_NOPE,
  command_CMD_ZBX_NOPE,
  command_CMD_ZBX_NOPE,
  command_CMD_ZBX_NOPE,
#endif

#ifdef FEATURE_SYSTEM_RTC_ENABLE
  command_CMD_SET_LOCALTIME,
#else
  command_CMD_ZBX_NOPE,
#endif

  command_CMD_SYS_PORTWRITE,

#ifdef FEATURE_SHIFTOUT_ENABLE 
  command_CMD_SYS_SHIFTOUT,
#else
  command_CMD_ZBX_NOPE,
#endif

  command_CMD_SYS_REBOOT,

#ifdef FEATURE_SYSTEM_RTC_ENABLE
  command_CMD_SYSTEM_LOCALTIME,
#else
  command_CMD_ZBX_NOPE,
#endif

#ifdef FEATURE_REMOTE_COMMANDS_ENABLE
  command_CMD_SYSTEM_RUN,
#else
  command_CMD_ZBX_NOPE,
#endif

#ifdef FEATURE_SYSINFO_ENABLE
  command_CMD_SYSTEM_HW_CHASSIS,
  command_CMD_SYSTEM_HW_CPU,
  command_CMD_NET_PHY_NAME,
  command_CMD_NET_PHY_REINITS,
  command_CMD_SYS_CMD_COUNT,
  command_CMD_SYS_CMD_TIMEMAX,
  command_CMD_SYS_CMD_TIMEMAX_N,
  command_CMD_SYS_RAM_FREE,
  command_CMD_SYS_RAM_FREEMIN,
#else 
  command_CMD_ZBX_NOPE,
  command_CMD_ZBX_NOPE,
  command_CMD_ZBX_NOPE,
  command_CMD_ZBX_NOPE,
  command_CMD_ZBX_NOPE,
  command_CMD_ZBX_NOPE,
  command_CMD_ZBX_NOPE,
  command_CMD_ZBX_NOPE,
  command_CMD_ZBX_NOPE,
#endif
  
  command_CMD_SYS_VCC,
  command_CMD_SYS_VCCMIN,
  command_CMD_SYS_VCCMAX,

#ifdef FEATURE_EXTERNAL_INTERRUPT_ENABLE
  command_CMD_EXTINT_COUNT,
#else
  command_CMD_ZBX_NOPE,
#endif

#ifdef FEATURE_INCREMENTAL_ENCODER_ENABLE
  command_CMD_INCENC_VALUE,
#else
  command_CMD_ZBX_NOPE,
#endif

#ifdef FEATURE_OW_ENABLE
  command_CMD_OW_SCAN,
#else
  command_CMD_ZBX_NOPE,
#endif

#ifdef FEATURE_I2C_ENABLE
  command_CMD_I2C_SCAN,
  command_CMD_I2C_WRITE,
  command_CMD_I2C_READ,
  command_CMD_I2C_BITWRITE,
  command_CMD_I2C_BITREAD,
#else
  command_CMD_ZBX_NOPE,
  command_CMD_ZBX_NOPE,
  command_CMD_ZBX_NOPE,
  command_CMD_ZBX_NOPE,
  command_CMD_ZBX_NOPE,
#endif

#ifdef FEATURE_DS18X20_ENABLE
  command_CMD_DS18X20_TEMPERATURE,
#else
  command_CMD_ZBX_NOPE,
#endif

#ifdef FEATURE_DHT_ENABLE
  command_CMD_DHT_HUMIDITY,
  command_CMD_DHT_TEMPERATURE,
#else
  command_CMD_ZBX_NOPE,
  command_CMD_ZBX_NOPE,
#endif

#ifdef FEATURE_BMP_ENABLE
  command_CMD_BMP_PRESSURE,
  command_CMD_BMP_TEMPERATURE,
#else
  command_CMD_ZBX_NOPE,
  command_CMD_ZBX_NOPE,
#endif

#ifdef SUPPORT_BME280_INCLUDE
  command_CMD_BME_HUMIDITY,
#else
  command_CMD_ZBX_NOPE,
#endif

#ifdef FEATURE_BH1750_ENABLE
  command_CMD_BH1750_LIGHT,
#else
  command_CMD_ZBX_NOPE,
#endif

#ifdef FEATURE_MAX7219_ENABLE
  command_CMD_MAX7219_WRITE,
#else
  command_CMD_ZBX_NOPE,
#endif

#ifdef FEATURE_PCF8574_LCD_ENABLE
  command_CMD_PCF8574_LCDPRINT,
#else
  command_CMD_ZBX_NOPE,
#endif
  
#ifdef FEATURE_SHT2X_ENABLE
  command_CMD_SHT2X_HUMIDITY,
  command_CMD_SHT2X_TEMPERATURE,
#else
  command_CMD_ZBX_NOPE,
  command_CMD_ZBX_NOPE,
#endif
  
#ifdef FEATURE_ACS7XX_ENABLE
  command_CMD_ACS7XX_ZC,
  command_CMD_ACS7XX_AC,
  command_CMD_ACS7XX_DC,
#else
  command_CMD_ZBX_NOPE,
  command_CMD_ZBX_NOPE,
  command_CMD_ZBX_NOPE,
#endif

#ifdef FEATURE_ULTRASONIC_ENABLE
  command_CMD_ULTRASONIC_DISTANCE,
#else
  command_CMD_ZBX_NOPE,
#endif

#ifdef FEATURE_IR_ENABLE
  command_CMD_IR_SEND,
  command_CMD_IR_SENDRAW,
#else
  command_CMD_ZBX_NOPE,
  command_CMD_ZBX_NOPE,
#endif

#ifdef FEATURE_WS2812_ENABLE
  command_CMD_WS2812_SENDRAW,
#else
  command_CMD_ZBX_NOPE,
#endif

#ifdef FEATURE_PZEM004_ENABLE
  command_CMD_PZEM004_CURRENT,
  command_CMD_PZEM004_VOLTAGE,
  command_CMD_PZEM004_POWER,
  command_CMD_PZEM004_ENERGY,
  command_CMD_PZEM004_SETADDR,
#else
  command_CMD_ZBX_NOPE,
  command_CMD_ZBX_NOPE,
  command_CMD_ZBX_NOPE,
  command_CMD_ZBX_NOPE,
  command_CMD_ZBX_NOPE,
#endif

#ifdef FEATURE_UPS_APCSMART_ENABLE
  command_CMD_UPS_APCSMART,
#else
  command_CMD_ZBX_NOPE,
#endif

#ifdef FEATURE_UPS_MEGATEC_ENABLE
  command_CMD_UPS_MEGATEC,
#else
  command_CMD_ZBX_NOPE,
#endif

#ifdef FEATURE_INA219_ENABLE
  command_CMD_INA219_BUSVOLTAGE,
  command_CMD_INA219_CURRENT,
  command_CMD_INA219_POWER,
#else
  command_CMD_ZBX_NOPE,
  command_CMD_ZBX_NOPE,
  command_CMD_ZBX_NOPE,
#endif

#ifdef FEATURE_AT24CXX_ENABLE
  command_CMD_AT24CXX_WRITE,
  command_CMD_AT24CXX_READ,
#else
  command_CMD_ZBX_NOPE,
  command_CMD_ZBX_NOPE,
#endif

#ifdef FEATURE_MAX44009_ENABLE
  command_CMD_MAX44009_LIGHT,
#else
  command_CMD_ZBX_NOPE,
#endif

#ifdef FEATURE_MHZXX_PWM_ENABLE
  command_CMD_MHZXX_PWM_CO2,
#else
  command_CMD_ZBX_NOPE,
#endif
#ifdef FEATURE_MHZXX_UART_ENABLE
  command_CMD_MHZXX_UART_CO2,
#else
  command_CMD_ZBX_NOPE,
#endif

#ifdef FEATURE_USER_FUNCTION_PROCESSING
  command_CMD_USER_RUN,
#else
  command_CMD_ZBX_NOPE,
#endif

#ifdef FEATURE_VEML6070_ENABLE
   command_CMD_VEML6070_UV,
#else
  command_CMD_ZBX_NOPE,
#endif


#ifdef FEATURE_SERVO_ENABLE
   command_CMD_SERVO_TURN,
#else
  command_CMD_ZBX_NOPE,
#endif

};
