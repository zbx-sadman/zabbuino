#pragma once
#include "sys_structs.h"

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

#define CMD_MAX6675_TEMPERATURE                                 (0x50)
#define CMD_PCA9685_WRITE                                       (0x51)

#define CMD_RELAY                                               (0x52)
#define CMD_PULSE                                               (0x53)

#define CMD_SERVO_TURN                                          (0x54)

#define CMD_TSL2561_LIGHT                                       (0x55)

#define CMD_DFPLAYER_RUN                                        (0x56)

#define CMD_ADPS9960_AMBIENT                                    (0x57)
#define CMD_ADPS9960_RED                                        (0x58)
#define CMD_ADPS9960_GREEN                                      (0x59)
#define CMD_ADPS9960_BLUE                                       (0x5A)

#define CMD_PLANTOWER_PMS_ALL                                   (0x5B)
#define CMD_PLANTOWER_PMS_EPM25                                 (0x5C)

#define CMD_MLX90614_TEMPERATURE                                (0x5D)

#define CMD_SGP30_CO2E                                          (0x5E)
#define CMD_SGP30_TVOC                                          (0x5F)

// add new command as "const char command_<COMMAND_MACRO> PROGMEM". Only 'const' push string to PROGMEM.
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

const char command_CMD_MAX6675_TEMPERATURE[]                    PROGMEM = "max6675.temperature";
const char command_CMD_PCA9685_WRITE[]                          PROGMEM = "pca9685.write";

const char command_CMD_RELAY[]                                  PROGMEM = "relay";
const char command_CMD_PULSE[]                                  PROGMEM = "pulse";
const char command_CMD_SERVO_TURN[]                             PROGMEM = "servo.turn";

const char command_CMD_TSL2561_LIGHT[]                          PROGMEM = "tsl2561.light";

const char command_CMD_DFPLAYER_RUN[]                           PROGMEM = "dfplayer.run";

const char command_CMD_ADPS9960_AMBIENT[]                       PROGMEM = "adps9960.ambient";
const char command_CMD_ADPS9960_RED[]                           PROGMEM = "adps9960.red";
const char command_CMD_ADPS9960_GREEN[]                         PROGMEM = "adps9960.green";
const char command_CMD_ADPS9960_BLUE[]                          PROGMEM = "adps9960.blue";

const char command_CMD_PLANTOWER_PMS_ALL[]                      PROGMEM = "pms.all";
const char command_CMD_PLANTOWER_PMS_EPM25[]                    PROGMEM = "pms.epm25";

const char command_CMD_MLX90614_TEMPERATURE[]                   PROGMEM = "mlx90614.temperature";

const char command_CMD_SGP30_CO2E[]                             PROGMEM = "sgp30.co2e";
const char command_CMD_SGP30_TVOC[]                             PROGMEM = "sgp30.tvoc";

//
const command_t PROGMEM commands[] = {
    { CMD_ZBX_NOPE                , command_CMD_ZBX_NOPE},               
    { CMD_ZBX_AGENT_PING          , command_CMD_ZBX_AGENT_PING},         
    { CMD_ZBX_AGENT_HOSTNAME      , command_CMD_ZBX_AGENT_HOSTNAME},     
    { CMD_ZBX_AGENT_VERSION       , command_CMD_ZBX_AGENT_VERSION},      
    { CMD_SYSTEM_UPTIME           , command_CMD_SYSTEM_UPTIME},                                             

#ifdef FEATURE_ARDUINO_BASIC_ENABLE
    { CMD_ARDUINO_ANALOGWRITE     , command_CMD_ARDUINO_ANALOGWRITE},    
    { CMD_ARDUINO_ANALOGREAD      , command_CMD_ARDUINO_ANALOGREAD},     
#endif

#ifdef FEATURE_AREF_ENABLE
    { CMD_ARDUINO_ANALOGREFERENCE , command_CMD_ARDUINO_ANALOGREFERENCE},
#endif

#ifdef FEATURE_ARDUINO_BASIC_ENABLE
    { CMD_ARDUINO_DELAY           , command_CMD_ARDUINO_DELAY},          
    { CMD_ARDUINO_DIGITALWRITE    , command_CMD_ARDUINO_DIGITALWRITE},   
    { CMD_ARDUINO_DIGITALREAD     , command_CMD_ARDUINO_DIGITALREAD},                                        
#endif

#ifdef FEATURE_TONE_ENABLE
    { CMD_ARDUINO_TONE            , command_CMD_ARDUINO_TONE},           
    { CMD_ARDUINO_NOTONE          , command_CMD_ARDUINO_NOTONE},                                              
#endif

#ifdef FEATURE_RANDOM_ENABLE
    { CMD_ARDUINO_RANDOMSEED      , command_CMD_ARDUINO_RANDOMSEED},     
    { CMD_ARDUINO_RANDOM          , command_CMD_ARDUINO_RANDOM},         
#endif

#ifdef FEATURE_EEPROM_ENABLE
    { CMD_SET_HOSTNAME            , command_CMD_SET_HOSTNAME},           
    { CMD_SET_NETWORK             , command_CMD_SET_NETWORK},            
    { CMD_SET_PASSWORD            , command_CMD_SET_PASSWORD},           
    { CMD_SET_SYSPROTECT          , command_CMD_SET_SYSPROTECT},         
#endif

#ifdef FEATURE_SYSTEM_RTC_ENABLE
    { CMD_SET_LOCALTIME           , command_CMD_SET_LOCALTIME},          
    { CMD_SYSTEM_LOCALTIME        , command_CMD_SYSTEM_LOCALTIME},       
#endif

    { CMD_SYS_PORTWRITE           , command_CMD_SYS_PORTWRITE},          

#ifdef FEATURE_SHIFTOUT_ENABLE 
    { CMD_SYS_SHIFTOUT            , command_CMD_SYS_SHIFTOUT},           
    { CMD_SYS_REBOOT              , command_CMD_SYS_REBOOT},             
#endif

#ifdef FEATURE_REMOTE_COMMANDS_ENABLE
    { CMD_SYSTEM_RUN              , command_CMD_SYSTEM_RUN},             
#endif

#ifdef FEATURE_SYSINFO_ENABLE
    { CMD_SYSTEM_HW_CHASSIS       , command_CMD_SYSTEM_HW_CHASSIS},      
    { CMD_SYSTEM_HW_CPU           , command_CMD_SYSTEM_HW_CPU},          
    { CMD_NET_PHY_NAME            , command_CMD_NET_PHY_NAME},           
    { CMD_NET_PHY_REINITS         , command_CMD_NET_PHY_REINITS},        
    { CMD_SYS_CMD_COUNT           , command_CMD_SYS_CMD_COUNT},          
    { CMD_SYS_CMD_TIMEMAX         , command_CMD_SYS_CMD_TIMEMAX},        
    { CMD_SYS_CMD_TIMEMAX_N       , command_CMD_SYS_CMD_TIMEMAX_N},      
    { CMD_SYS_RAM_FREE            , command_CMD_SYS_RAM_FREE},           
    { CMD_SYS_RAM_FREEMIN         , command_CMD_SYS_RAM_FREEMIN},        
#endif

    { CMD_SYS_VCC                 , command_CMD_SYS_VCC},                
    { CMD_SYS_VCCMIN              , command_CMD_SYS_VCCMIN},             
    { CMD_SYS_VCCMAX              , command_CMD_SYS_VCCMAX},             

#ifdef FEATURE_EXTERNAL_INTERRUPT_ENABLE
    { CMD_EXTINT_COUNT            , command_CMD_EXTINT_COUNT},           
#endif

#ifdef FEATURE_INCREMENTAL_ENCODER_ENABLE
    { CMD_INCENC_VALUE            , command_CMD_INCENC_VALUE},           
#endif
 
#ifdef FEATURE_OW_ENABLE
    { CMD_OW_SCAN                 , command_CMD_OW_SCAN},                
#endif

#ifdef FEATURE_I2C_ENABLE
    { CMD_I2C_SCAN                , command_CMD_I2C_SCAN},               
    { CMD_I2C_WRITE               , command_CMD_I2C_WRITE},              
    { CMD_I2C_READ                , command_CMD_I2C_READ},               
    { CMD_I2C_BITWRITE            , command_CMD_I2C_BITWRITE},           
    { CMD_I2C_BITREAD             , command_CMD_I2C_BITREAD},            
#endif

#ifdef FEATURE_DS18X20_ENABLE
    { CMD_DS18X20_TEMPERATURE     , command_CMD_DS18X20_TEMPERATURE},    
#endif

#ifdef FEATURE_DHT_ENABLE
    { CMD_DHT_HUMIDITY            , command_CMD_DHT_HUMIDITY},           
    { CMD_DHT_TEMPERATURE         , command_CMD_DHT_TEMPERATURE},        
#endif

#ifdef FEATURE_BMP_ENABLE
    { CMD_BMP_PRESSURE            , command_CMD_BMP_PRESSURE},           
    { CMD_BMP_TEMPERATURE         , command_CMD_BMP_TEMPERATURE},        
#endif

#ifdef SUPPORT_BME280_INCLUDE
    { CMD_BME_HUMIDITY            , command_CMD_BME_HUMIDITY},           
#endif

#ifdef FEATURE_BH1750_ENABLE
    { CMD_BH1750_LIGHT            , command_CMD_BH1750_LIGHT},           
#endif

#ifdef FEATURE_MAX7219_ENABLE
    { CMD_MAX7219_WRITE           , command_CMD_MAX7219_WRITE},          
#endif

#ifdef FEATURE_PCF8574_LCD_ENABLE
    { CMD_PCF8574_LCDPRINT        , command_CMD_PCF8574_LCDPRINT},       
#endif

#ifdef FEATURE_SHT2X_ENABLE
    { CMD_SHT2X_HUMIDITY          , command_CMD_SHT2X_HUMIDITY},         
    { CMD_SHT2X_TEMPERATURE       , command_CMD_SHT2X_TEMPERATURE},      
#endif

#ifdef FEATURE_ACS7XX_ENABLE
    { CMD_ACS7XX_ZC               , command_CMD_ACS7XX_ZC},              
    { CMD_ACS7XX_AC               , command_CMD_ACS7XX_AC},              
    { CMD_ACS7XX_DC               , command_CMD_ACS7XX_DC},              
#endif

#ifdef FEATURE_ULTRASONIC_ENABLE
    { CMD_ULTRASONIC_DISTANCE     , command_CMD_ULTRASONIC_DISTANCE},    
#endif
    
#ifdef FEATURE_IR_ENABLE
    { CMD_IR_SEND                 , command_CMD_IR_SEND},                
    { CMD_IR_SENDRAW              , command_CMD_IR_SENDRAW},             
#endif

#ifdef FEATURE_WS2812_ENABLE
    { CMD_WS2812_SENDRAW          , command_CMD_WS2812_SENDRAW},         
#endif

#ifdef FEATURE_PZEM004_ENABLE
    { CMD_PZEM004_CURRENT         , command_CMD_PZEM004_CURRENT},        
    { CMD_PZEM004_VOLTAGE         , command_CMD_PZEM004_VOLTAGE},        
    { CMD_PZEM004_POWER           , command_CMD_PZEM004_POWER},          
    { CMD_PZEM004_ENERGY          , command_CMD_PZEM004_ENERGY},         
    { CMD_PZEM004_SETADDR         , command_CMD_PZEM004_SETADDR},        
#endif

#ifdef FEATURE_UPS_APCSMART_ENABLE
    { CMD_UPS_APCSMART            , command_CMD_UPS_APCSMART},           
#endif

#ifdef FEATURE_UPS_MEGATEC_ENABLE
    { CMD_UPS_MEGATEC             , command_CMD_UPS_MEGATEC},            
#endif

#ifdef FEATURE_INA219_ENABLE
    { CMD_INA219_BUSVOLTAGE       , command_CMD_INA219_BUSVOLTAGE},      
    { CMD_INA219_CURRENT          , command_CMD_INA219_CURRENT},         
    { CMD_INA219_POWER            , command_CMD_INA219_POWER},           
#endif

#ifdef FEATURE_AT24CXX_ENABLE
    { CMD_AT24CXX_WRITE           , command_CMD_AT24CXX_WRITE},          
    { CMD_AT24CXX_READ            , command_CMD_AT24CXX_READ},           
#endif
    
#ifdef FEATURE_MAX44009_ENABLE
    { CMD_MAX44009_LIGHT          , command_CMD_MAX44009_LIGHT},         
#endif

#ifdef FEATURE_MHZXX_PWM_ENABLE
    { CMD_MHZXX_PWM_CO2           , command_CMD_MHZXX_PWM_CO2},          
    { CMD_MHZXX_UART_CO2          , command_CMD_MHZXX_UART_CO2},         
#endif

#ifdef FEATURE_USER_FUNCTION_PROCESSING
    { CMD_USER_RUN                , command_CMD_USER_RUN},               
#endif

#ifdef FEATURE_VEML6070_ENABLE
    { CMD_VEML6070_UV             , command_CMD_VEML6070_UV},            
#endif

#ifdef FEATURE_MAX6675_ENABLE
    { CMD_MAX6675_TEMPERATURE     , command_CMD_MAX6675_TEMPERATURE},    
#endif

#ifdef FEATURE_PCA9685_ENABLE
    { CMD_PCA9685_WRITE           , command_CMD_PCA9685_WRITE},          
#endif

#ifdef FEATURE_RELAY_ENABLE
    { CMD_RELAY                   , command_CMD_RELAY},                  
    { CMD_PULSE                   , command_CMD_PULSE},                  
#endif

#ifdef FEATURE_SERVO_ENABLE
    { CMD_SERVO_TURN              , command_CMD_SERVO_TURN},             
#endif

#ifdef FEATURE_TSL2561_ENABLE
    { CMD_TSL2561_LIGHT           , command_CMD_TSL2561_LIGHT},          
#endif

#ifdef FEATURE_DFPLAYER_ENABLE
    { CMD_DFPLAYER_RUN            , command_CMD_DFPLAYER_RUN},
#endif

#ifdef FEATURE_ADPS9960_ENABLE
    { CMD_ADPS9960_AMBIENT        , command_CMD_ADPS9960_AMBIENT},
    { CMD_ADPS9960_RED            , command_CMD_ADPS9960_RED},
    { CMD_ADPS9960_GREEN          , command_CMD_ADPS9960_GREEN},
    { CMD_ADPS9960_BLUE           , command_CMD_ADPS9960_BLUE},
#endif

#ifdef FEATURE_PLANTOWER_PMS_SEPARATE_ENABLE
    { CMD_PLANTOWER_PMS_EPM25     , command_CMD_PLANTOWER_PMS_EPM25},
#endif

#ifdef FEATURE_PLANTOWER_PMS_ALL_ENABLE
    { CMD_PLANTOWER_PMS_ALL       , command_CMD_PLANTOWER_PMS_ALL},
#endif

#ifdef FEATURE_MLX90614_ENABLE
    { CMD_MLX90614_TEMPERATURE    , command_CMD_MLX90614_TEMPERATURE},
#endif

#ifdef FEATURE_SGP30_ENABLE
    { CMD_SGP30_CO2E    , command_CMD_SGP30_CO2E},
    { CMD_SGP30_TVOC    , command_CMD_SGP30_TVOC},
#endif
};
