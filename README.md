# Zabbuino
An Zabbix agent firmware for Arduino

Actual release is v1.4 (**AVR only**).

Testing release is v1.5 (**AVR & ESP8266 & ESP32** [here](https://github.com/zbx-sadman/zabbuino/tree/esp-integration-test).

Compilation tested on Arduino IDE 1.6.11 (from https://www.arduino.cc/) and above.

**Note: Zabbuino is not any kind of Arduino library. It is a "ready-to-use" project. Rename "zabbuino-master" dir to "zabbuino" just after cloning/downloading/unzipping/etc. to avoid compilation error.**

All new features can be found on [experimental](https://github.com/zbx-sadman/Zabbuino/tree/experimental) branch. Old releases placed in [old_releases](https://github.com/zbx-sadman/Zabbuino/tree/old_releases) branch.


See [change log](https://github.com/zbx-sadman/Zabbuino/blob/master/ChangeLog.md) before update.

**You can help to the project** by providing new ideas and/or hardware to testing and integrating. Or yoг can just donate for further development. Contact to me via [email](mailto://zbx.sadman@gmail.com) or use 
[Yandex.Money](https://money.yandex.ru/to/410014475924637) service.

![Zabbuino: example of chart](https://cloud.githubusercontent.com/assets/12827470/20768231/30de116a-b74e-11e6-932f-09eb6f7712e3.png)

![Zabbuino: example of chart](https://user-images.githubusercontent.com/12827470/53301400-748fa700-3863-11e9-96c4-f8c5ae47c08d.png)

Implemented:
- A few Zabbix agent commands;
- Wraps a lot of Arduino Language functions;
- OneWire and I2C bus scaning to detect sensors ID or adresses;
- Network DHCP and static IP support;
- Remote configuring & rebooting, system protection;
- Storing system setting in EEPROM;
- AVR WatchDog feature support;
- MCU and runtime metrics (current/min/max VCC, current/min RAM, uptime, MCU name) obtaining;
- Support W5100 and ENC28J60 network modules, drivers is implemented to source code;
- Support one or more DS18X20 thermometer;
- Support MLX90614 infrared thermometer;
- Support MAX6675 termocoupler ADC;
- Support DHT11/21/22/33/44 or AM2301/2302 humidity and temperature sensors;
- Support SHT2X humidity and temperature sensors serie;
- Support BMP180/085, BMP280/BME280 pressure and temperature sensors;
- Support BH1750, MAX44009, TSL2561 light sensors;
- Support ADPS9960 light/color sensor;
- Support Telaire T67xx family CO2 sensor;
- Support Winsen MH-Z14/MH-Z19/MH-Z19B, ZE08-CH2O, ZE14-O3 (ZE25-O3, ZE27-O3), ZP14 (ZC05), ZE15-CO, ZE16-CO sensor modules;
- Support VEML6070 ultraviolet sensor;
- Support DS3231 & PCF8563 RTC I2C module;
- Support incremental encoder (on interrupt's pin);
- Support any devices that need to use hardware interrupt - tilt switches, dry contacts, water flow sensor, and so;
- Support INA219 power/current monitor;
- Support HC-SR04 ultrasonic ranging module;
- Support any other analog or digital sensor via `analogread` /`digitalread` commands;
- Support indicators, that connected to MAX7219, 8x8 Led matrix for example;
- Support simple I2C devices (expanders, digital linear potentiometers, etc.);
- Support One- or Two- (and maybe Four-) lines LCD Character displays with PC8574 I2C expander;
- Support any actuators or indicators via `digitalwrite` command;
- Support simple operations with digital servos and relays;
- Support PCA9685 16 channel PWM controller;
- Support WS2801 Led stripe and any indicators on shift registers via extended `shiftout` command;
- Support WS2812 Led stripe;
- Support Plantower PMS-A003 (and similar) dust sensors;
- Support Nova Fitness SDS011 (and similar) dust sensors;
- Support Wuhan Cubic PM2012 (and similar) dust sensors;
- Support DFPlayer Mini;
- Support PZEM-004 energy meter;
- Support APC Smart UPS (with RS232 interface);
- Support Megatec UPS's (with RS232 interface);
- Simulate various vendor's IR transmitters.

Minimum requirements: 
- Arduino board with ATMega328 & ENC28J60 or W5100 Ethernet Module.

Tested with:
- Arduino Mini Pro / Nano / Duemilanove (ATmega328 MCU, 5V), Arduino Micro (ATmega32u4);
- ENC28J60 mini module & ENC28J60 shield for Arduino Nano (5V both), Ethernet Shield (W5100), Mini Red W5100 Ethernet Module (5V), WIZ811MJ Ethernet module (3.3V);
- SW-520D sensors, encoders (EC11), buttons & etc;
- Sensors: DS18B20, AM2302 (DHT22), AM2320, BMP180, BME280, SHT21 (SI7021), BH1750, MAX44009, TSL2561, ADPS9960, MH-Z19B, ZE08-CH2O, ZE15-CO, T6703, VEML6070, INA219,
HC-SR04, Plantower PMS-A003, PM2012, SDS011, MLX90614, MAX6675;
- MAX7219 8x8 LED module & 8-Digit LED Display Module; 
- LCD 2002 (Winstar) & 2004 (Generic) displays with PC8574 I2C convertor;
- PCF8574 I2C expander;
- MCP4725 I2C module;
- PCF8591 I2C module;
- WS2801, WS2812 pixel modules
- PZEM-004 energy meter;
- DFPlayer Mini;
- RS-232 module;
- Smart-UPS 1500, APC Smart-UPS 2200 RM;
- may be i forget someting...

Manuals:
- For v1.4 [In Russian](https://github.com/zbx-sadman/zabbuino/wiki/Zabbuino-in-Russian-(for-release-1.4.x));
- For v1.5 [in Russian](https://github.com/zbx-sadman/zabbuino/wiki/Zabbuino-in-Russian-(for-release-1.5.x));

User cases:
- In [Russian](https://github.com/zbx-sadman/Zabbuino/wiki/Zabbuino-User-Cases-in-Russian)

You can also download [templates](https://github.com/zbx-sadman/Zabbuino/tree/master/Zabbix_Templates) for Zabbix 2.4.x

