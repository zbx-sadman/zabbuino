# Zabbuino
An Zabbix agent firmware for Arduino (**AVR only**)

Actual release is v1.2.0. Compilation tested on Arduino IDE 1.6.11 and above.

All new features can be found on [experimental](https://github.com/zbx-sadman/Zabbuino/tree/experimental) branch. Old releases placed in [old_releases](https://github.com/zbx-sadman/Zabbuino/tree/old_releases) branch.

See [change log](https://github.com/zbx-sadman/Zabbuino/blob/master/ChangeLog.md) before update.

**You can help to the project** by providing hardware to testing and integrating. Or yoг can just donate for it. Contact to me via email: [zbx.sadman@gmail.com](mailto://zbx.sadman@gmail.com)

![Zabbuino: example of chart](https://cloud.githubusercontent.com/assets/12827470/20768231/30de116a-b74e-11e6-932f-09eb6f7712e3.png)

Implemented:
- A few Zabbix agent commands;
- Zabbix's Action support;
- Wraps a lot of Arduino Language functions;
- OneWire and I2C bus scaning to detect sensors ID or adresses;
- Network DHCP and static IP support;
- Remote configuring & rebooting, system protection;
- Storing system setting in EEPROM;
- AVR WatchDog feature support;
- MCU and runtime metrics (current/min/max VCC, current/min RAM, uptime, MCU name, MCU ID & signature) obtaining;
- Network module's IP/MAC addresses modification based on MCU ID;
- Testing connected peripherals without network connection via serial-port terminal (Arduino's Serial Monitor);
- Support W5100 and ENC28J60 network modules, drivers is implemented to source code;
- Support DS18X20 thermometers;
- Support DHT11/21/22/33/44 or AM2301/2302 humidity and temperature sensors;
- Support SHT2X humidity and temperature sensors serie;
- Support BMP180/085, BMP280/BME280 pressure and temperature sensors;
- Support BH1750, MAX44009 light sensors;
- Support DS3231 RTC I2C module;
- Support incremental encoder (on interrupt's pin);
- Support any devices that can be used with hardware interrupt - tilt switches, dry contacts, water flow sensor, and so;
- Support INA219 power/current monitor;
- Support ACS7xx sensors;
- Support HC-SR04 ultrasonic ranging module;
- Support any other analog or digital sensor via `analogread` /`digitalread` commands;
- Support indicators, that connected to MAX7219, 8x8 Led matrix, 7-segment numeric LED display for example;
- Support simple I2C devices (expanders, digital linear potentiometers, etc.);
- Support One- or Two- (and maybe Four-) lines LCD Character displays with PC8574 I2C expander;
- Support any actuators or indicators via `digitalwrite` command;
- Support WS2801 Led stripe and any indicators on shift registers via extended `shiftout` command;
- Support WS2812 Led stripe;
- Support PZEM-004 energy meter;
- Support APC Smart UPS (with RS232 interface);
- Simulate varuious vendor's IR transmitters.

Minimum requirements: 
- Arduino board with ATMega328 & ENC28J60 or W5100 Ethernet Module.

Tested with:
- Arduino Mini Pro / Nano / Duemilanove (ATmega328 MCU, 5V), Arduino Micro (ATmega32u4);
- ENC28J60 mini module & ENC28J60 shield for Arduino Nano (5V both), Ethernet Shield (W5100), Mini Red W5100 Ethernet Module (5V), WIZ811MJ Ethernet module (3.3V);
- SW-520D sensors, encoders (EC11), buttons & etc;
- Bunch of Dallas DS18B20 thermometers;
- AOSONG AM2302 (DHT22) sensor;
- BOSCH BMP180, BME280 sensors;
- SHT21 (SI7021) sensor;
- ROHM BH1750 sensor;
- MAX44009 sensor;
- INA219 power/current monitor;
- ACS712 sensor;
- HC-SR04 sensor;
- MAX7219 8x8 LED module & 8-Digit LED Display Module;
- LCD 2002 (Winstar) & 2004 (Generic) displays with PC8574 I2C convertor;
- PCF8574 I2C expander;
- MCP4725 I2C module;
- PCF8591 I2C module;
- WS2801, WS2812 pixel modules
- PZEM-004 energy meter;
- RS-232 module;
- Smart-UPS 1500, APC Smart-UPS 2200 RM;
- may be i forget someting...

Manuals:
- In [Russian](https://github.com/zbx-sadman/Zabbuino/wiki/Zabbuino-in-Russian-(for-release-1.1.x));

User cases:
- In [Russian](https://github.com/zbx-sadman/Zabbuino/wiki/Zabbuino-User-Cases-in-Russian)

You can also download [templates](https://github.com/zbx-sadman/Zabbuino/tree/master/Zabbix_Templates) for Zabbix 2.4.x

