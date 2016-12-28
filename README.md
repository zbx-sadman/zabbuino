# Zabbuino
An Zabbix agent firmware for Arduino (**AVR only**)

Actual release is v1.1.1. All new features can be found on [experimental](https://github.com/zbx-sadman/Zabbuino/tree/experimental) branch.
Old releases placed in [old_releases](https://github.com/zbx-sadman/Zabbuino/tree/old_releases) branch.

See [change log](https://github.com/zbx-sadman/Zabbuino/blob/master/ChangeLog.md) before update.

**You can help to the project** by providing hardware to testing and integrating. Or yoг can just donate for it. Contact to me via email: [zbx.sadman@gmail.com](mailto://zbx.sadman@gmail.com)

![Zabbuino: example of chart](https://cloud.githubusercontent.com/assets/12827470/20768231/30de116a-b74e-11e6-932f-09eb6f7712e3.png)

Implemented:
- A few Zabbix agent commands;
- Wraps a lot of Arduino Language functions;
- OneWire and I2C bus scaning to detect sensors ID or adresses;
- Network DHCP and static IP support;
- Remote configuring & rebooting, system protection;
- Storing system setting in EEPROM;
- AVR WatchDog feature support;
- MCU and runtime metrics (current/min/max VCC, current/min RAM, uptime, MCU name) obtaining;
- Support W5100 and ENC28J60 network modules;
- Support one or more DS18X20 thermometer;
- Support DHT11/21/22/33/44 or AM2301/2302 humidity and temperature sensors;
- Support SHT2X humidity and temperature sensors serie;
- Support BMP180/085, BMP280/BME280 pressure and temperature sensors;
- Support BH1750 light sensor;
- Support incremental encoder (on interrupt's pin);
- Support any devices that can be used with hardware interrupt - tilt switches, dry contacts, water flow sensor, and so;
- Support ACS7xx sensors;
- Support HC-SR04 ultrasonic ranging module;
- Support any other analog or digital sensor via `analogread` /`digitalread` commands;
- Support indicators, that connected to MAX7219, 8x8 Led matrix for example;
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

Manuals:
- In [Russian](https://github.com/zbx-sadman/zabbuino/wiki/Zabbuino-in-Russian);
- Partially translated to [English](https://github.com/zbx-sadman/Zabbuino/wiki/Zabbuino-in-English).

User cases:
- In [Russian](https://github.com/zbx-sadman/Zabbuino/wiki/Zabbuino-User-Cases-in-Russian)

You can also download [templates](https://github.com/zbx-sadman/Zabbuino/tree/master/Zabbix_Templates) for Zabbix 2.4.x
