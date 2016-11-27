# Zabbuino
An Zabbix agent firmware for Arduino (**AVR only**)

Actual release is v1.0.0. All new features (like APC Smart UPS, PZEM-004T implementation) can be found on [experimental](https://github.com/zbx-sadman/Zabbuino/tree/experimental) branch.

**You can help to the project** by providing hardware to testing and integrating. Or yoг can just donate for it. Contact to me via email: [zbx.sadman@gmail.com](mailto://zbx.sadman@gmail.com)

Implemented:
- A few Zabbix agent commands;
- Wraps a lot of Arduino Language functions;
- OneWire and I2C bus scaning to detect sensors ID or adresses;
- Network DHCP and static IP support;
- Remote configuring & rebooting, system protection;
- Storing system setting in EEPROM;
- AVR WatchDog feature support;
- MCU and runtime metrics (current/min/max VCC, current/min RAM, uptime, MCU name) obtaining;
- Support W5100 or ENC28J60 network modules;
- Support one or more Dallas DS18X20 thermometer;
- Support DHT11/21/22/33/44 or AM2301/2302 humidity and temperature sensors;
- Support Sensirion SHT2X humidity and temperature sensors serie;
- Support BOSCH BMP180/BMP085/BMP280/BME280 pressure, temperature and humidity sensors;
- Support ROHM BH1750 light sensor;
- Support incremental encoder (on interrupt's pin);
- Support devices that can be used with hardware interrupt - tilt switches, dry contacts, water flow sensor, and so;
- Support Allegro ACS7xx sensor;
- Support HC-SR04 ultrasonic ranging module;
- Support any other analog or digital sensor via `analogread` /`digitalread` commands;
- Support indicators, that connected to MAX7219, 8x8 Led matrix for example;
- Support simple I2C devices (expanders, digital linear potentiometers, DAC's etc.);
- Support One- or Two- and Four- lines LCD Character displays with PC8574 I2C expander;
- Support any actuators or indicators via `digitalwrite` command;
- Support WS2801 Led stripe and any indicators on shift registers via extended `shiftout` command;
- Simulate various vendor's IR transmitters.


Minimum requirements: 
- Arduino board with ATMega328 & ENC28J60 / W5100 Ethernet Module.

Manuals:
- In [Russian](https://github.com/zbx-sadman/zabbuino/wiki/Zabbuino-in-Russian);
- Partially translated to [English](https://github.com/zbx-sadman/Zabbuino/wiki/Zabbuino-in-English).

User cases:
- In [Russian](https://github.com/zbx-sadman/Zabbuino/wiki/Zabbuino-User-Cases-in-Russian)

You can also download [templates](https://github.com/zbx-sadman/Zabbuino/tree/master/Zabbix_Templates) for Zabbix 2.4.x
