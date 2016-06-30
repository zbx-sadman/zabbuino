# Zabbuino
An Zabbix agent firmware for Arduino

Try Zabbuino 1.0.0 [pre-release](https://github.com/zbx-sadman/Zabbuino/tree/master/v1.0.0_pre-release)

Implemented:
- A few Zabbix agent commands;
- Wraps a lot of Arduino Language functions.
- Support one or more DS18B20 Thermometer;
- Support DHTxx humidity/temperature sensor serie;
- Support BMP085/180 pressure/temperature sensor serie.
- Support any other analog or digital sensor via `analogread` /`digitalread` commands .

Minimum requirements: 
- W5100 Ethernet Module with ATMega 168 and upward;
- ENC28J60 Ethernet Module with ATMega 328 and upward

Manuals:
- Draft in [Russian](https://github.com/zbx-sadman/zabbuino/wiki/Zabbuino-in-Russian);
- Partially translated to [English](https://github.com/zbx-sadman/Zabbuino/wiki/Zabbuino-in-English).

