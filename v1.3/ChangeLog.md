## Zabbuino change log 

### v1.3.0 (24 Feb 2019)

Added:
- Zabbix v4, ATMega2560 compability;
- Zabbuino pluginization improved;
- MELT LCD displays support;
- Compression for WS2812.sendRaw[] and shiftOut[] commands;
- New sensors, drivers and devices support: Winsen MH-Z19, Melexis MLX90614, Plantower PMS-A003 (and similar), ADPS9960, TSL2561, PCA9685, MAX6675, DFPlayer Mini
- Relays and Servos operations;
- OneWire library to src tree.

Fixed: 
- INA219.* commands wrong work;
- Silly errors in DS18x20 function code;
- SoftwareWire libreary errors (moved to actual release).

### v1.2.0 (18 Apr 2017)

New release with:
- Integrated network drivers for Wiznet & Microchip ENC28J60;
- Network module monitoring function, that re-init module when detect problem;
- Integrated SoftWire lib to allow use I2C devices on most Arduino board pins;
- Support DS3231 realtime clock chip;
- Support AT24CXX external EEPROM to storing user data;
- "Plugins" creating ability to allow make autonomic devices;
- Support MAX44009 ambient light sensor;
- New commands for Zabbix inventarization feature: system.hw.cpu, system.hw.chassis, and etc.;
- Ability to work with multiple PZEM004 devices on a single SoftSerial port;
- Support INA219 Zerø-Drift, Bidirectional Current/Power Monitor.

### v1.1.4 (11 Apr 2017)
Added:
- _PZEM004.setAddr[]_ command.

Changed:
- _PZEM004.*[]_ commands have new optional parameter now: _address_ - point to device to which request addressed.

### v1.1.3 (01 Apr 2017)
Fixed:
- DS18B20 sensor: negative temperature wrong output fixed.

### v1.1.2 (25 Jan 2017)
Fixed:
- Wrong output of discovered OneWire device ID (some zeros not displayed).

Changed:
- Default of pins init state. 

### v1.1.1 (28 Dec 2016)
Fixed:
- DHT11 readings;
- MAX7219 ASCII output on digital led indicator;
- ENC28J60 reinit on "hang" code;


### v1.1.0 (13 Dec 2016)
Features:
- Network procedures improved;
- Commands can be entered via Arduino IDE Serial Monitor;
- Zabbix can handle Zabbuino directly from Actions;
- Debugging verbosity is splitted to LOW/MIDDLE/HIGH levels;
- EEPROM read/write functions try to detect corruption and try to avoid "broken" cells;
- New MCU metrics (ID, Signature) can be obtained;
- Agent's name/MAC/IP can be gnerated using MCU ID;
- Using ASCII for LCD Character display's commands is allowed;
- Using ASCII for MAX7219 based digital indicators is allowed too;
- WS2812 Led stripe support is implemented;
- PZEM-004 energy meter's metrics can be readed via UART;
- Handling of APC Smart UPS (with RS232 interface) added;
- I2C commands improved;

Important changes:
- Source code is refactored.


### v1.0.0 (15 Aug 2016)

Features:
- Scaning OneWire and I2C buses to detect sensors ID or adresses is available;
- Network DHCP protocol support added;
- Remote configuration & control is available;
- System setting stored in EEPROM;
- AVR WatchDog hedging in case of accident;
- MCU and runtime metrics (current/min/max VCC, current/min RAM, uptime, MCU name) can be obtained;
- DHT sensors support is extended for DHT33/44;
- Sensirion SHT2X humidity and temperature sensors serie handling implemented;
- New BOSCH pressure, temperature and humidity sensors is supported: BMP280/BME280 ;
- ROHM BH1750 light sensor is supported too;
- Incremental encoder can be used on interrupt's pin;
- Various contactor-based devices allowed on hardware interrupt pin too - tilt switches, dry contacts, water flow sensor, and so;
- Allegro ACS7xx sensor support implemented;
- HC-SR04 ultrasonic ranging module support added;
- Now can be used indicators, that connected to MAX7219, 8x8 Led matrix for example;
- Simple I2C devices (expanders, digital linear potentiometers, DAC's etc.) now able to use in general (via read/write registry operations);
- One- or Two- and Four- lines LCD Character displays with PC8574 I2C expander can show incoming information;
- WS2801 Led stripe and any indicators on shift registers can be used via extended `shiftout` command;
- Various vendor's IR transmitters emulated.

Important changes:
- Optimization is not performed for ATmega168.
