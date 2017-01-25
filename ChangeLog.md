##Zabbuino change log 

###v1.1.2 (25 Jan 2017)
Fixed:
- Wrong output of discovered OneWire device ID (some zeros not displayed).

###v1.1.1 (28 Dec 2016)
Fixed:
- DHT11 readings;
- MAX7219 ASCII output on digital led indicator;
- ENC28J60 reinit on "hang" code;


###v1.1.0 (13 Dec 2016)
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


###v1.0.0 (15 Aug 2016)

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
