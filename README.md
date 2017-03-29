# Zabbuino 1.2.0

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
- Support DHT11/21/22/33/44 or AM2301/2302 humidity and temperature sensors;
- Support SHT2X humidity and temperature sensors serie;
- Support BMP180/085, BMP280/BME280 pressure and temperature sensors;
- Support BH1750, MAX44009 light sensors;
- Support DS3231 RTC I2C module;
- Support incremental encoder (on interrupt's pin);
- Support any devices that need to use hardware interrupt - tilt switches, dry contacts, water flow sensor, and so;
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


####29 March 2016

Changes: 

New feature:
 - _FEATURE\_MAX44009\_ENABLE_ - enable support of I2C connected MAX44009 sensor and allow to use command:
   - MAX44009.light[sdaPin, sclPin, i2cAddress, mode, integrationTime] - get Ambient light value from MAX44009 sensor. _mode_ is mode of reading (0x80 - continuous, and 0x00 - once on request); _integrationTime_ is optional parameter which set time of data collecting by sensor, all alowed values can be found on page #9 of datasheet. If _integrationTime_ is skipped - sensor will be switched to auto-measurement mode. Example: _max44009.light[18,19,0x4A,0x00]_ - make one reading in "auto" mode, _max44009.light[18,19,0x4A,0x80,0x03]_ - switch sensor to continuous measurement mode with 100ms interval and take current light value;

Note#1: You can get wrong light level, if set unsuitable _integrationTime_ value.
Note#2: 800ms delay used to avoid returns wrong light level on _mode_ / _integrationTime_ change. 
Note#3: Automatic measurement always run for 800ms, because i see no way at this time to detect finish of operation;



####13 March 2016

Changes: 
 - I2C-related subroutines now used modified SoftwareWire lib (integrated to sources). I2C Sensor can be connected to various MCU pins (not only A4/A5 for ATmega328);
 - Command execution procedure was optimized for add speed a little;
 - _FEATURE\_BMP\_ENABLE_ + _SUPPORT\_BMP180\_INCLUDE_ (or _SUPPORT\_BMP280\_INCLUDE_, _SUPPORT\_BME280\_INCLUDE_) replaced by _FEATURE\_BMP180\_ENABLE_, _FEATURE\_BMP280\_ENABLE_, and _FEATURE\_BME280\_ENABLE_ to make configuration file more simply.
 - _FEATURE\_SYSTEM\_DISPLAY\_ENABLE_ renamed to _FEATURE\_USER\_DISPLAY\_ENABLE_ because some users want to see sensors metric values & various messages. All related constants moved to _plugin.ino_;

New feature:
 - _FEATURE\_AT24CXX\_ENABLE_ allow to use commands:
   - AT24CXX.write[sdaPin, sclPin, i2cAddress, cellAddress, data] - to write user _data_ into AT24CXX EEPROM starting from _cellAddress_ . _Data_ must be HEX-string (0xABCDEF); 
   - AT24CXX.read[sdaPin, sclPin, i2cAddress, cellAddress, length] - to read _length_ bytes from AT24CXX EEPROM starting from _cellAddress_ . Result is HEX-string (0xABCDEF); 
 - Now users can write its own data from EEPROM to using it inside _plugin.ino_ subroutines. For example it allow to make autoswitched light sensor with remotely setted bounds.


####10 March 2016

Changes: 
 - RTC timezone now saved on MCU EEPROM, because small DS3231 modules does not have onboard AT24C32 and timezony only read/write is expensive operation (300bytes of flash space for two byte handling).
   \_SYSTEM\_RTC\_ONBOARD\_EEPROM\_ENABLE_ is not used and _FEATURE\_EEPROM\_ENABLE_ must be used to allow remotely timezone set.

####03 March 2016

Changes: 
 - _set.localtime[unixTimestamp, timeZoneOffset]_ have new options timeZoneOffset (in sec) to be able use TZ inside Zabbuino code. timeZoneOffset can be used with AT24C32 EEPROM;

New features:
 - _FEATURE\_SYSTEM\_RTC\_ONBOARD\_EEPROM\_ENABLE_ - allow to save timezone into RTC module's onboard AT24C32 EEPROM;

####02 March 2016

Changes: 
 - _sys.uptime_ renamed to _system.uptime_ to get more compability with Zabbix;
 - _sys.mcu.name_, sys.mcu.id_, sys.mcu.sign_ now called:  _system.hw.cpu_, _system.hw.cpu[id]_ ,  and _system.hw.cpu[model]_ ;
 - _sys.net.reinits_ renamed to _net.phy.reinits_ , and _sys.phy.module_ renamed to _net.phy.name_ ;
 - _FEATURE\_REPORT\_SCREEN\_ENABLE_ renamed to _FEATURE\_SYSTEM\_DISPLAY\_ENABLE_ ;
 - _FEATURE\_DEBUG\_COMMANDS\_ENABLE_ renamed to _FEATURE\_SYSINFO\_ENABLE_ ;
 - BH1750 subroutine refactored but not tested yet;

New features:
 - _FEATURE\_SYSTEM\_RTC\_ENABLE_ - enable system RTC (I2C DS3231 module supported only);
 - _FEATURE\_REMOTE\_COMMANDS\_ENABLE_  works like Zabbix's EnableRemoteCommands directive and enable _system.run[]_ command;

New commands:
 - _system.hw.chassis_ - returns board/platform name;
 - _set.localtime[unixTimestamp]_ - set time on system RTC if it used;
 - _system.localtime_ - returns time as unixTimestamp from system RTC if it used;

####26 Feb 2016

New feature:
 - With _FEATURE\_REPORT\_SCREEN\_ENABLE_ you can build your own virtual report screen and send it to I2C connected LCD screen. Refer to "SYSTEM HARDWARE SECTION" in _tune.h_ to get more info about LCD connection settings. _reportToScreen()_ function source code can be found in _plugin.ino_
 
####31 Jan 2016

Changes: 
 - Network drivers (UIPEthernet and Wiznet libraries) now integrated to source code. It works but not tested so much;
 - _USE_DIRTY_HACK_AND_REBOOT_ENC28J60_IF_ITS_SEEMS_FREEZE_ renamed to _FEATURE_NETWORK_MONITORING_ ;
 - if _FEATURE_NET_USE_MCUID_ enabled - 3 last byte of MCU ID's will used for 4, 5, 6 octets of MAC address, and last ID's byte used for 4-th octet of IP address;
 - Refactored Dallas temperature sensors subroutines, device ID validation and tesing for presence on the bus is added. 

####29 Jan 2016

Going to Zabbuino 1.2.0.

Fixes:
 - DHT11 really works with _dht.\*_ commands (tested on real hardware);
 - _ow.scan[]_ command wrong address print (some zeros was lost).

New commands:
 - _INA219.BusVoltage[sdaPin, sclPin, i2cAddress, maxVoltage, maxCurrent]_ - command returns value of "bus voltage" metric (in mV), obtained from INA219 Current/Power Monitor, connected to I2C bus: 
  - _sdaPin, sclPin_ - I2C pins;
  - _i2cAddress_ - address of I2C device. It can be found with _I2C.scan[]_ command;
  - _maxVoltage_ - Bus Voltage Range - 16, 32 (V, optional, default is 32);
  - _maxCurrent_ - Maximum expected current - 1000, 2000, 3000  (mA, optional, default is 3000);
 - _INA219.Current [sdaPin, sclPin, i2cAddress, maxVoltage, maxCurrent]_ - command returns value of "current" metric (in mA);
 - _INA219.Power [sdaPin, sclPin, i2cAddress, maxVoltage, maxCurrent]_ - command returns value of "power" metric (in mW).
Example: `zabbix_get -s 192.168.0.1 -k "ina219.current[18,19,0x40,16,1000]"` - get value of current on electrical circuit section, that used 12V power supply and consume no more 1A.

Note: currently you can use 1000, 2000, 3000 (1A, 2A, 3A) as _maxCurrent_. Overflow bit not controlled at this time.

####08 Dec 2016

Changes: 
  - Zabbuino configs was splitted to _basic.h_ (a little basic setting set) and _src/tune.h_ (tuning settings);
  - Compilation is improved (i think so);
  - Network processing reworked a little again.

####28 Nov 2016

Fixes:
  - PZEM-004's commands works again (was broken a little on code refactoring).

Changes: 
  - Debug level is available in levels: low, mid, high (FEATURE_DEBUG_TO_SERIAL_LOW, FEATURE_DEBUG_TO_SERIAL_MIDDLE, FEATURE_DEBUG_TO_SERIAL_HIGH). Need to choose only one feature;
  - Due fight for ENC28J60 stability UIPEthernet source code is changed. [Library archive](https://github.com/zbx-sadman/Zabbuino/tree/experimental/libraries) is uploaded to github. Testing continues...
  - Free memory size measured on end of some subroutines. Perhaps this is to be more reliable. We'll see.


####10 Nov 2016

Fixes:
  - Wrong MCU Signature reading.

Changes: 
 - Subroutine that save config to EEPROM now search non-damaged area if corrupted cell is detected.
 

####07 Nov 2016

Refactoring is processed...

Changes: 
 - _encInc.count[]_ renamed to _encInc.value[]_ due its decremented and incremented, not only count.
 - _encInc.value[]_ and _extInt.count[]_ was shrinked a lot - _intNumber_ arg is removed because interrupt number changing have no sense at this time. 
 - Internal voltage metrics now gather outside timer's interrupt. It's so slow to using in interrupt subroutine and, probaly, give no profit for measuring.
 
New feature:
 - _FEATURE_NET_DEBUG_TO_SERIAL_ allow to see the additional debug messages on the Serial Monitor when network errors probaly occurs. May be it help to resolve some cases of the ENC28J60 module freezing.


####22 Sep 2016

Changes:
 - _ups.apcsmart[]_ can send to APC Smart UPS _^N_ & _Z_ (Turn on UPS, Shutdown UPS) commands now.

New command:
 - _system.run[newCommand]_ command say to Zabbuino to run _newCommand_. It's allow to use Zabbuino's functionality directly from Zabbix's _Actions_. Just fill form as follows:
  * Operation type: Remote command;
  * Target list, Target: Current host;
  * Type: Custom script;
  * Execute on: Zabbix agent;
  * Commands: any simple Zabbuino command, _ups.apcsmart[2,3,0x5A]_ for example to turn off APC Smart UPS, which controlled to Zabbuino.


####21 Sep 2016

Changes:
 - _ups.apc.smart[]_ command renamed to _ups.apcsmart[]_. Just for name ordering;
 - New macro - _FEATURE\_SERIAL\_LISTEN\_TOO_ . Entering the commands in Serial Monitor as Zabbix's keys available now, and you can check your sensors output without network connection.

New commands:
 - _ups.megatec[rxPin, txPin, command, fieldNumber]_ command return specified field data from Megatec-compatible UPS answer to _command_. 
   - _rxPin_ SoftSerial's RX pin to which UPS's (over RS232 convertor) TX connected;
   - _rxPin_ SoftSerial's TX pin to which UPS's (over RS232 convertor) RX connected;
   - _command_ Megatec UPS command, that can be found on [Megatec Protocol information](http://networkupstools.org/protocols/megatec.html). _Command_ can be specified as ASCII string like "Q1", or HEX-string like 0x49 (I-command). 
   - _fieldNumber_ the number of data field that must be found and returned. With _fieldNumber_ == 0 returns full answer string. If the field number is greater than the available, then use the data from the last field. Example: `zabbix_get -s 192.168.0.1 -k ups.megatec[2,3,"Q1", 6]` - get 6-th field from the UPS answer on "Status Inquiry" command.

**Note #1** UPS must be connected to Arduino with RS232-TTL convertor (MAX232 IC). 
**Note #2** _ups.megatec[]_ haven't tested on real hardware, user reports require.


####19 Sep 2016

Fixes:
  - _PZEM004.*_ memory leak caused the system hang.

New commands:
 - _ups.apc.smart[rxPin, txPin, command]_ command return APC Smart UPS answer to _command_. 
   - _rxPin_ SoftSerial's RX pin to which UPS's (over RS232 convertor) TX connected;
   - _rxPin_ SoftSerial's TX pin to which UPS's (over RS232 convertor) RX connected;
   - _command_ Smart UPS command, that can be found on [APC’s smart protocol page](http://networkupstools.org/protocols/apcsmart.html). _Command_ can be specified as ASCII string like "c", or HEX-string like 0x01 (^A - command). Example: `zabbix_get -s 192.168.0.1 -k ups.apcsmart[2,3,"n"]` - get UPS's serial number.

**Note #1** UPS must be connected to Arduino with RS232-TTL convertor (MAX232 IC). 
**Note #2** APC Smart UPS have non-standart pinout, use APC-branded cables.


####14 Sep 2016

Changes:
  - _I2C.Read[]_ command have new optional parameter - _doDoubleReading_ . Some sensors/IC's (like PCF8591) returns the value of previous conversion and need to reply reading to get actual data. Now Zabbuino can ask for result twice if you specify _doDoubleReading_ parameter greater that 0. For example, to get _actual_ 1-byte data from ADC3 (AIN3 pin) of PCF8591: `I2C.Read[18,19,0x48,0x03,1,1]`.


**Note** Zabbuino _I2C.Read[] /I2C.Write[]_ commands tested with PCF8591 module (YL-40). Reading & writing is sucessfull. For example, set voltage on PCF8591 AOUT pin to ~0.75\*VDD Volts ( 255\*0.75 => ~191 ): `zabbix_get -s 192.168.0.1 -k I2C.Write[18,19,0x48,0x40,191]`.

####13 Sep 2016
Changes:
  - _I2C.Write[]_ command have new optional parameter. It's a _numBytes_ - how much bytes must be written to the register of I2C device. Example: _I2C.Write[18,19,0x62,0x40,2048,2]_ - send value 2048 in two byte to register 0x40 of MCP4725 DAC IC. If no _numBytes_ specified - _data_ cast to uint8_t and send in one byte;
  - _PZEM004.*[]_ commands have new optional parameter: _ip_ . It's used only for authorization on energymeters with default ip other than 192.168.1.1 and not replaced PZEM's ip. Example: _PZEM004.voltage[5, 6, 0xC0A80001]_ - take voltage from PZEM004 that have 192.168.0.1 address;
  - _PCF8574.LCDPrint[]_ allow to use ASCII-string as _data_ parameter. _data_ must be doublequoted, internal doublequote must be escaped with '\'. Supported control symbols '\t' - horizontal tabulation, '\n' - new line, and '\xHH' for using other symbols with HEX representation. 
Example: `zabbix_get -s 192.168.0.1 -k 'PCF8574.LCDPrint[18,19,0x20,1,2002,"\x06\x01\x0FHello\n\tJust \"testing\""]'`
    - _\x06_ - "Left to right" entry mode enable;
    - _\x01_ - Clear display;
    - _\x0F_ - Blinking block cursor on;
    - _Hello_ - just the word;
    - _\n\t_ - go to new line and make 'Tab';
    - _Just \"testing\"_ - the phrase with doublequotes.
  - _MAX7219.Write[]_ allow to use ASCII-string as _data_ parameter too. Its can be used on "MAX7219 8-Digit LED Display Module" from Aliexpress. Only next chars can be displayed: '0'..'9', '-', 'c', 'C', 'h', 'H', 'E', 'L', 'P', 'n', 'o', 'r', dot and space. Example: 
`zabbix_get -s 192.168.0.1 -k MAX7219.write[4,5,6,5,"Co2 4851"]`

**Note #1** Many LCD's have right codepage only for english chars. All national sythmbols images not mapped to the ASCII-table. You can use \xHH for its.
**Note #2** MAX7219's ASCII-value feature haven't tested on real hardware.
**Note #3** _I2C.Write[]_ have tested with PCF8574, and MCP4725 modules. All seems OK. Value of the "voltage" for MCP4725 (0...4095) must be multiplied by 16, because it's pushed in two bytes with left padding: D12.D11.D10.D09,D08,D07.D06.D05   D04.D03.D02.D01.X.X.X.X (X.X.X.X - unused bits).


####08 Sep 2016
Changes:
  - New macro - _FEATURE\_NET\_USE\_MCUID_ . If its defined - ATMega's ID is used as Zabbuino's hostname, and the last byte of ATMega's ID is replace the last byte of default MAC/IP address.

New commands:
 - _pzem004.voltage[rxPin, txPin]_ command return voltage value that obtained from Peacefair PZEM-004 energy meter via TTL port;
   - _rxPin_ SoftSerial's RX pin to which PZEM-004's TX connected;
   - _rxPin_ SoftSerial's TX pin to which PZEM-004's RX connected;
 - _pzem004.current[rxPin, txPin]_ command return PZEM-004's current value;
 - _pzem004.power[rxPin, txPin]_ command return PZEM-004's power value;
 - _pzem004.energy[rxPin, txPin]_ command return PZEM-004's energy value.

**Note** For authorization on PZEM-004 hardcoded fixed default internal IP (192.168.1.1) is used at this time. It's haven't any relation to yours LAN IP.


####05 Sep 2016
Changes:
 - New macro - _NETWORK\_MODULE_ . Now network interface's libs including depend of _NETWORK\_MODULE_ value. Its allow to recompile source code for various modules without headers #includes mass commenting/uncommenting.
   Unfortunatly, oldest releases of Arduino IDE may throw error when that trik used and you must plug in headers by own hand. IDE v1.6.11 works correctly with NETWORK_MODULE macro;
 - To _analogRead[]_ command added _mapToLow_ and _mapToHigh_ arguments to be called with _map(..., 0, 1023, mapToLow, mapToHigh)_ Arduino function. Example _analogRead[15,, 0, 8]_ equal to _map(analogRead(15), 0, 1023, 0, 8)_ ;

Fixes:
 - Zabbuino now tested on ATmega32u4-based boards (Arduino Micro, Leonardo). Small oddities in debugging with Serial exist at this moment, but basic functions performs well.


New command:
 - _sys.mcu.id_ command return ID of ATMega chip: http://www.avrfreaks.net/forum/unique-id-atmega328pb .


####25 Aug 2016

New commands:
 - _WS2812.SendRaw[dataPin, data]_ - send data to WS2812 led stripe.
   - _pwmPin_ - pin to which WS2812's DIN connected;
   - _data_ - prefixed HEX-string that contain encoded color data. Every led color specify by group of six HEX-numbers - two number (one byte) for every color in GRB order. Number of HEX groups is equal to number of leds in stripe. [8 leds bar](https://ru.aliexpress.com/item/Free-Shipping-NeoPixel-Stick-8-channel-WS2812-5050-RGB-LED-lights-built-in-full-color-driven/32582877809.html) example: zabbix_get.exe -s 192.168.0.1 -k "ws2812.sendraw[5,0x00FF00 001100 0000FF 000011 FF0000 110000 003333 330033]" (max red, min red, max blue, min blue and etc.);

**Note #1** Code was taken from [Adafruit's NeoPixel](https://github.com/adafruit/Adafruit_NeoPixel) library and handles 800 KHz bitstreams on 16 MHz ATmega MCUs only.

**Note #2** The _shiftOut[]_ command can be temporary broken due bit-banging code refactored and not tested yet on real hardware. It's will be tested soon.
**Note #2 update** The _shiftOut[]_ command has been tested and fixed.


####15 Aug 2016

**Zabbuino 1.0.0 released** 

Work on Zabbuino 1.1.x is begin.

####10 Aug 2016

Fixes:
- _BMP.*/BME.*_ commands sometime returns unexpectedly large value of BMP280/BME280 metrics.


####29 Jul 2016

New commands:
- _IR.Send[pwmPin, irPacketType, nBits, data]_ - send command or data to IR-driven device (TV, Soundbar, etc);
  - _pwmPin_ - pin to which IR-LED connected. It must be OC2B, due IRRemote code implemented;
  - _irPacketType_ - transmitted data packet format, refer to _ir.ino_ ;
  - _nBits_ - refer to IRRemote code comments;
  - _data_ - command or other data, that must be transmitted to IR-driven device.

- _IR.SendRaw[pwmPin, irFrequency, nBits, data]_ - send RAW bit stream to IR-driven device;
  - _pwmPin_ - pin to which IR-LED connected. It must be OC2B, due IRRemote code implemented;
  - _irFrequency_ - IR-LED's frequency;
  - _nBits_ - refer to IRRemote code comments;
  - _data_ - HEX-string (with _0x_ prefix) that represent IRRemote RAW-data array. Every IRRemote array item must be encode to four-HEX substring (for example: 509 => 01FD). Take in account, that you need to increase ARGS_PART_SIZE.

**Note #1** _IR.*_ commands not tested at all, because i haven't TV's of all vendors ;)


####27 Jul 2016

Fixes:
- _bme.*_/_bmp.*_ commands show previous measured metric values instead actual data (default behavior for _BMP280/BME280 forced mode_);


####19 Jul 2016

Changes:
- _pc8574.write_ command code moved to  _i2c.write_ ;
- _pc8574.LCDPrint_ command renamed to _pc**f**8574.LCDPrint_ (just my inattention caused error in chip name);
- _pcf8574.LCDPrint_ have new function. When no data specified, the command toggle backlight in accordance with the value of _lcdBacklight_ option.

New commands:
- _I2C.Write[sdaPin, sclPin, i2cAddress, register, data]_ - write data (one byte) to the register of I2C device.
  - _sdaPin, sclPin_ - I2C pins;
  - _i2cAddress_ - address of I2C device. It can be found with _I2C.scan[]_ command;
  - _register_ - register, which must be written (optional);
  - _data_ - one byte data to expander write.
- _I2C.Read[sdaPin, sclPin, i2cAddress, register, bytes]_ - read the value with _bytes_ lenght from the register of I2C device;
  - _sdaPin, sclPin_ - I2C pins;
  - _i2cAddress_ - address of I2C device. It can be found with _I2C.scan[]_ command;
  - _register_ - register, which must be read (optional);
  - _bytes_ - how many bytes need to read (1 .. 4, uint8_t .. uint32_t).
- _I2C.BitWrite[sdaPin, sclPin, i2cAddress, register, bitNumber, value]_ - write _value_ to  _bitNumber_ of I2C device's register;
  - _sdaPin, sclPin_ - I2C pins;
  - _i2cAddress_ - address of I2C device. It can be found with _I2C.scan[]_ command;
  - _register_ - register, which must be written (optional);
  - _bitNumber_ - bit, which must be written (0 .. 7);
  - _value_ - 1 or 0.
- _I2C.BitRead[sdaPin, sclPin, i2cAddress, register, bitNumber]_ - read _value_ from  _bitNumber_ of I2C device's register;
  - _sdaPin, sclPin_ - I2C pins;
  - _i2cAddress_ - address of I2C device. It can be found with _I2C.scan[]_ command;
  - _register_ - register, which must be read (optional);
  - _bitNumber_ - bit, which must be read (0 .. 7).

**Note #1** Set of I2C.* commands allow to handle most of simple I2C devices - PCF8574/PCF8574A expanders, MCP4018T digital linear potentiometer, and etc.

**Note #2** Some I2C devices havent registers (PCF8574 expander for example). Just skip value of _register_ option when work with it.

####15 Jul 2016
New commands:
- _ultrasonic.distance[triggerPin, echoPin]_ - returns from obtained HC-SR04 module distance (in **mm**) to the object.
  - _triggerPin_ - pin to which HC-SR04 "Trig" connected;
  - _echoPin_ - pin to which HC-SR04 "Echo" connected;

- _pc8574.write[sdaPin, sclPin, i2cAddress, data]_ - write data (one byte) to PC8574/PC8574A I2C expander. 
  - _sdaPin, sclPin_ - I2C pins;
  - _i2cAddress_ - address of PC8574/PC8574A expander. It can be found with _I2C.scan[]_ command;
  - _data_ - one byte data to expander write.

**Note #1** - _ultrasonic.distance_ returns 6551 if the object out of distance range (too close or too far).
 

####13 Jul 2016
Changes:
- Programm features section moved to zabbuino.h for code reorganization to reducing firmware size;
- AnalogReference() related functions disabled until FEATURE_AREF_ENABLE defined. It must prevent MCU damage (see AREF pin connect recommendation).

New commands:
- _acs7xx.zc[sensorPin, refVoltage]_ - returns Arduino ADC's "zero point". ACS7xx sensor's load must be disconnected before command using.
  - _sensorPin_ - pin (analog) to which ACS7xx sensor's output is connected;
  - _refVoltage_ - reference voltage settings. Can be an number that defined as DEFAULT for Arduino's _analogReference()_ function on your platform, or voltage value (in mVolts) on AREF pin (please read all notes about AREF pin using).
- _acs7xx.dc[sensorPin, refVoltage, sensitivity, zeroPoint]_ - returns current for DC. 
  - _sensorPin_ - pin (analog) to which ACS7xx sensor's output is connected;
  - _refVoltage_ - reference voltage settings. Refer to _acs7xx.zc[]_ command description;
  - _sensitivity_ - sensitivity of used sensor in A/mV. For example - 185 for ACS712-05A or 66 for ACS712-30A. Refer to sensor datasheet;
  - _zeroPoint_ - Arduino ADC's "zero point". For example - 511. It can be found with _acs7xx.zc[]_ command.

**Note #1** - _acs7xx.ac_ presently not ready to use.
 
**Note #2** - The value that measured by _acs7xx.*_  can have large error (may be 5% or more) due to Arduino's circuit design (unstable internal reference voltage, noisy ADC and so) and _acs7xx_ noise (21mv / 0.11A for ACS712-05) exists. To get better results you can try to use external reference voltage on [AREF pin](http://www.skillbank.co.uk/arduino/measure.htm).

####12 Jul 2016

Fixes:
- DHT reading unstability (see note #1);
- _BH1750.light_ command. It tested on real hardware and works now;

Changes:
- _BMP.*_ commands supports BMP280 sensor now;
- New option for _BMP.Temperature_ command if BMP280/BME280 used: _overSampling_ => _BMP.Temperature[sdaPin, sclPin, i2cAddress, overSampling]_. For more details refer to BMP280 datasheet, section "3.4 Filter selection";
- New option for _BMP.Pressure_ command if BMP280/BME280 used: _filterCoef_ => _BMP.Pressure[sdaPin, sclPin, i2cAddress, overSampling, filterCoef]_. For more details refer to BMP280 datasheet, section "3.3.3 IIR filter".

New commands:
- _BME.Humidity[sdaPin, sclPin, i2cAddress, overSampling, filterCoef]_ - returns humidity value, obtained from BME280 (equal BMP280 + Humidity) sensor.
  - _sdaPin, sclPin_ - I2C pins;
  - _i2cAddress_ - address of BME280 sensor. It can be found with _I2C.scan[]_ command;
  - _overSampling_ - hardware sampling accuracy modes, refer to BMP280 datasheet, section "3.4 Filter selection";
  - _filterCoef_ - IIR filter setting, refer to BMP280 datasheet, section "3.3.3 IIR filter".
- _SHT2x.Humidity[sdaPin, sclPin, i2cAddress]_ - returns humidity value, obtained from SHT2x sensor;
- _SHT2x.Temperature[sdaPin, sclPin, i2cAddress]_ - returns temperature value, obtained from SHT2x sensor;

**Note #1** DHT sensor have limit to readings frequency. Its a 1kHz (1 reading / sec). Because of this a little delay implemented to function. It stops the process until the one second expitre since the last time reading. If Zabbix put so close into poller queue requests to DHT sensor, Zabbuino slowed the output of the results, because it will execute commands in sequence and every time will stops for a little time (~0.5 .. 0.7 sec). 	
Therefore need to specify Data Item's update interval to distribute polls on timeline with more that 1 sec gaps. For example: _DHT.Humidity_ - 60 sec, _DHT.Temperature_ - 57 sec. **Warning!** All connected to Zabbuino DHT's have the same delay counter and polls must be distributed on timeline together.


**Note #2** _SHT2x.*_ commands tested on SHT21 module (SI7021 chip).


####09 Jul 2016

Unfortunately, i've found that DHT-functions of Zabbuino works unstable on long time testing and can hang up the system. I keep on to seek a solution.

####08 Jul 2016

Fixes:
- Reboot on incoming data buffer overflow;

Changes:
- DHT functions was reworked, part of DHTlib source code is used. Now DHT33 & DHT44 is supported.

New commands:
- _pc8574.LCDPrint[sdaPin, sclPin, i2cAddress, lcdBacklight, lcdType, data]_ - this command print text that extracted from _data_ to character HD44780-based LCD, connected to I2C via PC8574 expander.
  - _sdaPin, sclPin_ - I2C pins;
  - _i2cAddress_ - address of PC8574 expander. It can be found with _I2C.scan[]_ command;
  - _lcdBacklight_ - LCD backlight mode. 0 - off, other - on;
  - _lcdType_ - type of LCD: 801, 1601, 802, 1202, 1602, 2002, 2402, 4002, 1604, 2004, 4004;
  - _data_ - HEX-coded string that contain text and display commands. It must begin with _0x_ prefix. Use [http://www.asciitohex.com](http://www.asciitohex.com/) to recode yours text or write small perl script.
- _pc8574.LCDBLight[sdaPin, sclPin, i2cAddress, lcdBacklight]_ - set LCD backlight mode. _lcdBacklight_ => 0 - off, other - on. Command can be abolished in future;

**Note** You must increase _ARGS_PART_SIZE_ in _zabbuino.h_  if full text want to see on display ;)

Supported display HD44780-compatible commands:

- _0x00_ - turn display off. Need to send any other command to turn display on;
- _0x01_ - clear screen;
- _0x02_ - go to home (move cursor to top/left character position);
- _0x04_ - print from right to left (look to animated gifs on [http://www.dinceraydin.com/lcd/commands.htm)](http://www.dinceraydin.com/lcd/commands.htm);
- _0x05_ - print from right to left with screen shifting;
- _0x06_ - print from left to right;
- _0x07_ - print from left to right with screen shifting;
- _0x08_ - blank the screen (without clearing);
- _0x0C_ - turn cursor off;
- _0x0E_ - turn on "underline" cursor;
- _0x0F_ - turn on "blinking block" cursor;
- _0x10_ - move cursor one char left;
- _0x14_ - move cursor one char right;
- _0x18_ - shift screen to left;
- _0x1E_ - shift screen to right.

Additional display commands:
- _0x03_ - blink by backlight twice;
- _0x09_ - print four spaces (simply "tab");
- _0x0A_ - line feed, go to newline, position 0;
- _0x80..0x9F_ - set cursor to _(N - 0x80)_ position. I.e. _0x80_ - position 0 (first char), _0x81_ - position 1 (second char), and so;


Example:


Print on LCD of 2002 type with turned backlight, that connected via expander with I2C address 0x27:

    zabbix_get.exe -s 192.168.0.1 -k "pc8574.lcdprint[18,19,0x27,1,2002,0x01 93 04 48 65 6c 6c 6f]"

    0x 01             93                    04                            48 65 6c 6c 6f
       clear screen   from 20-th position   from the right to the left    H  e  l  l  o

    zabbix_get.exe -s 192.168.0.1 -k "pc8574.lcdprint[18,19,0x27,1,2002,0x01 0F 48 65 0A 6c 09 6c 6f]"

    0x 01              0F                    48 65 0A          6c   09           6c 6f
       clear screen    turn "block" cursor   H  e  line feed   l    four space   l  o

**Note #1** I'm not sure about correct work of _XX04_ displays. It should be tested, but i haven't hardware.

**Note #2** BMP280 sensor support is added, but not tested yet. I just wait for sensors pack;

####01 Jul 2016

New command:
- _incEnc.count[terminalAPin, terminalBPin, intNumber, initialValue]_ - this command allow to get signed long counter, that was increased and decreased by incremental encoder (tested on mechanical EC12E24204A9) that connected to pin which mapped to interrupt.
  - _terminalAPin_ - which pin used to connect first encoder terminal and catching interrupt (refer to https://www.arduino.cc/en/Reference/AttachInterrupt);
  - _terminalBPin_ - which pin used to connect second encoder terminal;
  - _intNumber_ - not used at this time, reserved for future;
  - _initialValue_ - defines initial counter value. 

**Note #1** You can reverse encoder's "incrementing" direction - just connect wire on _terminalAPin_ to encoder's terminal B and wire on _terminalBPin_ to encoder's terminal A.

**Note #2** Code of interrupt's handling not yet optimized and can be buggy.

Testings:
 - _extInt.count_ command tested on tilt switch SW-520D (CHANGE mode) and PIR-sensor HC-SR501 (RISING mode). All events registred well by counter.



####30 Jun 2016

Fixes:
- _extInt.count_ always attached with LOW mode. Silly typo;
- float numbers in return values had unnecessary zeros after the decimal point.

Changes:
- _analogReferenceSource_ optional parameter added to _analogRead_ command. Now, if _analogReferenceSource_ specified, Zabbuino set source of the reference voltage before read from analog pin;
- _duration_ parameter in _tone_ command now optional. If its not specified - tone generation can be stopped only with _noTone_ command. May be its evil, don't know;
- _value_ parameter in _randomSeed_ command now optional. If its not specified - the pseudo-random number generator is initializes by _millis()_;
- _min_ parameter in _random_ command now optional. If only one parameter in command _random_ is specified - called Arduino's _random(max)_ to get next number;

[Russian manual](https://github.com/zbx-sadman/Zabbuino/wiki/Zabbuino-in-Russian) is almost finished, [English manual](https://github.com/zbx-sadman/Zabbuino/wiki/Zabbuino-in-English) partially translated using Google translator.


####27 Jun 2016

New command:
- _MAX7219.write[dataPin, clockPin, loadPin, intensity, value]_ - draw on 8x8 led matrix which connected to MAX7219. You can change _intensity_ (0..15) and send _value_ as HEX string to switch on leds in line. Every two HEX char specify one line leds state. For example - _0x6666001818817E00_ will draw smile.

For example - you can use _MAX7219.write[]_ to indicate via Zabbix Action (Operation type: Remote command && Execute on: Zabbix Server && a little shell script) your's Zabbix server mood - a smiling or sad. Also, with using Zabbix API yo can get number of triggers with various severety and draw a hystogram. Or send mystic sign to display in the remote VPN'ed office. Or do something more fun.

Example scripts: 
- [send smiled robot's face if no active triggers exist and sad robot's face if any trigger active](https://github.com/zbx-sadman/Zabbuino/tree/master/v1.0.0_pre-release/examples/smileWithZabbuino.sh)
- [draw hystogram that represent on led matrix the number of active triggers](https://github.com/zbx-sadman/Zabbuino/tree/master/v1.0.0_pre-release/examples/sendHystogrammToZabbuino.pl)


####26 Jun 2016

At first words: command names is not case sensitive, you can write them in your own style (_SyS.CmDCouNT_, for example).

Fixes:
- _BMP.Pressure[]_ command can be hang up system. 
- "millis() overflow after 50 days runtime" case now is handled well. 

Changes:
- Some commands is reorganized to achieve a more structured: 
  - _sys.CmdCount_ -> _sys.cmd.count_;
  - _sys.CPUname_  -> _sys.MCU.name_;
  - _sys.NetModule_ -> _sys.net.module_;
  - _sys.FreeRAM_ -> _sys.RAM.free_;
  - _interrupt.count_ -> _extInt.count_;
  - _DS18x20.Search_ -> _OW.scan_;
  - _sethostname_ -> _set.hostname_; 
  - _setnetwork_ -> _set.network_; 
  - _setpassword_ -> _set.password_; 
  - _setprotection_ -> _set.sysprotect_
- _OW.scan_ (was _DS18x20.Search_) now return list of Devices ID's (first found ID was returned early), which have been found on specified by _pin_ OneWire bus;
- _sys.RAM.free_ now gather periodically (~ every 2 sec) like _sys.VCC[]_ metrics;

New commands:
- _sys.RAM.freeMin_ - minimal size of "RAM" (free space between the data area and stack: .DATA => | free RAM | <= .STACK ), registred since device is powered on. Gather at the same time with _sys.RAM.free_ metric;
- _sys.cmd.timeMax_ - maximal time of command execution in _ms_. Exclude network library and Zabbix key parsing overheads.

Improvements:
- Now you can use more blinks to runtime stage (refer to source code, please) indication. Just uncomment _#define ADVANCED_BLINKING_ in _zabbuino.h_

*Note* To avoid gaps on Zabbix graphs try to increase _Timeout_ directive in _zabbix_server.conf_. Try 10 sec for example. Or you can decrease number of Data Items. Choose one or both.

So, you can import [zabbuino.xml](https://github.com/zbx-sadman/Zabbuino/tree/master/v1.0.0_pre-release/zabbuino.xml) to your Zabbix Server (v2.4 min) to see Data Items examples.

####17 Jun 2016

Changes:
- System (internal) metrics (like _sys.vccmin_ / _sys.vccmax_ ) gathering process can be use timer interrupt instead calling sub on every loop. I think this will help to get more reliable results of VCC rising/falling (for example) with sensors that have long-time conversion (like DS18x20). So, that feature may be do system unstable when PWM is used (with _analogWrite[]_ command);
- Code reorganized;

New command:
- _interrupt.count[intPin, intNumber, mode]_. This command allow to get unsigned long counter, that was incremented by external interrupt. It's can be used in DYI anemometer projects, for example. On first (after power on) call of _interrupt.count_ command _intPin_ will be switched to INPUT_PULLUP mode and attached to interrupt. On next call number of rising/failing/changing will be returned. If _mode_ is changed for _intPin_, that already used by interrupt - counter will be reset and interrupt will be reattached on new _mode_;
  - _intPin_ - which pin used to interrupt catching. For ATMega328p this can be 2 or 3 (refer to https://www.arduino.cc/en/Reference/AttachInterrupt );
  - _intNumber_ - not used at this time, reserved for future;
  - _mode_ - defines when the interrupt should be triggered. Four constants are predefined as valid values: 0 - LOW, 1 - CHANGE, 2 - FALLING, 3 - RISING

I'm not sure that _interrupt.count_ will be run stable and properly, due not test it in production, but i hope for it.

**Note #1** You can get unexpected growing of _interrupt.count_ value. It's can be 'button bounce' or 'bad electrical contact' problem.


####15 Jun 2016

Changes:
- _agent.cmdcount_ renamed to _sys.cmdcount_, because original Zabbix agent haven't _agent.cmdcount_ command;
- numeric arguments can be digital, hexadecimal (C++ _strtoul()_ function was used now for converting); 
- To I2C-sensor-related commands added _I2CAddress_ argument. That need for 2-addresses devices like BH1750. Now _BMP's_ commands is: 
  - _BMP.Temperature[sdaPin, sclPin, i2cAddress]_; 
  - _BMP.Pressure[sdaPin, sclPin, i2cAddress, overSampling]_,

New commands:
- _bh1750.light[sdaPin, sclPin, i2cAddress, mode]_ - return light intensity in _lux_. 
  - i2cAddress: 0x23 or 0x5C; 
  - mode: 0x20 - ONE_TIME_HIGH_RES_MODE, 0x21 - ONE_TIME_HIGH_RES_MODE_2, 0x23 - ONE_TIME_LOW_RES_MODE (refer to datasheet for more details);  
- _i2c.scan[sdaPin, sclPin]_ - return addresses of devices that will be found on I2C bus; 
- _sys.netmodule_ - return netmodule name, that depends on #included library: _W5xxx_ or _ENC28J60_. 

**Note #1** _bh1750.light_  do not tested with real hardware.

####13 Jun 2016

Seems that no dirty hacks with ENC28J60 need. UIPEthernet library have https://github.com/ntruchsess/arduino_uip/tree/fix_errata12 brahch. Its newer that _master_-branch and fix some freezes (may be all?).

Fixes:
- _analogRead[]_ command work was blocked by measureVoltage procedure.

New commands:
- _sys.uptime_ - system working time since power on.

####10 Jun 2016

I this time i make test with only W5100 Ethernet Shield only. ENC28J60 testing coming soon. Detailing source code commenting coming soon too.

*UPD:* So, i was upload sketch to second Arduino: Mini Pro + ENC28J60 and make stress test with  _hping3 --flood <ipaddr>_. ENC stops responding to _zabbix\_get_ and another _ping_ session (non-flood). Soon the ENC was re-inited by Zabbuino, non-flood _ping_ and and _zabbix\_get_ works was restored. 

Improvements:
- Network concurrent connections problem fixed. 4 Zabbix item's with 10 sec polling period works good;
- More RAM is free. More Flash is used.

New features:
- Loading and saving network settings (and several other) to EEPROM for remote configuring and changing settings without re-flashing sketch;
- Factory reset button can be connected directly to pins (internal pull-up resistor is used);
- AVR Watchdog support;
- DHCP support;
- Do periodical ENC28J60 init() if no network activity for some time. This action may helps to kick ENC28J60 if that no answer. May be. I hope for it.

New commands:
- _sys.cpuname_ - controller name if known;
- _sys.vcc_ - current VCC (mV);
- _sys.vccmin_ - minimal VCC (mV) for power on time. Gather periodically (~ every  2 sec);
- _sys.vccmax_ - maximal VCC (mV) for power on time. -"-"- ;
- _sethostname[password, hostname]_ - Set and store to EEPROM set new hostname;
- _setpassword[oldPassword, newPassword]_ - Set and store to EEPROM new password;
- _setprotection[password, protection]_ - Store to EEPROM enable / disable password protection flag for _set\*_ and _reboot_ commands;
- _setnetwork[password, useDHCP, macAddress, ipAddress, ipNetmask, ipGateway]_ - Store to EEPROM new network settings. macAddress, ipAddress, ipNetmask, ipGateway is hex-strings. Use http://www.miniwebtool.com/ip-address-to-hex-converter/ to converting from decimal format. Changing useDHCP on fly may be buggy at this time. Need _reboot_ to apply;
- _reboot[password]_ - soft-reboot (_asm: jmp 0_);
- _delay[value]_ - Arduino _delay()_ wrapper;
- _DS18x20.Search[pin]_ - Call OneWire search for Dallas devices, connected to _pin_;


May be something else...
