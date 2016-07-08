# Zabbuino 1.0.0 (pre-release)


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

**Note** I'm not sure about correct work of _XX04_ displays. It should be tested, but i haven't hardware.


####01 Jul 2016

New command:
- _incEnc.count[terminalAPin, terminalBPin, intNumber, initialValue]_ - this command allow to get signed long counter, that was increased and decreased by incremental encoder (tested on mechanical EC12E24204A9) that connected to pin which mapped to interrupt.
  - _terminalAPin_ - which pin used to connect first encoder terminal and catching interrupt (refer to https://www.arduino.cc/en/Reference/AttachInterrupt);
  - _terminalBPin_ - which pin used to connect second encoder terminal;
  - _intNumber_ - not used at this time, reserved for future;
  - _initialValue_ - defines initial counter value. 

**Note** You can reverse encoder's "incrementing" direction - just connect wire on _terminalAPin_ to encoder's terminal B and wire on _terminalBPin_ to encoder's terminal A.

**Note** Code of interrupt's handling not yet optimized and can be buggy.

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

**Note** You can get unexpected growing of _interrupt.count_ value. It's can be 'button bounce' or 'bad electrical contact' problem.


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

**Note** _bh1750.light_  do not tested with real hardware.

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
