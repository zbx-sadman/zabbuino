# Zabbuino 1.0.0 (pre-release)

####15 Jun 2016

Changes:
- _agent.cmdcount_ renamed to _sys.cmdcount_, because original Zabbix agent haven't _agent.cmdcount_ command.
- To I2C sensors command added _I2CAddress_ field. That need for 2-addresses devices like BH1750. Now _BMP's_ commands is: 
  - _BMP.Temperature[sdaPin, sclPin, i2cAddress]_; 
  - _BMP.Pressure[sdaPin, sclPin, i2cAddress, overSampling]_;

New commands:
- _bh1750.light[sdaPin, sclPin, i2cAddress, mode]_ - return light intensity in _lux_. 
  - i2cAddress: 0x23 or 0x5C; 
  - mode: 0x20 - ONE_TIME_HIGH_RES_MODE, 0x21 - ONE_TIME_HIGH_RES_MODE_2, 0x23 - ONE_TIME_LOW_RES_MODE (refer to datasheet for more details);  
- _i2c.scan[sdaPin, sclPin]_ - return addresses of devices that will be found on I2C bus; 
- _sys.netmodule_ - return netmodule name, that depends on #included library: _W5xxx_ or _ENC28J60_. 

*Note* _bh1750.light_  do not tested with real hardware; 

####13 Jun 2016

Seems that no dirty hacks with ENC28J60 need. UIPEthernet library have https://github.com/ntruchsess/arduino_uip/tree/fix_errata12 brahch. Its newer that _master_-branch and fix some freezes (may be all?).

Fixed:
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
