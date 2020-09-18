# Zabbuino
### 18-09-2020
Segments probally done:

- Network (alpha release mode ;)
- Generic Arduino commands wraps;
- EEPROM (configure stuff);
- I2C;
- OneWire (Maxim Dallas protocol)

*Note*: Hardware compatibility not fully tested.

Added new command:
- `set.wifi[password,"APName",psk]` , where `password` is Zabbuino's system password, `APName` - name of Access Point (minimal lenght - 8 sym), `psk` - AP PreShared Key (minimal lenght - 8 sym if used, psk can be omitted on open networks). Please, use doublequote if you have case sensivity AP name/password. Example: `set.wifi[password,"Tr0n",row8c1j9e]`


### 10-09-2020
v1.5 started. 

Plan: add support for ESP2866.
Used hardware: Wemos D1 Mini
Used software: Arduino IDE 1.8.9 + Arduino Wiring-based Framework (ESP8266 Core) 2.4.7.