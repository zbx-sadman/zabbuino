# Zabbuino (ESP integration test)

### 19-09-2020
Segments probally done:
- DHT/AM sensors;

Added new command:
- `dht.all[pin, model]` - returns JSON with both metric values. Example: `dht.all[5,21]`=> `{"h":31.9,"t":25.4}`

### 18-09-2020
Segments probally done:

- Network (alpha release mode ;)
- Generic Arduino commands wraps;
- EEPROM (configure stuff);
- I2C;
- OneWire (Maxim Dallas protocol)

Added new command:
- `set.wifi[password,"APName",psk]` , where `password` is Zabbuino's system password, `APName` - name of Access Point (minimal lenght - 8 sym), `psk` - AP PreShared Key (minimal lenght - 8 sym if used, psk can be omitted on open networks). Please, use doublequote if you have case sensivity AP name/password. Example: `set.wifi[password,"Tr0n",row8c1j9e]`

**Note**: Hardware compatibility not fully tested.
**Note**: Pins number in the commands is GPIO number. `i2.scan[4,5]` act with GPIO4 (D2) & GPIO5 (D1), not D4 & D5.

### 10-09-2020
v1.5 started. 

Plan: add support for ESP2866.

Used hardware: Wemos D1 Mini

Used software: Arduino IDE 1.8.9 + Arduino Wiring-based Framework (ESP8266 Core) 2.4.7.
