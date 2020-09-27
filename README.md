# Zabbuino (ESP integration test)

### 27-09-2020

ESP32 support added and tested with pair of sensors - OneWire & I2C. I hope this will works in general...

### 24-09-2020
Changes:
 - Winsen sensor "core" reworked.

Added new command:
 - `sys.all` - returns JSON with some system metric. Example: `sys.all`=> `{"sysRamFree":51768,"sysRamFreeMin":51584,"sysVcc":3078,"sysVccMin":3077,"sysVccMax":3085,"sysCmdCount":2,"netPHYReinits":55}`

### 23-09-2020
Segments probally done:
 - Pixel leds support;

Changed:
 - Command `ws2812.sendraw[]` renamed to `ws281x.sendraw[dataPin, speed, compressionType, data]` and take new option `speed` - 400/800KHz switch. Now you can use old WS2811 leds too. Just use value `400` for 400Khz, `800` or nothing for 800Khz.
Example: `ws281x.sendraw[5,400,1,0x0F0 010 00F 001 F00 100 033 303]` - send data do WS2811 (400KHz), `ws281x.sendraw[5,,1,0x0F0 010 00F 001 F00 100 033 303]` - send data do WS2812/2813 (800KHz by default).


### 21-09-2020
Segments probally done:
 - Software SPI stuff and sensors handling;
 - Shiftout feature (w/o pixel leds support).

Changed:
 - "Factory reset button" is "User function button" now.

Added new feature:
 - Two new user-level function added: userFunctionButtonActivate() & userFunctionButtonDeactivate(). They are called when "User function button" stay in some state (on/off) for constUserFunctionButtonDebounceTime.

For example, you can use userFunctionButtonActivate() to system factory reset on ESP2866 based device like Sonoff TH10/TH16, which have onboard button connected to GPIO0 (GPIO0 can't be used to factory reset on boot stage);

### 20-09-2020
Segments probally done:
- Servo, relay;
- UART sensors sensors;

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
- I2C sensors;
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
