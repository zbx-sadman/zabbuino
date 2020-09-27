#if defined(ARDUINO_ARCH_ESP32)
/**
   When using ESP32 with the Arduino IDE, it does not support the native watchdog timer.
   The following watchdog implementation does not use the official watchdog timer
   but employs a normal timer to reset the CPU if wdt_reset() is not called in time.
   Inspired by the following discussion:
   https://github.com/espressif/arduino-esp32/issues/841
   Credit goes to @me-no-dev.
*/
#ifndef WDT_H
#define WDT_H
/**
   Enable the watchdog timer, configuring it for expiry
   after durationMs milliseconds.
*/
void wdt_enable(const unsigned long durationMs);

/**
   Disable the watchdog timer.
*/
void wdt_disable();

/**
   Reset the watchdog timer. When the watchdog timer is enabled,
   a call to this method is required before the timer expires,
   otherwise a watchdog-initiated device reset will occur.
*/
void wdt_reset();

#endif
#endif
