// Config & common included files
#include "sys_includes.h"


#include <SoftwareSerial.h>

#include "ModbusMaster\ModbusMaster.h"
#include "modbus.h"

#define MAX485_RE_NEG 5
#define MAX485_DE 5

void preTransmission()
{
  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);
}

void postTransmission()
{
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
}

/*****************************************************************************************************************************
*
*
*
*   Returns: 
*     - 
*
*****************************************************************************************************************************/
uint8_t getModbusRTUMetric() {
  int8_t rc = DEVICE_ERROR_TIMEOUT;
  uint8_t _rxPin=3, _txPin=4;
  uint8_t result;
  uint16_t data[6];

  // instantiate ModbusMaster object
  ModbusMaster node;

  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);

  SoftwareSerial swSerial(_rxPin, _txPin);

  // Modbus slave ID 1
  node.begin(1, swSerial);

  // Callbacks allow us to configure the RS485 transceiver correctly
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  result = node.readInputRegisters(0x3100, 16);
  if (result == node.ku8MBSuccess) {
      Serial.println(node.getResponseBuffer(0x04));
      Serial.println(node.getResponseBuffer(0x0D));
  }

  finish:
  swSerial.~SoftwareSerial(); 
  return rc;
}
