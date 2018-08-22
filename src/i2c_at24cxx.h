#pragma once

int8_t AT24CXXWrite(SoftwareWire* _softTWI, const uint8_t _i2cAddress, uint16_t _cellAddress, uint16_t _lenght, uint8_t* _src);
int8_t AT24CXXRead(SoftwareWire* _softTWI, const uint8_t _i2cAddress, const uint16_t _cellAddress, const uint16_t _lenght, uint8_t* _dst);

