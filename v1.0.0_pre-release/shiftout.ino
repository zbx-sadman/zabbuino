/* ****************************************************************************************************************************
*
*  Advanced shiftOut()
*  Can get HEX-string as the data to shift out
*
**************************************************************************************************************************** */
void advShiftOut(const uint8_t _dataPin, const uint8_t _clockPin, const uint8_t _bitOrder, char* _dataBuffer)
{
  uint16_t len = 0;
  uint8_t dataPinBit, clockPinBit;
  volatile uint8_t *dataPortOutputRegister, *clockPortOutputRegister;
  uint8_t oldSREG;

  dataPinBit = digitalPinToBitMask(_dataPin);
  clockPinBit = digitalPinToBitMask(_clockPin);
  dataPortOutputRegister = portOutputRegister(digitalPinToPort(_dataPin));
  clockPortOutputRegister = portOutputRegister(digitalPinToPort(_clockPin));

  /* from wiring_digital.c
  byte dataPinTimer = digitalPinToTimer(_dataPin);
  byte clockPinTimer = digitalPinToTimer(_clockPin);
  if (clockPinTimer != NOT_ON_TIMER) turnOffPWM(clockPinTimer);
  if (dataPinTimer != NOT_ON_TIMER) turnOffPWM(dataPinTimer);
  */

  // Do special procedure if incoming data is the hexadecimal string 
  if (haveHexPrefix(_dataBuffer)) {
    // Skip "0x"
    _dataBuffer += 2; 
    // "Least Significant Bit First" bit order must be used? 
    if (_bitOrder == LSBFIRST) {
       // Walk over buffer to the '\0' - calculate its length
       while (*_dataBuffer) { len++; _dataBuffer++; }
       // disable interrupt
       oldSREG = SREG; cli();
       // Repeat cycle for a 'len' times - to take all chars by moving pointer to backward
       while (len--) {
         _dataBuffer--;
        // Convert one HEX char to DEC and push that 4 bit to Shift Register 
         shiftOut4bits(dataPortOutputRegister, dataPinBit, clockPortOutputRegister, clockPinBit, _bitOrder, htod(*_dataBuffer));
      }
    } else  {
      // "Most Significant Bit First" bit order is used
      // disable interrupt
      oldSREG = SREG; cli();
      // Walk to the buffer end ('\0' char)
      while (*_dataBuffer) {
        // Convert one HEX char to DEC and push that 4 bit to Shift Register 
        shiftOut4bits(dataPortOutputRegister, dataPinBit, clockPortOutputRegister, clockPinBit, _bitOrder, htod(*_dataBuffer));
        // Move pointer to next char
        _dataBuffer++;
      }
    }
    // enable interrupts
    SREG = oldSREG;
  } else {
    // Data is not hex-string. Push its to Shift Register as integer
    shiftOut(_dataPin, _clockPin, _bitOrder, atoi(_dataBuffer));
  }
}

/* ****************************************************************************************************************************
*
*  Four-bit (one HEX-char) shiftOut(). Use direct port manipulation.
*
**************************************************************************************************************************** */
void  shiftOut4bits(volatile uint8_t *_dataPortOutputRegister, const uint8_t _dataPinBit, volatile uint8_t *_clockPortOutputRegister, const uint8_t _clockPinBit, const uint8_t _bitOrder, const uint8_t _val)
{
  uint8_t i, currBit;
  for (i = 0; i < 4; i++)  {
    if (_bitOrder == LSBFIRST)
      // Get i-th bit from end of bit chain. "!!" convert value 0100 to 1.
    {
      currBit = !!(_val & (1 << i));
    }
    else
      // Get i-th bit from begin of bit chain
    {
      currBit = !!(_val & (1 << (3 - i)));
    }

    if (currBit == LOW)
    {
      *_dataPortOutputRegister &= ~_dataPinBit;
    }
    else
    {
      *_dataPortOutputRegister |= _dataPinBit;
    }

    // Make bit-blink with clockPin
    //	eq digitalWrite(_clockPin, LOW);
    *_clockPortOutputRegister &= ~_clockPinBit;
    //	eq digitalWrite(_clockPin, HIGH);
    *_clockPortOutputRegister |= _clockPinBit;
  }
}

