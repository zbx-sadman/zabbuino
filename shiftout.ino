
/* ****************************************************************************************************************************
*
*  Расширенный аналог Функции shiftOut().
*  Поддерживает вывод на сдвиговый регистр значения в шестнадцатеричной форме.
*  Размер шестнадцатеричного числа ограничен размером буфера за вычетом длины остальных аргументов, служебных символов и длины команды.
*
**************************************************************************************************************************** */
void advShiftOut(uint8_t _dataPin, uint8_t _clockPin, uint8_t _bitOrder, char* _dataBuffer)
{
  uint16_t bufferReadPosition;
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

  // Выводимая величина задана в шестнацатеричном формате?
  if (_dataBuffer[0] == '0' && _dataBuffer[1] == 'x')
  {
    // Выводимое значение содержится в буфере, начиная с позиции 2 (после "0x")
    bufferReadPosition = 2;
    // Вывод в порядке "Least Significant Bit First"?
    if (_bitOrder == LSBFIRST)
    {
      // Необходимо найти конец буфера
      while (_dataBuffer[bufferReadPosition]) {
        bufferReadPosition++;
      }
      // Запрещаем прерывая для обеспечения атомарности операций манипуляций с портами
      oldSREG = SREG;
      cli();
      // Движение по данным происходит от конца буфера к началу.
      for (; bufferReadPosition > 1; bufferReadPosition--)
      {
        // Данные выводятся по 4 бита - один шестнадцатеричный символ
        shiftOut4bits(dataPortOutputRegister, dataPinBit, clockPortOutputRegister, clockPinBit, _bitOrder, htod(_dataBuffer[bufferReadPosition]));
      }
      // разрешаем прерывания
      SREG = oldSREG;
    } else  {
      // Запрещаем прерывания для обеспечения атомарности операций манипуляций с портами
      oldSREG = SREG;
      cli();
      // Движение по данным происходит от начала буфера к концу.
      while (_dataBuffer[bufferReadPosition])
      {
        // Данные выводятся по 4 бита - один шестнадцатеричный символ
        shiftOut4bits(dataPortOutputRegister, dataPinBit, clockPortOutputRegister, clockPinBit, _bitOrder, htod(_dataBuffer[bufferReadPosition]));
        bufferReadPosition++;
      }
      // разрешаем прерывания
      SREG = oldSREG;
    }
  } else {
    // Выводимая величина задана в десятичном формате. Необходима конвертация и вызов штатной подпрограммы.
    shiftOut(_dataPin, _clockPin, _bitOrder, atoi(_dataBuffer));
  }
}

/* ****************************************************************************************************************************
*
*  Аналог функции shiftOut, но для вывода 4-х бит (одного шестнацатеричного числа) с помощью манипуляций состоянием портов
*
**************************************************************************************************************************** */
void  shiftOut4bits(volatile uint8_t *_dataPortOutputRegister, uint8_t _dataPinBit, volatile uint8_t *_clockPortOutputRegister, uint8_t _clockPinBit, uint8_t _bitOrder, uint8_t _val)
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
    //	eq digitalWrite(_clockPin, HIGH);
    *_clockPortOutputRegister |= _clockPinBit;
    //	eq digitalWrite(_clockPin, LOW);
    *_clockPortOutputRegister &= ~_clockPinBit;
  }
}

