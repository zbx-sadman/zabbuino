/* ****************************************************************************************************************************
*
*   Set I/O port mode
*
**************************************************************************************************************************** */
void setPortMode(const uint8_t _port, const uint8_t _mode, const uint8_t _pullup)
{
  volatile uint8_t *modeRegister, *pullUpRegister;
  uint8_t oldSREG;

  // pull-up on/off register
  pullUpRegister = portOutputRegister(_port);
  // Input/Output toggle register
  modeRegister = portModeRegister(_port);

  if (modeRegister == NOT_A_PORT) return;
  // Сохранение значение регистра SREG / запрет прерываний / совершение записи / восстановление регистра SREG.
  oldSREG = SREG;
  cli();
  *modeRegister |= _mode;
  *pullUpRegister |= _pullup;
  SREG = oldSREG;
}

/* ****************************************************************************************************************************
*
*   Write to I/O port
*
**************************************************************************************************************************** */
void portWrite(const uint8_t _port, const uint8_t _value)
{
  volatile uint8_t *portRegister;
  uint8_t oldSREG;

  portRegister = portOutputRegister(_port);
  if (portRegister == NOT_A_PORT) return;
  // Атомарная запись в порт.
  // Сохранение значение регистра SREG / запрет прерываний / совершение записи / восстановление регистра SREG.
  oldSREG = SREG;
  cli();
  // Запись значения в порт с применением защитной маски.
  *portRegister = (*portRegister & port_protect[_port]) | (_value & ~port_protect[_port]);
  SREG = oldSREG;
}


/* ****************************************************************************************************************************
*
*   Pin protection testing. Refer to zabbuino.h -> port_protect[] array
*
**************************************************************************************************************************** */
boolean isSafePin(const uint8_t _pin)
{
  // Определение байта в порте, соотносящегося с заданным пином
  uint8_t result = digitalPinToBitMask(_pin);
  if (result == NOT_A_PIN) return false;
  // Проверка на "защищенность"
  result &= ~port_protect[digitalPinToPort(_pin)];
  // pinmask=B00100000, safemask=B11011100. result = B00100000 & ~B11011100 = B00100000 & B00100011 = B00100000. B00100000 > 0, pin is safe (not protected)
  // pinmask=B00100000, safemask=B11111100. result = B00100000 & ~B11111100 = B00100000 & B00000011 = B00000000. B00000000 == 0, pin is unsafe (protected)
  if (0 == result) return false; else return true;
}



