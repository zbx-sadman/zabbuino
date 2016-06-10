// Model IDs
#define DS18S20MODEL 0x10  // also DS1820
#define DS18B20MODEL 0x28
#define DS1822MODEL  0x22
#define DS1825MODEL  0x3B

// Device resolution
#define TEMP_9_BIT  0x1F //  9 bit
#define TEMP_10_BIT 0x3F // 10 bit
#define TEMP_11_BIT 0x5F // 11 bit
#define TEMP_12_BIT 0x7F // 12 bit


// OneWire commands
#define STARTCONVO      0x44  // Tells device to take a temperature reading and put it on the scratchpad
#define COPYSCRATCH     0x48  // Copy EEPROM
#define READSCRATCH     0xBE  // Read EEPROM
#define WRITESCRATCH    0x4E  // Write to EEPROM
#define RECALLSCRATCH   0xB8  // Reload from last known
#define READPOWERSUPPLY 0xB4  // Determine if device needs parasite power
#define ALARMSEARCH     0xEC  // Query bus for devices with an alarm condition


// Scratchpad locations
#define TEMP_LSB        0
#define TEMP_MSB        1
#define HIGH_ALARM_TEMP 2
#define LOW_ALARM_TEMP  3
#define CONFIGURATION   4
#define INTERNAL_BYTE   5
#define COUNT_REMAIN    6
#define COUNT_PER_C     7
#define SCRATCHPAD_CRC  8


/*
int16_t DS18X20Discovery(uint8_t _pin)
{
  uint8_t i, dsAddr[8],;

  // Создание нового объекта OneWire
  OneWire ow(_pin);
  ethClient.print("{\"data\":[");

  ow.reset_search();
  ow.reset();
  i=0;
  while (!ow.search(dsAddr))
  {
    i++;
    ethClient.print("{\"{#INDEX}\":\""); ethClient.print(i); ethClient.print("\",")
    ethClient.print("\"{#ID}\":\""); ethClient.print(dsAddr[0]); ethClient.print("\",")
    ethClient.print("\"{#ADDR}\":\""); ethClient.print(addr-to-str(dsAddr[0])); ethClient.print("\"},")
  }
  ethClient.println("]}");
}
*/

uint8_t DS18X20GetFirstID(OneWire _ow, uint8_t *_addr)
{
    _ow.reset_search();
    _ow.reset();
    return _ow.search(_addr);
}

/* ****************************************************************************************************************************
*
*   Функция считывания температуры с цифрового термометра семейства DS18x20.
*
*   С термометрами, иными нежели DS18B20 работа не проверялась. Вероятны проблемы с правильным вычислением температуры из-за
*   неправильных корректировок tRaw
*
**************************************************************************************************************************** */
int16_t DS18X20Read(uint8_t _pin, uint8_t _resolution, char* _id, char* _outBuffer)
{
  uint8_t i, signBit, dsAddr[8], scratchPad[9], parasitePowerUsed;
  int16_t conversionTimeout;
  uint32_t tRaw;

  // Приведение в порядок параметра _resolution
  _resolution = constrain(_resolution, 9, 12);

  // Создание нового объекта OneWire
  OneWire ow(_pin);


  // Перенос идентификатора (адреса) датчика из строки в массив (структура DeviceAddress)
  if ('\0' != _id[0] && '0' == _id[0] && 'x' == _id[1]) {
    _id+=2;
    for (i = 0; i < 8; i++) {
      dsAddr[i] = htod(*_id++) << 4;
      dsAddr[i] += htod(*_id++);
    }
  } else {
    if (!DS18X20GetFirstID(ow, dsAddr)) {return DEVICE_DISCONNECTED_C;};
  }

  // Проверка возможности работы с датчиком. Идентификатор модели - в первом байте адреса
  if (DS18S20MODEL != dsAddr[0] && DS18B20MODEL != dsAddr[0] && DS1822MODEL != dsAddr[0])
    return DEVICE_DISCONNECTED_C;

  // Чтение ScratchPad термометра с целью получения значений регистров CONFIGURATION, HIGH_ALARM_TEMP, LOW_ALARM_TEMP
  // В случае неудачного чтения (несовпадение контрольного кода пакета) - выход из подпрограммы
  if (!DS18X20ReadScratchPad(ow, dsAddr, scratchPad))
    return DEVICE_DISCONNECTED_C;

  // Определение схемы питания - полноценная или паразитная
  parasitePowerUsed = false;
  ow.reset();
  ow.select(dsAddr);
  ow.write(READPOWERSUPPLY);
  if (ow.read_bit() == 0) parasitePowerUsed = true;

  // From Dallas's datasheet:
  //        12 bit resolution, 750 ms conversion time
  //        11 bit res, 375 ms
  //        10 bit res, 187.5 ms
  //        09 bit res, 93.75 ms
  // conversionTimeout = (tCONV + 10%) / (2 ^ (12 [bit resolution]- N [bit resolution])). 12bit => 750ms+10%, 11bit => 375ms+10% ...
  // For some DS sensors you may need increase tCONV to 1250ms or more
  conversionTimeout = 825 / (1 << (12 - _resolution));

  // Если разрешение менять не нужно, то можно сэкономить на операции записи в ScratchPad
  if (scratchPad[CONFIGURATION] != _resolution)
  {
    // Модели DS1820 и DS18S20 не имеют регистра CONFIGURATION, установка разрешения через запись значения регистра бессмысленна.
    if (DS18B20MODEL == dsAddr[0])
    {
      switch (_resolution)
      {
        case 12:
          scratchPad[CONFIGURATION] = TEMP_12_BIT;
          break;
        case 11:
          scratchPad[CONFIGURATION] = TEMP_11_BIT;
          break;
        case 10:
          scratchPad[CONFIGURATION] = TEMP_10_BIT;
          break;
        case 9:
        default:
          scratchPad[CONFIGURATION] = TEMP_9_BIT;
          break;
      }

      ow.reset();
      ow.select(dsAddr);
      ow.write(WRITESCRATCH);
      // Записываем только три байта. Этого достаточно для смены разрешения термометра
      for ( i = HIGH_ALARM_TEMP; i <= CONFIGURATION; i++)
        ow.write(scratchPad[i]); // configuration

      // Для питания термометра по схеме с паразитным питанием необходимо скопировать установки в EEPROM. В противном случае они сбросятся при ow.reset перед конвертацией
      if (parasitePowerUsed) {
        ow.write(COPYSCRATCH, 1);
        delay(11);
      }
    }
  }

  // Запуск процедуры замера температуры
  ow.reset();
  ow.select(dsAddr);
  ow.write(STARTCONVO, 1);         // start conversion, with parasite power on at the end
  // Время ожидания окончания конвертации зависит от выбранного разрешения
  delay(conversionTimeout);

  // Чтение ScratchPad термометра с целью получения показаний температуры
  // В случае неудачного чтения (несовпадение контрольного кода пакета) - выход из подпрограммы
  if (!DS18X20ReadScratchPad(ow, dsAddr, scratchPad))
    return DEVICE_DISCONNECTED_C;

  // Вычисление температуры
  //...
  tRaw = (((int16_t)scratchPad[TEMP_MSB]) << 8) | scratchPad[TEMP_LSB];

  // Коррекция полученного с трмометра значения в зависимости от модели термометра.
  if (DS18S20MODEL == dsAddr[0])
  {
    // DS18S20 и DS1820 имеют разрешение 9 бит, незначащие биты можно сбросить
    tRaw = tRaw << 3;
    // В регистре COUNT_PER_C термометра DS18S20 жестко прошито значение 0x10. Ориентировка на него.
    if (scratchPad[COUNT_PER_C] == 0x10) {
      // Довычисление значения tRaw до 12-битного разрешения  производится через доп. регистры
      tRaw = (tRaw & 0xFFF0) + 12 - scratchPad[COUNT_REMAIN];
    }
  }
  else
  {
    // Сброс незначащих ("мусорных") битов в температуре для DS18B20 в зависимости от разрешения термометра
    switch (scratchPad[CONFIGURATION])
    {
      case TEMP_9_BIT:
        tRaw = tRaw & ~7;
        break;
      case TEMP_10_BIT:
        tRaw = tRaw & ~3;
        break;
      case TEMP_11_BIT:
        tRaw = tRaw & ~1;
        break;
      case TEMP_12_BIT:
      default:
        break;
    }
  }

  //  ethClient.print("tRaw: "); ethClient.println(tRaw);
  // Проверка бита знака (бит 15). Если он установлен, температура отрицательна
  // if (signBit) ...
  signBit = false;
  if (tRaw & 0x8000)
  {
    signBit = true;
    // Необходима небольшая магия для последующего правильно вычисления температуры
    // См. DS18B20's datasheet,  Table 1. Temperature/Data Relationship
    tRaw = (tRaw ^ 0xffff) + 1; // 2's comp
    //    strcat(tStr, "-");
  }

  // Полученное значение должно быть умножено на 0.0625 (1/16 C)
  tRaw *= 100;
  tRaw = ((tRaw * 6) + (tRaw / 4));

  if (signBit) {
    tRaw = -tRaw;
  }

  ltoaf(tRaw, _outBuffer, 4);


  return RESULT_IN_BUFFER;
}

uint8_t DS18X20ReadScratchPad(OneWire _ow, uint8_t *_addr, uint8_t *_scratchPad)
{
  uint8_t i;
  _ow.reset();
  _ow.select(_addr);
  _ow.write(READSCRATCH);
  for ( i = 0; i < 9; i++) {
    _scratchPad[i] = _ow.read();
  }
  // CRC checking and return result
  return (dallas_crc8(_scratchPad, 8) == _scratchPad[SCRATCHPAD_CRC]);

}

