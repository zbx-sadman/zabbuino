/*
 Zabbix Agent для Arduino.
 Поддерживает протокол Zabbix2, Ethernet-модули WizNet и ENC28J60.
 Основан на коде Evgeny Levkov, Schotte Vincent.
 Prigodin Grigory (sadman@sfi.comi.com)

v 0.9999 (20 May 2015)
         В команде получения температуры с датчика DS18x20 (DS18x20.temperature[]) появилась возможность задавать точность замера через параметр resolution (9-12bit).
         Достигнуто паспортное значение точности - 1/16 С. Введена функция поиска датчика при при отсутствии заданного идентификатора.
         Добавлена поддержка датчиков DHT11/DHT21/DHT22/AM2301/AM2302 и аналогичных. Команды: DHT.Temperature[], DHT.Humidity[]. Точность замера паспортная - 0.1 ед.
         Добавлена поддержка датчиков BMP085/180. Комнды BMP.Temperature[], BMP.Pressure[]. Точность замера паспортная.

v 0.999 (14 May 2015)
         Введена возможность подключения блоков команд через определение соответствующих макросов
         Переписаны функции, связанные с командой shiftOut[]. Теперь вывод происходит путем прямого манипулирования портами.
         Добавлена поддержка цифровых термометров Dallas DS18x20.
         Реализована установка состояния пина при инициализации микроконтроллера.
         Испытывалась на Freeduino 2009 (ATmega 328) с Ethernet-модулем W5100.
         Опытная эксплуатация показала невозможность использования ENC28J60 в долговременной перспективе без применения функций watchdog совместно с бутлоадером optiboot.

 v 0.99 (07 May 2015)
         Первая публичная версия
         Испытывалась на Freeduino 2009 (ATmega 168) с Ethernet-модулем W5100 и Deek-Robot Arduino Mini Pro clone (ATmega 328) с Ethernet-модулем ENC28J60
*/

//#define ENC28J60

// tone[], noTone[]
#define TONE_COMMANDS_ENABLE
// randomSeed, random[]
#define RND_COMMANDS_ENABLE
// agent.cmdCount, agent.cmdStr, sys.freeRAM
#define DEBUG_COMMANDS_ENABLE
// shiftOut[]
#define SHIFTOUT_COMMANDS_ENABLE
// DS18x20.Temperature[]
#define DS18X20_COMMANDS_ENABLE

#define DHT_COMMANDS_ENABLE
#define BMP085_COMMANDS_ENABLE

//#define DEBUG_MSG_TO_ETHCLIENT

// Библиотека для модуля ENC28J60
// Испытано с UIPEthernet version 1.09
#include <UIPEthernet.h>
// Стандартная библиотека Arduino Ethernet для использования с модулями, построенными на базе WizNet W5100
//#include <SPI.h>
//#include <Ethernet.h>

// Wire (I2C) для BMP085/BMP180
#include <Wire.h>
// OneWire для DS18x20
#include <OneWire.h>

#define SENS_READ_TEMP 0x00
#define SENS_READ_HUMD 0x01
#define SENS_READ_PRSS 0x02

#define RESULT_IS_FAIL    -0xFFAL
#define RESULT_IS_OK      -0xFFBL
#define RESULT_IN_BUFFER  -0xFFCL
#define RESULT_IS_PRINTED -0xFFDL

// Error Codes
#define DEVICE_DISCONNECTED_C -127
#define DEVICE_DISCONNECTED_F -196.6
#define DEVICE_DISCONNECTED_RAW -7040


/*
  Аналоговые пины (A0..A7) могут быть использованны в некоторых функциях, подразумевающих использование цифровых пинов (D0..Dn).
  Их номера для определенной платформы содержатся в заголовочном файле ..\hardware\arduino\avr\variants\___платформа___\pins_arduino.h
  Например, для стандартной Arduino соответствие выглядит как: A0 = 14 .. A7 = 21;
  Соответственно, вызов функции digitalWrite(A0)можно записать как digitalWrite(14)
*/

/*
  Количество портов (не пинов!) ввода/вывода на плате. Порты обозначаются как PORTB, PORTC, PORTD, PORTE...
  Их число также можно найти в файле ..\hardware\arduino\avr\variants\___платформа___\pins_arduino.h. См. массив array port_to_mode_PGM
*/
#define PORTS_NUM 5

/*
  Защитные маски портов ввода/вывода.
  Значение '1' в определенной битовой позиции означает то, что при операциях записи в порт данный бит не будет изменен (находится под защитой). Эта маски
  также применяются для проверки безопасности при операциях изменения состояния определенного пина. Функция isSafePin() проверяет не установлена ли защита
  для конкретного пина.

  Вы можете изменять как сами маски, так и их количество. Однако, количество элементов данного массива должно соответствовать параметру PORTS_NUM.

  Например: Необходимо защитить от изменения pin D13, так как он используется библиотекой Ethernet и его изменение извне приведет к некорректной работе
  сетевого адаптера. В заголовочном файле pins_arduino.h определенном для необходимой платформы находим массив digital_pin_to_bit_mask_PGM. Элемент #13 указывает на
  связь данного пина с портом B (PORTB) и битом 5. Значит, для защиты данного пина необходимо установить в нижележащем массиве port_protect 5-й символ справа
  в строке "B..... // PORTB" в значение '1'.
*/

const byte port_protect[PORTS_NUM] = {
  B00000000, // not a port
  B00000000, // not a port
  // Защита установлена для битов 2, 3, 4, 5 (пины D10, D11, D12, D13), так как они используются SPI (ethernet shield)
  // и для битов 6, 7 потому что они не используются в Arduino Pro / Freeduino 2009
  B11111100, // PORTB (D8-D13)
  B00000000, // PORTC (A0-A7)
  // Защита установлена для битов 0,1 (пины D0, D1) в связи с тем, что они обслуживают линии RX, TX и нужны для отладки.
  B00000011  // PORTD (D0-D7)
};

/*
   Маски для задания направления ввода/вывода выходов микроконтроллера.
   Значение '1' в определенной битовой позиции означает то, что при инициализации соответствующий биту пин будет установлен в состояние OUTPUT.
   В противном случае он будет оставлен с состоянии по умолчанию для платформы, на которой выполняется программный код.

   Вы можете изменять как сами маски, так и их количество. Однако, количество элементов данного массива должно соответствовать параметру PORTS_NUM.

   Например: Необходимо при инициализации установить пин D8 в режим OUTPUT, а пин D9 оставить в состоянии по умолчанию - INPUT.
   В заголовочном файле pins_arduino.h определенном для необходимой платформы находим массив digital_pin_to_bit_mask_PGM. Элемент #8 связан с битом 0 порта B,
   а элемент #9 с битом 1 того же порта. Значит, для правильной инициализации следует установить в нижележащем массиве port_mode 0-й символ справа
   в строке "B..... // PORTB" в значение '1', а 1-й символ справа в значение '0'

   Будьте внимательны и осторожны. Установка пина в состояние INPUT увеличивает при неаккуратном обращении с устройством вероятность вывода из строя
   соответствующего вывода микроконтроллера.
*/

const byte port_mode[PORTS_NUM] = {
  B00000000, // not a port
  B00000000, // not a port
  // Все пины установлены в режим OUTPUT (см. так же описание массива port_protect[..])
  B11111111, // PORTB (D8-D13)
  B11111110, // PORTC (A0-A7)
  B11111111  // PORTD (D0-D7)
};

/*
  Маски для установки состояния выходов микроконтроллера.
  Значение '1' в определенной битовой позиции задает высокое состояние пина при инициализации (см. описание функции Arduino pinMode()). Если маской port_mode
  соответствующий пин определен, как OUTPUT, то его итоговое состояние станет OUTPUT+HIGH. В случае с определением пина, как работающего в режиме INPUT, итоговым
  состоянием будет INPUT_PULLUP.

  Вы можете изменять как сами маски, так и их количество. Однако, количество элементов данного массива должно соответствовать параметру PORTS_NUM.
  Вычисление битов, соотвующих пинам аналогично способам, применяемым в port_protect и port_mode.
*/
const byte port_pullup[PORTS_NUM] = {
  B00000000, // not a port
  B00000000, // not a port
  B00000000, // PORTB (D8-D13)
  B00000000, // PORTC (A0-A7)
  B00000000  // PORTD (D0-D7)
};

#define CPUTEMP_CORRECTION 32431

// Количество ожидаемых аргументов
#define ARGS_NUM 6

// Размер буфера для всех аргументов команды.
#define ARGS_SIZE 50
// Размер буфера для команды
#define CMD_SIZE 25
// Общий размер буфера - команды и аргументов. +1 => количество разделителей аргументов=кол-во аргументов-1, + '[' +']'
#define BUFFER_SIZE CMD_SIZE+ARGS_SIZE+ARGS_NUM+1

// Префикс заголовка запроса Zabbix сервера v2.x - 'ZBXD\1'.
// Для функции определения заголовка необходимо использовать длину заголовка == length('ZBXD\1')-1
#define ZBX_HEADER_PREFIX_LENGTH 4
// Длина всего заголовка запроса Zabbix сервера v2.x.
#define ZBX_HEADER_LENGTH 12

// Если директива ON_ALARM_STATE_BLINK определена, то при аварийной ситуации светодиод будет мигать.
#define ON_ALARM_STATE_BLINK 1

// FDQN устройства. Отдается по команде agent.hostname
#define ZBX_HOSTNAME "zabbuino.local.net"
// Версия агента. Отдается по команде agent.version
#define ZBX_AGENT_VERISON "Zabbuino 0.9999"
// Сообщение "Не поддерживается". Отдается в случае, если команда не была опознана
#define ZBX_NOTSUPPORTED_MSG "ZBX_NOTSUPPORTED"

// Таймаут состояния покоя. 60 sec
#define IDLE_TIMEOUT 60000UL

// Буфер принятой команды
char cmd[CMD_SIZE];
// "Массив" аргументов принятой команды
char* arg[ARGS_NUM];

// if buffsize >255 then parce cmdline is buggy
// Рабочий буфер.
char cBuffer[BUFFER_SIZE];

/* 
   Пин, к которому подключено устройство сигнализации состояния. Не используйте в качестве этого устройства светодиод, расположенный на плате Arduino.
   Пин D13, к которому присоединен данный светодиод, используется SPI (SPI pins is: 10 (SS), 11 (MOSI), 12 (MISO), __13__ (SCK))
*/

const byte stateLedPin = 8;

unsigned int bufferWritePosition;
boolean clntIsConnected, needSkipZabbix2Header;
unsigned long prevExecuteTime, prevConnectTime, cmdCounter;

//  arg2 = atoi(arg[2]); // frequency in tone[] - uint
//  arg3 = atol(arg[3]); // duration  in tone[] - ulong

EthernetServer ethServer(10050);
EthernetClient ethClient;

void setup() {
  byte i;
  // Сетевые настройки устройства
  IPAddress ip(192, 168, 0, 2);
  IPAddress gateway(192, 168, 0, 1);
  IPAddress subnet(255, 255, 255, 0);
  // MAC-адрес устройства
  byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

  /* Инициализация элементов массива аргументов производится через конструкцию itemN[]="0\0\...", так как это занимает меньше программного пространства
     контроллера, нежели при использовании других методов: arg0=char[..] & arg[0]=arg0 или malloc().
     Не стоит использовать одинаковые значения для всех аргументов. Компилятор производит излишнюю оптимизацияю и все элементы начинают указывать на
     один адрес памяти.
  */
  arg[0] = cmd;
  arg[1] = "\0\1";
  arg[2] = "\0\0\0\0\2";
  // max len = 16 is ID arg of DS18B20Read[] command
  arg[3] = "\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\3";
  arg[4] = "\0\4";
  arg[5] = cBuffer;
  // Инициализация портов ввода/вывода - установка режима работы.
  for (i = 0; i < PORTS_NUM; i++) setPortMode(i, port_mode[i], port_pullup[i]);

  // Запуск Ethernet сервера.
  Ethernet.begin(mac, ip, gateway, subnet);
  ethServer.begin();
}

void loop() {
  unsigned long nowTime;

  nowTime = millis();

  // Если давно не выполняли команд - сигнализируем об этом.
  if (nowTime - prevExecuteTime >= IDLE_TIMEOUT )
  {
#ifdef ON_ALARM_STATE_BLINK
    // Светодиод мигает
    digitalWrite(stateLedPin, nowTime % 1000 < 500);
#else
    // Или просто горит
    digitalWrite(stateLedPin, HIGH);
#endif
  }

  ethClient = ethServer.available();
  if (ethClient) {
    // В оригинальном коде использован закомментированный фрагмент, но в моем случае все работает корректно и без него.
    // При использовании ENC28J60 .flush() дропает часть входящего сообщения, W5100 обрабатывает его корректно (или просто игнорирует вызов функции)
    //    if (!clntIsConnected) {
    //      ethClient.flush();
    //      clntIsConnected = true;
    //      prevConnectTime=nowTime;
    //    }
    // Read bytes from clients buffer
    if (ethClient.available()) {
      analyzeStream(ethClient.read());
    }
  }
}

/* ****************************************************************************************************************************
*
*  Функция выполнения принятой команды
*  Проверяет аргументы, вызывает соответствующую команде подпрограмму
*  Все используемые переменные глобальны.
*
**************************************************************************************************************************** */
void executeCommand()
{
  boolean latchPinDefined;
  byte arg1, arg4, arg5;
  unsigned int arg2;
  long result = RESULT_IN_BUFFER;
  unsigned long arg3;

#ifdef DEBUG_MSG_TO_ETHCLIENT
  ethClient.print("executeCommand: "); ethClient.println(cmd);
  ethClient.print("cmdCounter: "); ethClient.println(cmdCounter);
  ethClient.print("buffer: "); ethClient.println(cBuffer);
#endif

  // Приращение счетчика принятых команд
  cmdCounter++;

  // Конвертация C-string в int-переменные
  // arg[0] содержит выделенную команду: arg[0] = cmd;
  arg1 = atoi(arg[1]);
  arg2 = atoi(arg[2]); // frequency in tone[] - uint
  arg3 = atol(arg[3]); // duration  in tone[] - ulong
  arg4 = atoi(arg[4]);
  arg5 = atoi(arg[5]);


  // Для защелкивания сдвигового регистра перед использованием команды shiftout значение пина latch должно быть определено.
  // В противном случае защелкивания сдвигового регистра не производится.
  latchPinDefined = false;
  if (arg[3][0] != '\0' && isSafePin(arg2)) {
    latchPinDefined = true;
  }

  // Анализ команды. Вывод текста, если таковой подразумевается результатом работы, происходит в рабочий буфер, который в конце пересылается клиенту.

  if (strcmp(cmd, "agent.ping") == 0) {
    // Команда: agent.ping
    // Параметры: не требуются
    // Результат: возвращается значение '1'
    result = RESULT_IS_OK;

  } else if (strcmp(cmd, "agent.hostname") == 0) {
    // Команда: agent.hostname
    // Параметры: не требуются
    // Результат: возвращается имя узла
    strcpy(cBuffer, ZBX_HOSTNAME);

  } else if (strcmp(cmd, "agent.version") == 0) {
    // Команда: agent.version
    // Параметры: не требуются
    // Результат: возвращается версия агента
    strcpy(cBuffer, ZBX_AGENT_VERISON);

#ifdef DEBUG_COMMANDS_ENABLE
  } else if (strcmp(cmd, "agent.cmdcount") == 0) {
    // Команда: agent.cmdCount
    // Параметры: не требуются
    // Результат: возвращается количество обработанных команд
    result = cmdCounter;

  } else if (strcmp(cmd, "agent.cmdstr") == 0) {
    // Команда: agent.cmdStr
    // Параметры: не требуются.
    // Результат: возвращается принятая команда.
    // Примечание: используется в целях отладки
    // Используется в целях отладки. Никакого действия предпринимать не требуется,
    // так как выводимое значение уже помещено в рабочий буфер функцией parseBuffer();
    ;

  } else if (strcmp(cmd, "sys.freeram") == 0) {
    // Команда: sys.freeRAM
    // Параметры: не требуются.
    // Результат: возвращается объем свободной оперативной памяти контроллера.
    result = (long) freeRam();
#endif

    // Need to unFloat routine for this sub
    //  } else if (strcmp(cmd, "sys.cputemp") == 0) {
    // Команда: sys.cpuTemp
    // Параметры: не требуются.
    // Результат: возвращается показания внутреннего датчика температуры микроконтроллера
    // http://playground.arduino.cc/Main/InternalTemperatureSensor
    // ....
    // ATmega168 : No
    // ATmega168P : Yes
    // ATmega328P : Yes
    // ATmega1280 (Arduino Mega) : No
    // ....
    //     result=(long) getInternalTemperature();

  } else if (strcmp(cmd, "portwrite") == 0) {
    // Команда: portWrite[port, value]
    // Параметры: port - символьное обозначение порта (B,C,D..),
    //            value - значение, которое требуется записать в заданный порт ввода/вывода
    // Результат: изменяется состояние порта ввода/вывода (PORTB, PORTC, PORTD...) и происходит возврат значения '1'.
    // Примечание: если ваш экземпляр Arduino имеет более, чем три порта, то на данный момент вам необходимо самостоятельно добавить в скетч информацию о них.
    //
    // Номер порта представляет собой разницу между ASCII-кодом аргумента port и 96. Таким образом b=2, c=3, d=4 и т.д.
    portWrite((byte) *arg[1] - 96, arg2);
    result = RESULT_IS_OK;

  } else if (strcmp(cmd, "analogwrite") == 0) {
    // Команда: analogWrite[pin, value]
    // Параметры: pin - цифровое обозначение пина, value - значение скважности, которое требуется задать для данного пина.
    // Результат: изменяется скважности PWM для пина. Удачное выполнение команды влечет за собой возврат значения '1', неудачное - значения '0'.
    // Примечание: команда является оберткой функции analogWrite() http://www.arduino.cc/en/Reference/AnalogWrite
    // Состояние OUTPUT для пина должно быть задано в коде скетча. Если пин защищен, изменения режима не происходит. Если пин не является PWM-совместимым, на нем выставляется значение HIGH.
    // Внимание! Функция analogWrite() самостоятельно устанавливает пин в режим работы OUTPUT
    result = RESULT_IS_FAIL;
    if (isSafePin(arg1)) {
      analogWrite(arg1, arg2);
      result = RESULT_IS_OK;
    }

  } else if (strcmp(cmd, "analogread") == 0) {
    // Команда: analogread[pin]
    // Параметры: pin - цифровое обозначение пина
    // Результат: возврат величины, "считанной" с пина. Диапазон значений 0...1023 (возможные варианты диапазона значений зависят от способа подключения сигнала к пину и внутренних настроек Arduino)
    // Примечание: команда является оберткой функции analogRead() www.arduino.cc/en/Reference/AnalogRead
    // Данная команда имеет смысл только для аналоговых пинов.
    // Состояние INPUT для пина должно быть определено в коде скетча. В противном случае совпадения считываемых данных с ожидаемыми может не произойти.    result = (long) analogRead(arg1);

  } else if (strcmp(cmd, "analogreference") == 0) {
    // Команда: analogReference[source]
    // Параметры: source - источник опорного напряжения (0..N). Значения можно найти в заголовочном файле Arduino.h
    // Результат: устанавливается источник опорного напряжения относительно которого происходят аналоговые измерения и происходит возврат значения '1'
    // Примечание: команда является оберткой функции analogReference() www.arduino.cc/en/Reference/AnalogReference
    analogReference(arg1);
    result = RESULT_IS_OK;

  } else if (strcmp(cmd, "digitalwrite") == 0) {
    // Команда: digitalWrite[pin, value]
    // Параметры: pin - цифровое обозначение пина, value - значение, которое требуется выставить на заданном пине.
    // Результат: изменяется состояние пина. Удачное выполнение команды влечет за собой возврат значения '1', неудачное - значения '0'.
    // Примечание: команда является оберткой функции digitalWrite() www.arduino.cc/en/Reference/DigitalWrite
    // Состояние OUTPUT для пина должно быть задано в коде скетча. Если пин защищен, изменения режима не происходит.    result = RESULT_IS_FAIL;
    if (isSafePin(arg1)) {
      digitalWrite(arg1, arg2);
      result = RESULT_IS_OK;
    }

  } else if (strcmp(cmd, "digitalread") == 0) {
    // Команда: digitalRead[pin]
    // Параметры: pin - цифровое обозначение пина
    // Результат: возвращается значение, "считанное" с пина. Диапазон значений - HIGH/LOW.
    // Примечание: команда является оберткой функции DigitalRead() http://www.arduino.cc/en/Reference/DigitalRead
    // Состояние INPUT для пина должно быть определено в коде скетча. В противном случае совпадения считываемых данных с ожидаемыми может не произойти.
    result = (long) digitalRead(arg1);

#ifdef TONE_COMMANDS_ENABLE
  } else if (strcmp(cmd, "tone") == 0) {
    // Команда: tone[pin, frequency, duration]
    // Параметры: pin - цифровое обозначение пина, frequency - частота сигнала, duration - длительность сигнала
    // Результат: начинается генерация на указанном пине сигнала "прямоугольная волна" заданной частоты.
    // Примечание: команда является оберткой функции tone() http://www.arduino.cc/en/Reference/Tone
    // Состояние OUTPUT для пина должно быть задано в коде скетча. Если пин защищен, изменения режима не происходит.
    result = RESULT_IS_FAIL;
    if (isSafePin(arg1)) {
      tone(arg1, arg2, arg3);
      result = RESULT_IS_OK;
    }

  } else if (strcmp(cmd, "notone") == 0) {
    // Команда: noTone[pin]
    // Параметры: pin - цифровое обозначение пина
    // Результат: завершается генерация на указанном пине сигнала "прямоугольная волна"
    // Примечание: команда является оберткой функции noTone() http://www.arduino.cc/en/Reference/NoTone
    // Состояние OUTPUT для пина должно быть задано в коде скетча. Если пин защищен, изменения режима не происходит.
    result = RESULT_IS_FAIL;
    if (isSafePin(arg1)) {
      noTone(arg1);
      result = RESULT_IS_OK;
    }
#endif

#ifdef RND_COMMANDS_ENABLE
  } else if (strcmp(cmd, "randomseed") == 0) {
    // Команда: randomSeed[value]
    // Параметры: value - начальное число ряда псевдослучайных значений
    // Результат: инициализируется генератор псевдослучайных чисел
    // Примечание: команда является оберткой функции randomSeed() http://www.arduino.cc/en/Reference/randomSeed
    randomSeed(arg1);
    result = RESULT_IS_OK;

  } else if (strcmp(cmd, "random") == 0) {
    // Команда: random[min, max]
    // Параметры: min, max - нижняя и верхняя границы псевдослучайных значений
    // Результат: возвращается псевдослучайное число
    // Примечание: команда является оберткой функции random() http://www.arduino.cc/en/Reference/random
    //  !! random return long
    result = (long) random(arg1, arg2);
#endif

#ifdef SHIFTOUT_COMMANDS_ENABLE
  } else if (strcmp(cmd, "shiftout") == 0) {
    // Команда: shiftOut[dataPin, clockPin, latchPin, bitOrder, value]
    // Параметры: dataPin, clockPin, latchPin - цифровое обозначения пинов вывода данных, синхронизации, защелкивания.
    //            bitOrder - последовательность вывода бит, value значение для вывода.
    // Результат: устанавливается соответствующее параметру value состояние выводов подключенного сдвигового регистра.
    //            Удачное выполнение команды влечет за собой возврат значения `1`, неудачное - значения '0'.
    // Примечание: команда является расширением функции shiftOut().
    //            Параметр value может быть задан как в десятичной и шестнадцатеричной форме (с префиксом 0x).
    //            Длина числа в шестнадцатеричной форме ограничена размером внутреннего буфера.
    // Состояние OUTPUT для пинов должно быть задано в коде скетча. Если пины защищены, вызова соотвествующих функций не происходит.
    result = RESULT_IS_FAIL;
    if (isSafePin(arg1) && isSafePin(arg2))
    {
      if (latchPinDefined) {
        digitalWrite(arg3, LOW);
      }
      advShiftOut(arg1, arg2, arg4, cBuffer);
      if (latchPinDefined) {
        digitalWrite(arg3, HIGH);
      }
      result = RESULT_IS_OK;
    }
#endif

#ifdef DS18X20_COMMANDS_ENABLE
  } else if (strcmp(cmd, "ds18x20.temperature") == 0) {
    // Команда: DS18x20.Temperature[pin, resolution, id]
    // Параметры: pin - цифровое обозначение пина, к которому подключен цифровой термометр DS18x20. resolution - разрешение термометра 9..12бит,
    //            id - идентификатор (адрес) термометра.
    // Результат: с цифрового термометра считывается температура и значение в градусах Цельсия возвращается пользователю.
    // Примечание: Точность показаний (1/2 ... 1/16 C) зависит от параметра resolution, от него, также зависит время выполнения команды.
    //             Максимальный временной промежуток - 825ms (resolution = 12bit).
    //             Идентификатор (адрес) термометра можно получить через Serial Monitor при выполнении скетча DallasTemperature -> Single.
    //             Значение -127 выдается при какой-либо ошибке в функции - невозможности считать данные с термометра вследствии ошибки подсоединения или ошибочно указанном ID.
    //             Так же это значение выдается при попытке обращения к термометру неподдерживаемой модели.
    if (isSafePin(arg1)) {
      result = DS18X20Read(arg1, arg2, arg[3], cBuffer);
    }
#endif

#ifdef DHT_COMMANDS_ENABLE
  } else if (strcmp(cmd, "dht.temperature") == 0) {
    // Команда: DHT.Temperature[pin, model]
    // Параметры: pin - цифровое обозначение пина, к которому подключен цифровой датчик DHT/AM/..
    //            model - идентификатор модели датчика - 11 (DHT11), 21 (DHT21, AM2301), 22 (DHT22, AM2302).
    // Результат: с цифрового датчика DHT11/21/22 считывается температура и значение в градусах Цельсия возвращается пользователю.
    // Примечание: Значение -127 выдается при какой-либо ошибке - например несовпадении CRC.
    //             Если модель датчика не указана или указана неверно, то расчет температуры производится по формуле, применяемой для DHT22.
    //             Команда самостоятельно устанавливает состояние INPUT/OUTPUT пина. В целях безопасности стоит инициализировать пин как OUTPUT.
    if (isSafePin(arg1)) {
      result = DHTRead(arg1, arg2, SENS_READ_TEMP, cBuffer);
    }

  } else if (strcmp(cmd, "dht.humidity") == 0) {
    // Команда: DHT.Humidity[pin, model]
    // Параметры: pin - цифровое обозначение пина, к которому подключен цифровой датчик DHT/AM/..
    //            model - идентификатор модели датчика - 11 (DHT11), 21 (DHT21, AM2301), 22 (DHT22, AM2302).
    // Результат: с цифрового датчика считывается величина влажности и значение в процентах возвращается пользователю.
    // Примечание: Значение -127 выдается при какой-либо ошибке - например несовпадении CRC.
    //             Если модель датчика не указана или указана неверно, то расчет величины влажности производится по формуле, применяемой для DHT22.
    //             Команда самостоятельно устанавливает состояние INPUT/OUTPUT пина. В целях безопасности стоит инициализировать пин как OUTPUT.
    if (isSafePin(arg1)) {
      result = DHTRead(arg1, arg2, SENS_READ_HUMD, cBuffer);
    }
#endif

#ifdef BMP085_COMMANDS_ENABLE
  } else if (strcmp(cmd, "bmp.temperature") == 0) {
    // Команда: BMP.Temperature[sdaPin, sclPin]
    // Параметры: sdaPin, sclPin - цифровые обозначение пинов, к которым подключена шина I2C.
    // Результат: с цифрового датчика BMP085/BMP180 считывается температура и значение в градусах цельсия возвращается пользователю.
    // Примечание: Точность датчика - 0,1C.
    //             sdaPin, sclPin на данный момент не применяются (используются стандартные пины для I2C подключения) и зарезервированы для внедрения SoftTWI интерфейсов.
    //if (isSafePin(arg1) && isSafePin(arg2)) {
      result = BMP085Read(arg1, arg2, arg3, SENS_READ_TEMP, cBuffer);
    //}

  } else if (strcmp(cmd, "bmp.pressure") == 0) {
    // Команда: BMP.Pressure[sdaPin, sclPin, overSampling]
    // Параметры: sdaPin, sclPin - цифровые обозначение пинов, к которым подключена шина I2C.
    //            overSampling - значение, определяющее точность и длительность измерения.
    //            0 - ultra low power, RMS noise = 6Pa, conversion time = 4,5ms ... 3 - ultra high resolution, RMS noise = 3Pa, conversion time = 25,5ms
    // Результат: с цифрового датчика считывается величина атмосферного давления и значение в Паскалях возвращается пользователю.
    // Примечание: sdaPin, sclPin на данный момент не применяются (используются стандартные пины для I2C подключения) и зарезервированы для внедрения SoftTWI интерфейсов.
    //  if (isSafePin(arg1) && isSafePin(arg2)) {
      result = BMP085Read(arg1, arg2, arg3, SENS_READ_PRSS, cBuffer);
    //  }
#endif

  } else {
    // В любом ином случае команда считается неопределенной.
    strcpy(cBuffer, ZBX_NOTSUPPORTED_MSG);
    // Прирощенный ранее счетчик сматывается
    cmdCounter--;
  }

  //  ethClient.println(result);
  // Результат уже выведен исполняемой командой?
  if  (RESULT_IS_PRINTED != result)
  {
    // Результат помещен в буфер заранее?
    if (RESULT_IN_BUFFER != result) {
      //  Необходимо возвратить '1'
      if (RESULT_IS_OK == result) {
        result = 1L;
        // или '0'
      } else if (RESULT_IS_FAIL == result) {
        result = 0L;
      }
      //  Если результатом работы команды является число, оно преобразуется в C-string.
      ltoa (result, cBuffer, 10);
    }
    //  Буфер отдается клиенту
    ethClient.println(cBuffer);
  }

}


/* ****************************************************************************************************************************
*
*  Функция разбора буфера.
*  Извлекает из буфера команду, аргументы.
*  Все используемые переменные глобальны.
*
**************************************************************************************************************************** */
void parseBuffer()
{
  char currChar;
  byte argPointer, nByteCounter;
  unsigned int bufferWritePosition, bufferReadPosition;

  bufferReadPosition = 0;
  bufferWritePosition = 0;
  argPointer = 0;

  // Need to find way to calculate size of arg[x] and dont increase bufferWritePosition if its = sizeof
  // Основной цикл.
  // В нем для анализа используется буфер, являющийся последним аргументом.
  // Все прочтенные из него символы, являющиеся допустимымы, пишутся в начало этого же самого буфера, а после обнаружения разделителя, фрагмент буфера [0...n]
  // копируется в соотв. элемент массива аргументов. Так достигается экономия оперативной памяти.
  do
  {
    // Получение очередного символа из рабочего буфера, перевод в нижний регистр
    currChar = (char) tolower(cBuffer[bufferReadPosition]);
    cBuffer[bufferReadPosition] = currChar;

    // Проверка на предмет соответствия символа знакам-разделителям или признаку конца буфера ('\0')
    if ('[' == currChar || ',' == currChar  || ']' == currChar || !currChar) {
      // Копирование найденного фрагмента в соотв. элемент массива аргументов
      copyChars(arg[argPointer], cBuffer, bufferReadPosition - bufferWritePosition, bufferWritePosition);
      // Формирование C-строки
      arg[argPointer][bufferWritePosition] = '\0';

      // ethClient.print("arg[");ethClient.print(argPointer);ethClient.print("]: "); ethClient.println(arg[argPointer]);
      // Запись в рабочий буфер начинается с начала
      bufferWritePosition = 0;
      argPointer++;

    } else if (0x20 != currChar) {
      // Если элемент не является пробелом - указатель записи в буфер должен быть приращен.
      // Таким образом пробелы исключаются из финальных данных
      bufferWritePosition++;
    }
    // Приращение указателя чтения из буфера.
    bufferReadPosition++;
    // Цикл выполняется до достижения финального ограничителя - ']' или конца буфера - '\0' или до исчерпания массива аргументов.
  } while (currChar && currChar != ']' && ARGS_NUM > argPointer);
}


/* ****************************************************************************************************************************
*
*  Функция анализа потока данных.
*  Помещает байты считанные из буфера Ethernet, в рабочий буфер, принимает решение о типе пакета,
*  инициирует выполнение принятой команды
*
**************************************************************************************************************************** */
void analyzeStream(char charFromClient) {

  // Заполнение буфера принятым элементом
  cBuffer[bufferWritePosition] = charFromClient;

#ifdef DEBUG_MSG_TO_ETHCLIENT
  ethClient.println(cBuffer[bufferWritePosition], HEX);
#endif

  // Проверка на наличие префикса заголовка  == "ZXBD"
  if (bufferWritePosition == ZBX_HEADER_PREFIX_LENGTH && strcmp(cBuffer, "ZBXD\01") == 0)
  { // Если префикс означает пакет Zabbix2, необходимо пропустить весь заголовок

#ifdef DEBUG_MSG_TO_ETHCLIENT
    ethClient.println("zbxd detected");
#endif

    needSkipZabbix2Header = true;
  }

  // Заголовок опускается
  if (needSkipZabbix2Header && bufferWritePosition >= ZBX_HEADER_LENGTH)
  { // Запись в буфер начинается с начала

#ifdef DEBUG_MSG_TO_ETHCLIENT
    ethClient.println("header dropped");
#endif

    bufferWritePosition = 0;
    needSkipZabbix2Header = false;
    // 'return' here save a lot cpu time
    return;
  }

  // Обнаружен конец строки (пакета)
  if (charFromClient == '\n' && !needSkipZabbix2Header) {
    // Формируется C-строка
    cBuffer[bufferWritePosition] = '\0';
    // Сигнализируется начало обработки команды
    digitalWrite(stateLedPin, HIGH);

#ifdef DEBUG_MSG_TO_ETHCLIENT
    ethClient.println("EOL catched");
#endif

    // Вызов подпрограммы разбора буфера. Все необходимые ей переменные глобальны.
    parseBuffer();
    // Вызов подпрограммы выполнения команды. Все необходимые ей переменные глобальны.
    executeCommand();

    prevExecuteTime = millis();
    // Соединение закрывается, все переменные переинициализируются
    ethClient.stop();
    cleanVars();
  }
  else if (bufferWritePosition <= BUFFER_SIZE) {
    // Если анализ не закончен и последний элементы буфера не достигнут - продвигаем указатель записи.
    bufferWritePosition++;
  }
}


/* ****************************************************************************************************************************
*
*   Функция переинициализации переменных при завершении сеанса.
*
**************************************************************************************************************************** */
void cleanVars()
{
  byte i;

  // Зачистка элементов массива аргументов путем формирования пустых C-строк.
  for (i = 0; i < ARGS_NUM; i++) arg[i][0] = '\0';
  //  bufferReadPosition = 0;  argPointer = 0;
  bufferWritePosition = 0;
  needSkipZabbix2Header = false;
  //  clntIsConnected = false;
  // Для корректной работы операции сравнения (strcmp(cBuffer, "ZBXD\01") == 0) необходимо, чтобы фрагмент буфера представлял собой C-строку в ZBX_HEADER_PREFIX_LENGTH символов
  cBuffer[ZBX_HEADER_PREFIX_LENGTH + 1] = '\0';
  // Сигнал обработки команды отключается
  digitalWrite(stateLedPin, LOW);
}



