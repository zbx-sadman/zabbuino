#pragma once


#define MODBUS_PACKET_SLAVE_ID                   (0x00)
#define MODBUS_PACKET_FUNCTION_CODE              (0x01)
#define MODBUS_PACKET_QUERY_REGISTER_ADDRESS     (0x02)
#define MODBUS_PACKET_QUERY_REGISTERS_COUNT      (0x04)
#define MODBUS_PACKET_QUERY_CRC                  (0x06)
#define MODBUS_PACKET_QUERY_LENGTH               (0x08)
#define MODBUS_PACKET_QUERY_FRAME_LENGTH         (0x06) // 1 (SlaveId) + 1 (FC) + 2 (1-st register address) + 2 (CRC)

#define MODBUS_PACKET_ANSWER_HEADER_LENGTH       (0x03) // 1 (SlaveId) + 1 (FC) + 1 (BytesCount)
#define MODBUS_PACKET_ANSWER_FRAME_LENGTH        (0x05) // 1 (SlaveId) + 1 (FC) + 1 (BytesCount) + 2 (CRC)

#define MODBUS_PACKET_ANSWER_EXCEPTION_CODE      (0x02) // Exception code placed into "Bytes count" field on error
#define MODBUS_PACKET_ANSWER_DATA_BYTES_COUNT    (0x02)

#define MODBUS_TRANSACTION_TIMEOUT               (1000UL)
#define MODBUS_BITS_PER_SYMBOL                   (11) // 1 (start) + 8 (data) + 1 (parity) + 1 (stop)
#define MODBUS_FUNCTION_CODE_ERROR_MASK          (0x80)

#define MODBUS_FUNCTION_CODE_01                  (0x01)
#define MODBUS_FUNCTION_CODE_02                  (0x02)
#define MODBUS_FUNCTION_CODE_03                  (0x03)
#define MODBUS_FUNCTION_CODE_04                  (0x04)

typedef enum : uint8_t {mrsProcessing = 0x00, mrsReadingDone, mrsErrorTimeout, mrsErrorOverflow} modbusReadState_t;
typedef enum : uint8_t {mqsSuccess = 0x00, mqsUnexpected, mqsWrongAnswer, mqsWrongAnswerLength, mqsBadCRC, mqsDeviceTimeout, mqsExceptionIllegalFunction, mqsExceptionIllegalDataAddress, mqsExceptionIllegalDataValue, mqsExceptionSlaveDeviceFailure, mqsExceptionAcknowledge, mqsExceptionSlaveDeviceBusy, mqsExceptionNegativeAcknowledge, mqsExceptionMemoryParityError, mqsExceptionUnknown} modbusQueryState_t;

//static uint16_t modbusRtuCalcCRC(uint8_t* _dst, uint8_t _len);
//static void modbusRtuProcessingFC(const uint8_t, uint8_t*, const uint8_t);
//static void modbusRtuMakeQueryHeader(const uint8_t, const uint8_t, const uint16_t, uint8_t*, const uint8_t);
//static modbusQueryState_t modbusRtuValidateAnswer(const uint8_t, uint8_t*, const uint8_t, const uint8_t);
int8_t modbusRtuUart(const uint8_t, const uint8_t, const int8_t, const uint16_t, const uint8_t, const uint8_t, const uint16_t, uint8_t*, const uint8_t);
