#pragma once
/*


*/


#define NOVA_SDS_UART_SPEED                                     (9600)
#define NOVA_SDS_DEFAULT_READ_TIMEOUT                           (1000UL)
#define NOVA_SDS_REQUEST_SUZE                                   (0x13)

#define NOVA_SDS_PARTICLES_SIZE_NONE                            (0x00)
#define NOVA_SDS_CONCENTRATION_TYPE_NONE                        (0x00)

#define NOVA_SDS_PARTICLES_SIZE_010                             (0x0A)
#define NOVA_SDS_PARTICLES_SIZE_025                             (0x19)

#define NOVA_SDS_FIELD_REQUEST_HEAD                             (0x00)
#define NOVA_SDS_FIELD_REQUEST_COMMAND_ID                       (0x01)
#define NOVA_SDS_FIELD_REQUEST_DATABYTE_01                      (0x02)
#define NOVA_SDS_FIELD_REQUEST_DATABYTE_14                      (0x0F)
#define NOVA_SDS_FIELD_REQUEST_DATABYTE_15                      (0x10)
#define NOVA_SDS_FIELD_REQUEST_CHECKSUM                         (0x11)
#define NOVA_SDS_FIELD_REQUEST_TAIL                             (0x12)

#define NOVA_SDS_FIELD_RESPONSE_HEAD                            (0x00)
#define NOVA_SDS_FIELD_RESPONSE_COMMAND_ID                      (0x01)
#define NOVA_SDS_FIELD_RESPONSE_DATABYTE_01                     (0x02)
#define NOVA_SDS_FIELD_RESPONSE_DATABYTE_02                     (0x03)
#define NOVA_SDS_FIELD_RESPONSE_DATABYTE_03                     (0x04)
#define NOVA_SDS_FIELD_RESPONSE_DATABYTE_04                     (0x05)
#define NOVA_SDS_FIELD_RESPONSE_DATABYTE_05                     (0x06)
#define NOVA_SDS_FIELD_RESPONSE_DATABYTE_06                     (0x07)
#define NOVA_SDS_FIELD_RESPONSE_CHECKSUM                        (0x08)
#define NOVA_SDS_FIELD_RESPONSE_TAIL                            (0x09)


/*****************************************************************************************************************************
*
*  Read values of the specified metric from the Nova Fitness SDS (011) sensors, put it to output buffer or variable's address on success. 
*
*   Returns: 
*     - RESULT_IS_BUFFERED        on success and SENS_READ_ALL metric specified
*     - RESULT_IS_UNSIGNED_VALUE  on success and single metric specified
*     - DEVICE_ERROR_TIMEOUT      if device stop talking
*     - DEVICE_ERROR_CHECKSUM     on checksum error
*
*****************************************************************************************************************************/
int8_t getNovaSDSOneMetric(const uint8_t, const uint8_t, uint8_t, const uint8_t, uint32_t*);
int8_t getNovaSDSAllMetrics(const uint8_t, const uint8_t, char*, const uint16_t);
