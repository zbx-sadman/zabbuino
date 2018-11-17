#pragma once
/*


*/

#define PLANTOWER_UART_SPEED                                    (9600)
#define PLANTOWER_DEFAULT_READ_TIMEOUT                          (2500UL)

typedef struct {
  uint16_t frameLength;
  uint16_t standartPM10, standartPM25, standartPM100;
  uint16_t environmentPM10, environmentPM25, environmentPM100;
  uint16_t particles03um, particles05um, particles10um, particles25um, particles50um, particles100um;
  uint16_t unused;
  uint16_t checkCode;
} plantowerData_t;

/*****************************************************************************************************************************
*
*   Overloads of main subroutine. Used to get numeric metric's value or it's char presentation only
*
*****************************************************************************************************************************/
int8_t getPlantowerPM25Metric(const uint8_t _rxPin, const uint8_t _txPin, const uint8_t _metric, uint32_t* _value);

int8_t getPlantowerPM25Metric(const uint8_t _rxPin, const uint8_t _txPin, const uint8_t _metric, char* _dst);

int8_t getPlantowerPM25AllMetrics(const uint8_t _rxPin, const uint8_t _txPin, char* _dst);


/*****************************************************************************************************************************
*
*  Read values of the specified metric from the Plantower PMS sensors, put it to output buffer on success. 
*
*   Returns: 
*     - RESULT_IS_BUFFERED on success
*     - DEVICE_ERROR_TIMEOUT if device stop talking
*     - DEVICE_ERROR_CHECKSUM on recieve error
*
*****************************************************************************************************************************/
int8_t getPlantowerPM25Metric(const uint8_t _rxPin, const uint8_t _txPin, const uint8_t _metric, char* _dst, uint32_t* _value, const uint8_t _outputType);
