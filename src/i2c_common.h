#ifndef _ZABBUINO_I2C_COMMON_H_
#define _ZABBUINO_I2C_COMMON_H_

/*****************************************************************************************************************************
*
*   Scan I2C bus and print to ethernet client addresses of all detected devices 
*
*   Returns: 
*     - RESULT_IS_PRINTED on success
*     - RESULT_IS_FAIL of no devices found 
*
*****************************************************************************************************************************/
int8_t scanI2C(SoftwareWire*, NetworkClass*);

#endif // _ZABBUINO_I2C_COMMON_H_