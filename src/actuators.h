#pragma once

// Send pulse  to servo every ... ms
#define SERVO_PULSE_INTERVAL       (20)

/*****************************************************************************************************************************
*
*  Send command to Servo to make turn it an turn back
*
*  Returns: 
*    - RESULT_IS_OK    on success
*    - RESULT_IS_FAIL  if wrong pin number is specified
*
*****************************************************************************************************************************/
int8_t servoTurn(const uint8_t, const uint16_t, const uint32_t, const uint32_t, const uint16_t);

/*****************************************************************************************************************************
*
*  Set pin to some state, wait, and set pin to other state
*
*  Returns: 
*    - RESULT_IS_OK  on any case
*
*****************************************************************************************************************************/
int8_t pulse(const uint8_t, const uint8_t, const uint32_t, const uint8_t);

/*****************************************************************************************************************************
*
*  Set pin to some state and test control pin (may be second pair of relay contacts)
*
*  Returns: 
*    - RESULT_IS_OK     on test OK
*    - RESULT_IS_FAIL   on otherwise
*
*****************************************************************************************************************************/
int8_t relay(const uint8_t, const uint8_t, const int8_t, const int8_t, const uint8_t);
