#pragma once

// Send pulse  to servo every ... ms
#define SERVO_PULSE_INTERVAL       (20)

/*****************************************************************************************************************************
*
*  
*
*   Returns: 
*     - 
*
*****************************************************************************************************************************/
int8_t servoTurn(const uint8_t, const uint16_t, const uint32_t, const uint32_t, const uint16_t);

int8_t pulse(const uint8_t, const uint8_t, const uint32_t, const uint8_t);

int8_t relay(const uint8_t, const uint8_t, const int8_t, const int8_t, const uint8_t);
