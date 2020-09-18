#pragma once

/*****************************************************************************************************************************
*
*   Scan I2C bus and store I2C addresses of detected devices 
*
*   Returns: 
*     - number of found devices
*
*****************************************************************************************************************************/
int8_t scanI2C(SoftwareTWI&, uint8_t*);

