#pragma once
/*


*/


#define PLANTOWER_UART_SPEED                                    (9600)
#define PLANTOWER_DEFAULT_READ_TIMEOUT                          (2500UL)

#define PLANTOWER_PARTICLES_SIZE_NONE                           (0x00)
#define PLANTOWER_PARTICLES_SIZE_003                            (0x03)
#define PLANTOWER_PARTICLES_SIZE_005                            (0x05)
#define PLANTOWER_PARTICLES_SIZE_010                            (0x0A)
#define PLANTOWER_PARTICLES_SIZE_025                            (0x19)
#define PLANTOWER_PARTICLES_SIZE_050                            (0x32)
#define PLANTOWER_PARTICLES_SIZE_100                            (0x64)

#define PLANTOWER_CONCENTRATION_TYPE_NONE                       (0x00)
#define PLANTOWER_CONCENTRATION_TYPE_FACTORY                    (0x01)
#define PLANTOWER_CONCENTRATION_TYPE_ENVIRONMENT                (0x02)

#define PLANTOWER_READ_FACTORY_PM010                            (0x10)
#define PLANTOWER_READ_FACTORY_PM025                            (0x11)
#define PLANTOWER_READ_FACTORY_PM100                            (0x12)
#define PLANTOWER_READ_ENVIRONMENT_PM010                        (0x13)
#define PLANTOWER_READ_ENVIRONMENT_PM025                        (0x14)
#define PLANTOWER_READ_ENVIRONMENT_PM100                        (0x15)
#define PLANTOWER_READ_PARTICLES_003_UM                         (0x16)
#define PLANTOWER_READ_PARTICLES_005_UM                         (0x17)
#define PLANTOWER_READ_PARTICLES_010_UM                         (0x18)
#define PLANTOWER_READ_PARTICLES_025_UM                         (0x19)
#define PLANTOWER_READ_PARTICLES_050_UM                         (0x1A)
#define PLANTOWER_READ_PARTICLES_100_UM                         (0x1B)
#define PLANTOWER_READ_ALL                                      (0xFF)

typedef struct {
  uint16_t frameLength;
  uint16_t standartPM10, standartPM25, standartPM100;
  uint16_t environmentPM10, environmentPM25, environmentPM100;
  uint16_t particles03um, particles05um, particles10um, particles25um, particles50um, particles100um;
  uint16_t unused;
  uint16_t crc;
} plantowerData_t;


/*****************************************************************************************************************************
*
*   
*
*****************************************************************************************************************************/
int8_t getPlantowerPMSOneMetric(const uint8_t, const uint8_t, uint8_t, const uint8_t, const uint8_t, uint32_t*);

int8_t getPlantowerPMSAllMetrics(const uint8_t, const uint8_t, char*, const uint16_t);

