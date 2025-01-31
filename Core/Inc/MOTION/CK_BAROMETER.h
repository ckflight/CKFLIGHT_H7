
#ifndef CK_BAROMETER_H_
#define CK_BAROMETER_H_

#include "CK_DEFINITIONS.h"

typedef struct barometerSensor_s{

    int32_t     pressure;
    int32_t     temperature;
    int32_t     altitude;

    int32_t     groundAltitude;
    uint32_t    pressureSum;

    int32_t     altitudeHold;

    int         isBarometerReady;

    int 		barometer_reInit_counter;

    bool 		is_baro_init;

    sensorModel_e sensor;

    syncTimer_t sync;

}barometerSensor_t;

extern barometerSensor_t barometer;

void CK_BAROMETER_Init(SPI_TypeDef* spin_, GPIO_TypeDef* cs_gpio_, uint8_t cs_pin_, sensorModel_e sensor, uint32_t barometerT, uint32_t mainT);

void CK_BAROMETER_Update(void);

uint32_t CK_BAROMETER_RecalculateTotal(uint8_t baroSampleCount, int32_t newPressureReading);

int32_t CK_BAROMETER_ApplyMedianFilter(int32_t newPressureReading);

void CK_BAROMETER_PerformBaroCalibration(void);

int32_t CK_BAROMETER_CalculateAltitude(void);

void CK_BAROMETER_CheckTimeout(void);

#endif /* CK_BAROMETER_H_ */
