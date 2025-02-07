
#ifndef CK_MAGNETO_H_
#define CK_MAGNETO_H_

#include "CK_DEFINITIONS.h"
#include "COMMON/axis.h"

#define MAG_READ_ARRAY_SIZE	16

typedef struct{

    float offset[3];
    float softiron_matrix[3][3];
    float field_strength;

}magnetometer_calibratio_t;

typedef struct magnetoSensor_s{
    int16_t     magADCRaw[XYZ_AXIS_COUNT];
    float       magADCFiltered[XYZ_AXIS_COUNT];

    //Mag does not use this type of calibration but not removing anyway
    //It uses motion calibration and spherical calibration for soft and hard iron
    int16_t     magADCMin[XYZ_AXIS_COUNT];
    int16_t     magADCMax[XYZ_AXIS_COUNT];

    float       magHardIron[XYZ_AXIS_COUNT];
    float       magSoftIron[XYZ_AXIS_COUNT];

    float       magADCf[XYZ_AXIS_COUNT];
    float       magScale[XYZ_AXIS_COUNT];
    int         magSign[XYZ_AXIS_COUNT];

    int         magHeading;

    int 		is_mag_spi_init;
    int 		is_mag_i2c_init;

    int 		mag_reInit_counter;

    bool		is_magswapaxis;

    sensorModel_e sensor;
    syncTimer_t sync;

}magnetoSensor_t;

extern magnetoSensor_t mag;

void CK_MAGNETO_Init(SPI_TypeDef* spin_, GPIO_TypeDef* gpio_cs_, uint8_t cs_pin_, sensorModel_e sensor, uint32_t magT, uint32_t mainT);

void CK_MAGNETO_Init2(I2C_TypeDef* i2cn_, sensorModel_e sensor, uint32_t magT, uint32_t mainT);

void CK_MAGNETO_LoadCalibrationParameters(void);

void CK_MAGNETO_Update(void);

void CK_MAGNETO_SwapXYAxis(void);

void CK_MAGNETO_PerformCalibration(void);

void CK_MAGNETO_CalculateHeading(float mx, float my, float mz);

void CK_MAGNETO_CheckTimeout(void);

#endif /* CK_MAGNETO_H_ */
