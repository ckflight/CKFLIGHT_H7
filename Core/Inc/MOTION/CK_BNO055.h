
#ifndef CK_BNO055_H_
#define CK_BNO055_H_

#include "CK_DEFINITIONS.h"
#include "COMMON/axis.h"

typedef struct{

    float       eulerAngles[XYZ_AXIS_COUNT];

    float       angleScale;

    uint8_t     calibration_gyroStatus;
    uint8_t     calibration_accStatus;
    uint8_t     calibration_magStatus;
    uint8_t     calibration_sysStatus;

    syncTimer_t sync;

}bno055Sensor_t;

extern bno055Sensor_t bno055;

void CK_BNO055_Init(I2C_TypeDef* i2cn_, uint32_t bnoT, uint32_t mainT);

void CK_BNO055_Update(void);

void CK_BNO055_ReadEulerRaw(int16_t* p_x, int16_t* p_y, int16_t* p_z);

void CK_BNO055_Calibrate(void);

void CK_BNO055_CONFIGMode(void);

void CK_BNO055_NDOFMode(void);

void CK_BNO055_ReadSensorOffsetAndRadius(void);

void CK_BNO055_WriteSensorOffsetAndRadius(void);

#endif /* CK_BNO055_H_ */
