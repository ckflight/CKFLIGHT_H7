
#include "DRIVERS/CK_I2C.h"

#include "MOTION/CK_MAGNETO.h"
#include "MOTION/CK_ACC.h"

#include "SENSORS/CK_LSM303D.h"

#define LSM303_ADDRESS_ACCEL          (0x32 >> 1)
#define LSM303_ADDRESS_MAG            (0x3C >> 1)

I2C_TypeDef* LSM303D_I2C;

typedef struct{

    bool AccInit;

    bool MagInit;

    uint8_t rxArray[ACC_READ_ARRAY_SIZE];


}LSM303D_PARAMETERS_t;

LSM303D_PARAMETERS_t lsm303d = {
    .AccInit = false,
    .MagInit = false,
    .rxArray = {0, 0, 0, 0, 0, 0}
};

void CK_LSM303D_AccInit(I2C_TypeDef* I2Cn, uint32_t accFreq){

	LSM303D_I2C = I2Cn;

	// LSM303 ACC
	CK_I2C_Transfer(LSM303D_I2C, LSM303_ADDRESS_ACCEL, 0x20, 0x77);// set to 400Hz

	CK_I2C_Transfer(LSM303D_I2C, LSM303_ADDRESS_ACCEL, 0x23, 0x28);// 8g, High Resolution 4mg/LSB

    lsm303d.AccInit = true;

}

void CK_LSM303D_MagInit(I2C_TypeDef* I2Cn, uint32_t magFreq){

    LSM303D_I2C = I2Cn;

    // LSM303 MAG
    //CK_I2C_Transfer(LSM303D_I2Cx, LSM303_ADDRESS_MAG, 0x00, 0x1C);// set to 220Hz
    //CK_I2C_Transfer(LSM303D_I2Cx, LSM303_ADDRESS_MAG, 0x00, 0x18);// set to 75Hz
    CK_I2C_Transfer(LSM303D_I2C, LSM303_ADDRESS_MAG, 0x00, 0x10);// set to 15Hz

    CK_I2C_Transfer(LSM303D_I2C, LSM303_ADDRESS_MAG, 0x01, 0x20);// 1.3 gauss

    CK_I2C_Transfer(LSM303D_I2C, LSM303_ADDRESS_MAG, 0x02, 0x00);// continuous mode


    mag.magScale[0] = (float)(1.0f / 1100.0f);

    mag.magScale[1] = (float)(1.0f / 980.0f);

    lsm303d.MagInit = true;

}

void CK_LSM303D_AlignAcc(int x, int y, int z){

	acc.accSign[X]  = x;

	acc.accSign[Y] = y;

	acc.accSign[Z]   = z;

}

void CK_LSM303D_AlignMag(int x, int y, int z){

	mag.magSign[X]  = x;

	mag.magSign[Y] = y;

	mag.magSign[Z]   = z;

}

void CK_LSM303D_ReadAccRaw(void){

	CK_I2C_ReadMulti(LSM303D_I2C, LSM303_ADDRESS_ACCEL, 0x28 | 0x80, lsm303d.rxArray, 6);

	uint8_t xlo = lsm303d.rxArray[0];
	uint8_t xhi = lsm303d.rxArray[1];
	uint8_t ylo = lsm303d.rxArray[2];
	uint8_t yhi = lsm303d.rxArray[3];
	uint8_t zlo = lsm303d.rxArray[4];
	uint8_t zhi = lsm303d.rxArray[5];

	acc.accADCRaw[X]  = (int16_t)(xlo | (xhi << 8)) >> 4;

	acc.accADCRaw[Y] = (int16_t)(ylo | (yhi << 8)) >> 4;

	acc.accADCRaw[Z]   = (int16_t)(zlo | (zhi << 8)) >> 4;

}

void CK_LSM303D_ReadMagRaw(void){

	CK_I2C_ReadMulti(LSM303D_I2C, LSM303_ADDRESS_MAG, 0x03, lsm303d.rxArray, 6);

	uint8_t xhi = lsm303d.rxArray[0];
	uint8_t xlo = lsm303d.rxArray[1];
	uint8_t zhi = lsm303d.rxArray[2];
	uint8_t zlo = lsm303d.rxArray[3];
	uint8_t yhi = lsm303d.rxArray[4];
	uint8_t ylo = lsm303d.rxArray[5];

	mag.magADCRaw[X]  = (int16_t)(xlo | (xhi << 8));
	mag.magADCRaw[Y] = (int16_t)(ylo | (yhi << 8));
	mag.magADCRaw[Z]   = (int16_t)(zlo | (zhi << 8));

}

bool CK_LSM303D_isAccSensorInitialized(void){

    return lsm303d.AccInit;
}

bool CK_LSM303D_isMagSensorInitialized(void){

    return lsm303d.MagInit;
}




