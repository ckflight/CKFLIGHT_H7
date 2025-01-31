
#include <COMMON/maths.h>
#include "DRIVERS/CK_SPI.h"
#include "DRIVERS/CK_GPIO.h"
#include "DRIVERS/CK_BUZZER.h"

#include "SENSORS/CK_BMP280.h"

#include "MOTION/CK_BAROMETER.h"


#define BMP280_REGISTER_DIG_T1        0x88
#define BMP280_REGISTER_DIG_T2        0x8A
#define BMP280_REGISTER_DIG_T3        0x8C

#define BMP280_REGISTER_DIG_P1        0x8E
#define BMP280_REGISTER_DIG_P2        0x90
#define BMP280_REGISTER_DIG_P3        0x92
#define BMP280_REGISTER_DIG_P4        0x94
#define BMP280_REGISTER_DIG_P5        0x96
#define BMP280_REGISTER_DIG_P6        0x98
#define BMP280_REGISTER_DIG_P7        0x9A
#define BMP280_REGISTER_DIG_P8        0x9C
#define BMP280_REGISTER_DIG_P9        0x9E

#define BMP280_REGISTER_CHIPID        0xD0
#define BMP280_REGISTER_CONTROL       0xF4
#define BMP280_REGISTER_CONTROL2      0xF5

#define BMP280_REGISTER_PRESSUREDATA  0xF7
#define BMP280_REGISTER_TEMPDATA      0xFA

#define BMP280_CHIPID                 0x58

#define READ_ARRAY_SIZE               30

SPI_TypeDef * SPI_BMP280;
GPIO_TypeDef* GPIO_CS_BMP280;
uint8_t CS_PIN_BMP280;

typedef struct{

	uint16_t dig_T1;
	int16_t  dig_T2;
	int16_t  dig_T3;

	uint16_t dig_P1;
	int16_t  dig_P2;
	int16_t  dig_P3;
	int16_t  dig_P4;
	int16_t  dig_P5;
	int16_t  dig_P6;
	int16_t  dig_P7;
	int16_t  dig_P8;
	int16_t  dig_P9;

	int32_t t_fine;

}barometerSensorBMP280_t;

barometerSensorBMP280_t bmp280_results;

typedef struct{

    bool BarometerInit;

    bool HardwareInit;

    uint8_t rxArray[READ_ARRAY_SIZE];


}BMP280_PARAMETERS_t;

BMP280_PARAMETERS_t bmp280 = {
    .BarometerInit = false,
    .HardwareInit = false,
    .rxArray = {0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0}

};


void CK_BMP280_Init(SPI_TypeDef* SPIn, GPIO_TypeDef* GPIO_CSn, uint8_t CS_PINn, uint32_t baroFreq){

	SPI_BMP280 = SPIn;
	GPIO_CS_BMP280 = GPIO_CSn;
	CS_PIN_BMP280 = CS_PINn;

	if(CK_SPI_WriteRegister(BMP280_REGISTER_CHIPID | 0x80, 0xFF, SPI_BMP280, GPIO_CS_BMP280, CS_PIN_BMP280) == 0x58){

		CK_SPI_WriteRegister(BMP280_REGISTER_CONTROL, 0x3F, SPI_BMP280, GPIO_CS_BMP280, CS_PIN_BMP280);

		// Read Trim Values
		CK_SPI_ReadRegisterMulti(BMP280_REGISTER_DIG_T1, SPI_BMP280, GPIO_CS_BMP280, CS_PIN_BMP280, bmp280.rxArray, 24);
		bmp280_results.dig_T1 = (bmp280.rxArray[1] << 8)            | bmp280.rxArray[0];
		bmp280_results.dig_T2 = (int16_t)((bmp280.rxArray[3] << 8)  | bmp280.rxArray[2]);
		bmp280_results.dig_T3 = (int16_t)((bmp280.rxArray[5] << 8)  | bmp280.rxArray[4]);

		bmp280_results.dig_P1 = ((bmp280.rxArray[7] << 8)           | bmp280.rxArray[6]);
		bmp280_results.dig_P2 = (int16_t)((bmp280.rxArray[9] << 8)  | bmp280.rxArray[8]);
		bmp280_results.dig_P3 = (int16_t)((bmp280.rxArray[11] << 8) | bmp280.rxArray[10]);
		bmp280_results.dig_P4 = (int16_t)((bmp280.rxArray[13] << 8) | bmp280.rxArray[12]);
		bmp280_results.dig_P5 = (int16_t)((bmp280.rxArray[15] << 8) | bmp280.rxArray[14]);
		bmp280_results.dig_P6 = (int16_t)((bmp280.rxArray[17] << 8) | bmp280.rxArray[16]);
		bmp280_results.dig_P7 = (int16_t)((bmp280.rxArray[19] << 8) | bmp280.rxArray[18]);
		bmp280_results.dig_P8 = (int16_t)((bmp280.rxArray[21] << 8) | bmp280.rxArray[20]);
		bmp280_results.dig_P9 = (int16_t)((bmp280.rxArray[23] << 8) | bmp280.rxArray[22]);

		bmp280.BarometerInit = true;

		bmp280.HardwareInit = true;

	}

}

void CK_BMP280_Calculate(void){

	if(bmp280.BarometerInit){

		CK_BMP280_ReadADC();

		CK_BMP280_CalculateTemperature();

		CK_BMP280_CalculatePressure();
	}
}

void CK_BMP280_CalculateTemperature(){

	int32_t var1, var2;

	int32_t adc_T = bmp280.rxArray[3] << 16 | bmp280.rxArray[4] << 8 | bmp280.rxArray[5];

	adc_T >>= 4;// ADC result is 20bit

	var1  = ((((adc_T>>3) - ((int32_t)bmp280_results.dig_T1 <<1))) * ((int32_t)bmp280_results.dig_T2)) >> 11;

	var2  = (((((adc_T>>4) - ((int32_t)bmp280_results.dig_T1)) * ((adc_T>>4) - ((int32_t)bmp280_results.dig_T1))) >> 12) * ((int32_t)bmp280_results.dig_T3)) >> 14;

	bmp280_results.t_fine = var1 + var2;

	float T  = (bmp280_results.t_fine * 5 + 128) >> 8;

	barometer.temperature = T/100;
}

void CK_BMP280_CalculatePressure(){

	int64_t var1, var2, p;

	int32_t adc_P = bmp280.rxArray[0] << 16 | bmp280.rxArray[1] << 8 | bmp280.rxArray[2];

	adc_P >>= 4;// ADC result is 20bit

	var1 = ((int64_t)bmp280_results.t_fine) - 128000;

	var2 = var1 * var1 * (int64_t)bmp280_results.dig_P6;

	var2 = var2 + ((var1*(int64_t)bmp280_results.dig_P5)<<17);

	var2 = var2 + (((int64_t)bmp280_results.dig_P4)<<35);

	var1 = ((var1 * var1 * (int64_t)bmp280_results.dig_P3)>>8) + ((var1 * (int64_t)bmp280_results.dig_P2)<<12);

	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)bmp280_results.dig_P1)>>33;

	if (var1 == 0) {
		return;  // avoid exception caused by dividing zero
	}
	p = 1048576 - adc_P;

	p = (((p<<31) - var2)*3125) / var1;

	var1 = (((int64_t)bmp280_results.dig_P9) * (p>>13) * (p>>13)) >> 25;

	var2 = (((int64_t)bmp280_results.dig_P8) * p) >> 19;

	p = ((p + var1 + var2) >> 8) + (((int64_t)bmp280_results.dig_P7)<<4);

	barometer.pressure = (float)p/256;

}


void CK_BMP280_ReadADC(void){

	// Read pressure and temp adc
	CK_SPI_ReadRegisterMulti(BMP280_REGISTER_PRESSUREDATA, SPI_BMP280, GPIO_CS_BMP280, CS_PIN_BMP280, bmp280.rxArray, 6);

}

