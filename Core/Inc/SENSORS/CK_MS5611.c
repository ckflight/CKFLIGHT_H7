
#include <COMMON/maths.h>
#include "DRIVERS/CK_SPI.h"
#include "DRIVERS/CK_GPIO.h"
#include "DRIVERS/CK_TIME_HAL.h"
#include "DRIVERS/CK_BUZZER.h"

#include "SENSORS/CK_MS5611.h"

#include "MOTION/CK_BAROMETER.h"


#define MS5611_RESET                0x1E
#define MS5611_CONV_D1_4096         0x48
#define MS5611_CONV_D2_4096         0x58
#define MS5611_ADC_READ             0x00

#define MS5611_PROM_RD              0xA0
#define MS5611_PROM_C1              0xA2
#define MS5611_PROM_C2              0xA4
#define MS5611_PROM_C3              0xA6
#define MS5611_PROM_C4              0xA8
#define MS5611_PROM_C5              0xAA
#define MS5611_PROM_C6              0xAC

SPI_TypeDef * SPI_MS5611;
GPIO_TypeDef* GPIO_CS_MS5611;
uint8_t CS_PIN_MS5611;

typedef struct{

	uint16_t C[8];
	uint32_t D1;
	uint32_t D2;

}barometerSensorMS5611_t;

barometerSensorMS5611_t ms5611_result;

typedef struct{

    bool BarometerInit;

    bool HardwareInit;

    int oneCycleComplete_MS5611;

    int adcType_MS5611;


}MS5611_PARAMETERS_t;

MS5611_PARAMETERS_t ms5611 = {

    .BarometerInit = false,

    .HardwareInit = false,

    .oneCycleComplete_MS5611 = 0,

    .adcType_MS5611 = 0
};

void CK_MS5611_Init(SPI_TypeDef* SPIn, GPIO_TypeDef* GPIO_CSn, uint8_t CS_PINn, uint32_t baroFreq){

	SPI_MS5611     = SPIn;
	GPIO_CS_MS5611 = GPIO_CSn;
	CS_PIN_MS5611  = CS_PINn;

	// Reset Command
	CK_MS5611_SendCommand(MS5611_RESET);
	CK_TIME_DelayMilliSec(10);

	CK_MS5611_ReadPROM();

	// Start Conversions for first time then cycle will update itself
	CK_MS5611_SendCommand(MS5611_CONV_D2_4096);
	CK_TIME_DelayMilliSec(10);

	ms5611.adcType_MS5611 = 0;

	ms5611.BarometerInit = true;

	ms5611.HardwareInit = true;

}

void CK_MS5611_CycleUpdate(void){

	if(ms5611.BarometerInit){

		// At the end of 20ms data is ready so 50Hz
		if(ms5611.adcType_MS5611 == 0){

		    ms5611.oneCycleComplete_MS5611 = 0;

			CK_MS5611_ReadADC(MS5611_TEMPERATURE_ADC);// Read 24bit Temperature ADC

			CK_MS5611_SendCommand(MS5611_CONV_D1_4096);// Send D1 Convert Command (Pressure)

			ms5611.adcType_MS5611++;

		}
		else if(ms5611.adcType_MS5611 == 1){

			CK_MS5611_ReadADC(MS5611_PRESSURE_ADC);// Read 24bit Pressure ADC

			CK_MS5611_SendCommand(MS5611_CONV_D2_4096);// Send D1 Convert Command (Temperature)

			ms5611.oneCycleComplete_MS5611 = 1;

			ms5611.adcType_MS5611 = 0;

		}

	}

}

void CK_MS5611_Calculate(){

    uint32_t press;
    int64_t temp;
    int64_t delt;

    int64_t dT = (int64_t)ms5611_result.D2 - ((uint64_t)ms5611_result.C[5] * 256);

    int64_t off = ((int64_t)ms5611_result.C[2] << 16) + (((int64_t)ms5611_result.C[4] * dT) >> 7);

    int64_t sens = ((int64_t)ms5611_result.C[1] << 15) + (((int64_t)ms5611_result.C[3] * dT) >> 8);

    temp = 2000 + ((dT * (int64_t)ms5611_result.C[6]) >> 23);

    if (temp < 2000) {		// temperature lower than 20 celcius
        delt = temp - 2000;
        delt = delt * delt;
        off -= (5 * delt) >> 1;
        sens -= (5 * delt) >> 2;
        if (temp < -1500) { // temperature lower than -15 celcius
            delt = temp + 1500;
            delt = delt * delt;
            off -= 7 * delt;
            sens -= (11 * delt) >> 1;
        }
    temp -= ((dT * dT) >> 31);
    }

    press = ((((int64_t)ms5611_result.D1 * sens) >> 21) - off) >> 15;

    barometer.pressure = press;				// pressure in Pascal
    barometer.temperature = temp * 0.01;		// In celcius

}

void CK_MS5611_ReadPROM(void){

	// Read PROM for calculations
	for(int i = 0; i< 8; i++){
		CK_GPIO_ClearPin(GPIO_CS_MS5611, CS_PIN_MS5611);

		CK_SPI_Transfer(SPI_MS5611, MS5611_PROM_RD+2*i);
		uint16_t temp;

		temp = CK_SPI_Transfer(SPI_MS5611, 0x00);
		temp <<= 8;
		temp |= CK_SPI_Transfer(SPI_MS5611, 0x00);

		ms5611_result.C[i] = temp;

		CK_GPIO_SetPin(GPIO_CS_MS5611, CS_PIN_MS5611);

	}
}

void CK_MS5611_SendCommand(uint8_t command){
	// This is a command send not writeRegister operation
	// That is why implemented this way rather than using CK_SPI_WriteRegister
	CK_GPIO_ClearPin(GPIO_CS_MS5611, CS_PIN_MS5611);

	CK_SPI_Transfer(SPI_MS5611, command);

	CK_GPIO_SetPin(GPIO_CS_MS5611, CS_PIN_MS5611);
}

void CK_MS5611_ReadADC(adcTypeMS5611_e adc){

	uint8_t temp1;
	uint8_t temp2;
	uint8_t temp3;

	CK_GPIO_ClearPin(GPIO_CS_MS5611, CS_PIN_MS5611);

	CK_SPI_Transfer(SPI_MS5611, MS5611_ADC_READ);

	// Read 24 bit data
	temp1 = CK_SPI_Transfer(SPI_MS5611, 0);

	temp2 = CK_SPI_Transfer(SPI_MS5611, 0);

	temp3 = CK_SPI_Transfer(SPI_MS5611, 0);

	if(adc == MS5611_PRESSURE_ADC){
	    ms5611_result.D1 = (temp1 << 16) | (temp2 << 8) | (temp3);
	}
	else if(adc == MS5611_TEMPERATURE_ADC){
	    ms5611_result.D2 = (temp1 << 16) | (temp2 << 8) | (temp3);
	}

	CK_GPIO_SetPin(GPIO_CS_MS5611, CS_PIN_MS5611);

}

int CK_MS5611_IsOneCycleCompleted(){
	return ms5611.oneCycleComplete_MS5611;
}














