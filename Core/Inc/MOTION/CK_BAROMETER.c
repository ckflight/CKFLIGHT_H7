

#include <COMMON/maths.h>
#include "DRIVERS/CK_TIME_HAL.h"
#include "DRIVERS/CK_SPI.h"
#include "DRIVERS/CK_BUZZER.h"

#include "SENSORS/CK_MS5607.h"
#include "SENSORS/CK_MS5611.h"
#include "SENSORS/CK_BMP280.h"

#include "MOTION/CK_BAROMETER.h"

#include "COMMON/CK_FILTERS.h"

#include "COMMUNICATION/CK_PRINTER.h"

#define NUM_OF_BAROMETERCALIBRATION         23

#define TIMEOUT_THREASHOLD                  10 // 10 Timeout starts reInit process
#define TIMEOUT_REINIT_THREASHOLD           3  // 3 Times max tries to reInit

barometerSensor_t barometer;

uint16_t calibratingB = 0;

int32_t baroGroundPressure = 8 * 101325;

DEBUG_TIME_t barometer_debug;

SPI_TypeDef* MOTION_BARO_SPI;
GPIO_TypeDef* MOTION_BARO_CS_PORT;
uint8_t MOTION_BARO_CS_PIN;

void CK_BAROMETER_Init(SPI_TypeDef* spin_, GPIO_TypeDef* cs_gpio_, uint8_t cs_pin_, sensorModel_e sensor, uint32_t barometerT, uint32_t mainT){

	MOTION_BARO_SPI 	= spin_;
	MOTION_BARO_CS_PORT = cs_gpio_;
	MOTION_BARO_CS_PIN 	= cs_pin_;

	barometer.sensor = sensor;

	barometer.sync.syncRate = barometerT / mainT;

	barometer.sync.targetLoopTime = barometerT;

	barometer.sync.syncCounter = 0;

	barometer.pressureSum = 0;

	barometer.isBarometerReady = 0;

	barometer.barometer_reInit_counter = 0;


	if(barometer.sensor == MS5607_BAROMETER){

		CK_MS5607_Init(MOTION_BARO_SPI, MOTION_BARO_CS_PORT, MOTION_BARO_CS_PIN, barometer.sync.targetLoopTime);

	}
	else if(barometer.sensor == BMP280_BAROMETER){

		CK_BMP280_Init(MOTION_BARO_SPI, MOTION_BARO_CS_PORT, MOTION_BARO_CS_PIN, barometer.sync.targetLoopTime);

	}
	else if(barometer.sensor == MS5611_BAROMETER){

		CK_MS5611_Init(MOTION_BARO_SPI, MOTION_BARO_CS_PORT, MOTION_BARO_CS_PIN, barometer.sync.targetLoopTime);

	}

	calibratingB = 200;


	CK_BAROMETER_PerformBaroCalibration();

	barometer.is_baro_init = true;

}

void CK_BAROMETER_Update(void){

	if(barometer.is_baro_init){

		// For MS56xx baro update sends command at 100Hz 10ms takes 2 times
		// one final reading done at 50Hz
		barometer.sync.syncCounter++;

		if(barometer.sync.syncCounter >= barometer.sync.syncRate){

			#if defined(DEBUG_TIMING)
			barometer_debug.start_time = CK_TIME_GetMicroSec();
			#endif

			barometer.sync.syncCounter = 0;

			// Get Pressure and Temperature
			if(barometer.sensor == MS5607_BAROMETER){

				CK_MS5607_CycleUpdate();

				if(CK_MS5607_IsOneCycleCompleted()){

					CK_MS5607_Calculate();

					barometer.pressureSum = CK_BAROMETER_RecalculateTotal(21, barometer.pressure);

				}

			}
			else if(barometer.sensor == MS5611_BAROMETER){

				CK_MS5611_CycleUpdate();

				if(CK_MS5611_IsOneCycleCompleted()){

					CK_MS5611_Calculate();

					barometer.pressureSum = CK_BAROMETER_RecalculateTotal(21, barometer.pressure);

				}
			}
			else if(barometer.sensor == BMP280_BAROMETER){

				CK_BMP280_Calculate();
			}

			CK_BAROMETER_CheckTimeout();

			#if defined(DEBUG_TIMING)
			barometer_debug.update_time = CK_TIME_GetMicroSec() - barometer_debug.start_time;
			#endif
		}

	}
}

#define BARO_SAMPLE_COUNT_MAX   48
#define PRESSURE_SAMPLE_COUNT (21 - 1)

uint32_t CK_BAROMETER_RecalculateTotal(uint8_t baroSampleCount, int32_t newPressureReading){

    static int32_t barometerSamples[BARO_SAMPLE_COUNT_MAX];
    static int currentSampleIndex = 0;
    int nextSampleIndex;

    // store current pressure in barometerSamples
    nextSampleIndex = (currentSampleIndex + 1);
    if (nextSampleIndex == baroSampleCount) {
        nextSampleIndex = 0;
        barometer.isBarometerReady = 1;
    }
    barometerSamples[currentSampleIndex] = CK_BAROMETER_ApplyMedianFilter(newPressureReading);

    barometer.pressureSum += barometerSamples[currentSampleIndex];
    barometer.pressureSum -= barometerSamples[nextSampleIndex];

    currentSampleIndex = nextSampleIndex;

    return barometer.pressureSum;
}

#define PRESSURE_SAMPLES_MEDIAN 3

int32_t CK_BAROMETER_ApplyMedianFilter(int32_t newPressureReading){

    static int32_t barometerFilterSamples[PRESSURE_SAMPLES_MEDIAN];
    static int currentFilterSampleIndex = 0;
    static int medianFilterReady = 0;//false
    int nextSampleIndex;

    nextSampleIndex = (currentFilterSampleIndex + 1);
    if (nextSampleIndex == PRESSURE_SAMPLES_MEDIAN) {
        nextSampleIndex = 0;
        medianFilterReady = 1;//true
    }

    barometerFilterSamples[currentFilterSampleIndex] = newPressureReading;
    currentFilterSampleIndex = nextSampleIndex;

    if (medianFilterReady)
        return quickMedianFilter3(barometerFilterSamples);
    else
        return newPressureReading;
}

void CK_BAROMETER_PerformBaroCalibration(void){

    CK_PRINTER_PrintString("BAROMETER Calibration");
	int count = 0;
	while(calibratingB){

		if(barometer.sensor == MS5607_BAROMETER){

			CK_MS5607_CycleUpdate();

			if(CK_MS5607_IsOneCycleCompleted()){

				count++;
				CK_MS5607_Calculate();

				barometer.pressureSum = CK_BAROMETER_RecalculateTotal(21, barometer.pressure);

				if(barometer.isBarometerReady){
				    static int32_t savedGroundPressure = 0;

				    baroGroundPressure -= baroGroundPressure / 8;
				    baroGroundPressure += barometer.pressureSum / PRESSURE_SAMPLE_COUNT;
				    barometer.groundAltitude = (1.0f - powf((baroGroundPressure / 8) / 101325.0f, 0.190295f)) * 4433000.0f;

				    if (baroGroundPressure == savedGroundPressure)
				      calibratingB = 0;
				    else {
				      calibratingB--;
				      savedGroundPressure = baroGroundPressure;
				    }
				}
			}

		}

		else if(barometer.sensor == MS5611_BAROMETER){

			CK_MS5611_CycleUpdate();

			if(CK_MS5611_IsOneCycleCompleted()){

				count++;
				CK_MS5611_Calculate();

				barometer.pressureSum = CK_BAROMETER_RecalculateTotal(21, barometer.pressure);

				if(barometer.isBarometerReady){
				    static int32_t savedGroundPressure = 0;

				    baroGroundPressure -= baroGroundPressure / 8;
				    baroGroundPressure += barometer.pressureSum / PRESSURE_SAMPLE_COUNT;
				    barometer.groundAltitude = (1.0f - powf((baroGroundPressure / 8) / 101325.0f, 0.190295f)) * 4433000.0f;

				    if (baroGroundPressure == savedGroundPressure)
				      calibratingB = 0;
				    else {
				      calibratingB--;
				      savedGroundPressure = baroGroundPressure;
				    }
				}
			}

		}

		CK_TIME_DelayMilliSec(10);
		if(count % 10 == 0){
		    CK_PRINTER_PrintString(".");
		}
	}
	CK_PRINTER_PrintlnString("");

}

int32_t CK_BAROMETER_CalculateAltitude(void){

    int32_t alt_tmp;

    alt_tmp = lrintf((1.0f - powf((float)(barometer.pressureSum / PRESSURE_SAMPLE_COUNT) / 101325.0f, 0.190295f)) * 4433000.0f); // in cm
    alt_tmp -= barometer.groundAltitude;
    barometer.altitude = lrintf((float)barometer.altitude * 0.6f + (float)alt_tmp * 0.4f); // additional LPF to reduce baro noise

    return barometer.altitude;

}

void CK_BAROMETER_CheckTimeout(void){

	// Check timeout
    if(CK_SPI_GetTimeOut(MOTION_BARO_SPI) == TIMEOUT_THREASHOLD){

    	CK_BUZZER_Tone3();
        CK_SPI_ResetTimeOut(MOTION_BARO_SPI);

        if(barometer.barometer_reInit_counter < TIMEOUT_REINIT_THREASHOLD){

        	if(barometer.sensor == MS5607_BAROMETER){
        		CK_MS5607_Init(MOTION_BARO_SPI, MOTION_BARO_CS_PORT, MOTION_BARO_CS_PIN, barometer.sync.targetLoopTime);
        	}
        	else if(barometer.sensor == MS5611_BAROMETER){
        		CK_MS5611_Init(MOTION_BARO_SPI, MOTION_BARO_CS_PORT, MOTION_BARO_CS_PIN, barometer.sync.targetLoopTime);
        	}

            barometer.barometer_reInit_counter++;
        }
    }


}

