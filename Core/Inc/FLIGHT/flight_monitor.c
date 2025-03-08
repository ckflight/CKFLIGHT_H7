/*
 * battery_monitor.c
 *
 *  Created on: Feb 8, 2025
 *      Author: ck
 */

#include "FLIGHT/flight_monitor.h"
#include "DRIVERS/CK_ADC.h"
#include "DRIVERS/CK_BUZZER.h"
#include "FLIGHT/CK_RECEIVER.h"

#define WARNING_TIME_US		500000U
#define BATTER_CRITICAL_VOLTAGE			22.0f
#define BATTERY_AVERAGE					32

#define CRITICAL_TEMP_MAX				65.0f
#define TEMPERATURE_AVERAGE				32

typedef struct{

	bool is_battery_low;
	bool is_temp_high;

	syncTimer_t monitor_sync;
	syncTimer_t warning_sync;

	float battery_voltage;
	float battery_sum;
	uint8_t battery_average;

	float temperature;
	float temp_sum;
	uint8_t temperature_average;

	float gyroacc_sensor_temperature;

}flight_monitor_s;

flight_monitor_s monitor;

void flight_monitor_init(uint32_t monitorT, uint32_t mainT){

	monitor.monitor_sync.syncCounter = 0;
	monitor.monitor_sync.targetLoopTime = monitorT;
	monitor.monitor_sync.syncRate = monitorT / mainT;

	monitor.warning_sync.syncCounter = 0;
	monitor.warning_sync.syncRate = WARNING_TIME_US / monitorT;

	monitor.is_battery_low = false;
	monitor.is_temp_high = false;

	monitor.battery_voltage = 0.0f;
	monitor.battery_average = 0;
	monitor.battery_sum = 0.0f;

	monitor.temperature = 0.0f;
	monitor.temperature_average = 0;
	monitor.temp_sum = 0.0f;

	monitor.gyroacc_sensor_temperature = 0.0f;

}

void flight_monitor_update(void){

	monitor.monitor_sync.syncCounter++;

	if(monitor.monitor_sync.syncCounter >= monitor.monitor_sync.syncRate){

		monitor.monitor_sync.syncCounter = 0;
		monitor.warning_sync.syncCounter++; // controls buzzer beeping in every BATTER_LOW_WARNING_TIME_US microsecond

		CK_ADC_Update();

		monitor_parameters_update();

#if BUZZER_WARNING_
		warning_monitor_update();

		// Buzzer works with rc switch command unless code has other use of it.
		if(!monitor.is_battery_low && !monitor.is_temp_high){
			CK_BUZZER_CheckBuzzer();
		}

#else
		CK_BUZZER_CheckBuzzer();
#endif

	}
}

void monitor_parameters_update(void){

	if(isDataReady()){

		monitor.battery_sum += CK_ADC_GetLipoResult() * VOLT_CALIBRATION_MULTIPLIER; // unscaled lipo battery
		monitor.battery_average++;

		monitor.temp_sum += CK_ADC_GetTemperatureResult();
		monitor.temperature_average++;

		if(monitor.battery_average >= BATTERY_AVERAGE){
			monitor.battery_voltage = monitor.battery_sum / BATTERY_AVERAGE;
			monitor.battery_average = 0;
			monitor.battery_sum = 0.0f;

			if(monitor.battery_voltage <= BATTER_CRITICAL_VOLTAGE){
				monitor.is_battery_low = true;
			}
			else{
				monitor.is_battery_low = false;
			}
		}

		if(monitor.temperature_average >= TEMPERATURE_AVERAGE){
			monitor.temperature = monitor.temp_sum / TEMPERATURE_AVERAGE;
			monitor.temperature_average = 0;
			monitor.temp_sum = 0.0f;
			if(monitor.temperature >= CRITICAL_TEMP_MAX){
				monitor.is_temp_high = true;
			}
			else{
				monitor.is_temp_high = false;
			}
		}

	}
}

void warning_monitor_update(void){

	if(monitor.warning_sync.syncCounter >= monitor.warning_sync.syncRate){

		monitor.warning_sync.syncCounter = 0;

		// buzzer should not be in use to use this warning system
		if(monitor.is_battery_low || monitor.is_temp_high){
			CK_BUZZER_Toggle();
		}
		// This else is actually unnecessary because battery wont be high untill it is changed.
		else{
			CK_BUZZER_Disable();
		}
	}

}

void monitor_set_gyroacc_temp(float t){
	monitor.gyroacc_sensor_temperature = t;
}

float monitor_get_gyroacc_temp(void){
	return monitor.gyroacc_sensor_temperature;
}










