/*
 * battery_monitor.h
 *
 *  Created on: Feb 8, 2025
 *      Author: ck
 */

#ifndef INC_FLIGHT_FLIGHT_MONITOR_H_
#define INC_FLIGHT_FLIGHT_MONITOR_H_

#include "CK_DEFINITIONS.h"

//TODO: Make the 'cell full' voltage user adjustable
#define CELL_VOLTAGE_FULL_CV 420

#define WARNING_TIME_US					500000U
#define BATTER_CRITICAL_VOLTAGE			21.0f
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

	uint16_t vbatwarningcellvoltage;        // warning voltage per cell, this triggers battery warning alarm, in 0.01V units, default is 350 (3.50V)

}flight_monitor_s;

extern flight_monitor_s monitor;

void flight_monitor_init(uint32_t monitorT, uint32_t mainT);
void flight_monitor_update(void);
void monitor_parameters_update(void);
void warning_monitor_update(void);
void monitor_set_gyroacc_temp(float t);
float monitor_get_gyroacc_temp(void);
#if defined(USE_BATTERY_VOLTAGE_SAG_COMPENSATION)
uint16_t getBatterySagCellVoltage(void);
#endif

#endif /* INC_FLIGHT_FLIGHT_MONITOR_H_ */
