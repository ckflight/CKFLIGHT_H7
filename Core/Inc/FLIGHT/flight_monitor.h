/*
 * battery_monitor.h
 *
 *  Created on: Feb 8, 2025
 *      Author: ck
 */

#ifndef INC_FLIGHT_FLIGHT_MONITOR_H_
#define INC_FLIGHT_FLIGHT_MONITOR_H_

#include "CK_DEFINITIONS.h"

void flight_monitor_init(uint32_t monitorT, uint32_t mainT);
void flight_monitor_update(void);
void monitor_parameters_update(void);
void warning_monitor_update(void);
void monitor_set_gyroacc_temp(float t);
float monitor_get_gyroacc_temp(void);

#endif /* INC_FLIGHT_FLIGHT_MONITOR_H_ */
