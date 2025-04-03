#ifndef INC_FLIGHT_SIMPLIFIED_TUNING_H_
#define INC_FLIGHT_SIMPLIFIED_TUNING_H_

#include "CK_DEFINITIONS.h"
#include "FLIGHT/CK_PID.h"
#include "MOTION/CK_GYRO.h"

#define SIMPLIFIED_TUNING_PIDS_MIN 		0
#define SIMPLIFIED_TUNING_FILTERS_MIN 	10
#define SIMPLIFIED_TUNING_MAX 			200
#define SIMPLIFIED_TUNING_DEFAULT 		100
#define SIMPLIFIED_TUNING_D_DEFAULT 	100

typedef enum {
    PID_SIMPLIFIED_TUNING_OFF = 0,
    PID_SIMPLIFIED_TUNING_RP,
    PID_SIMPLIFIED_TUNING_RPY,
    PID_SIMPLIFIED_TUNING_MODE_COUNT,
} pidSimplifiedTuningMode_e;

void calculateNewPidValues(pidProfile_t *pidProfile);

void applySimplifiedTuning(pidProfile_t *pidProfile, gyroSensor_t *gyroConfig);

void applySimplifiedTuningPids(pidProfile_t *pidProfile);
void applySimplifiedTuningDtermFilters(pidProfile_t *pidProfile);
void applySimplifiedTuningGyroFilters(gyroSensor_t *gyroConfig);

void disableSimplifiedTuning(pidProfile_t *pidProfile, gyroSensor_t *gyroConfig);


#endif

