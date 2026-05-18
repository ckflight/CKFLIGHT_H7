
#ifndef CK_RECEIVER_H_
#define CK_RECEIVER_H_

#include "CK_DEFINITIONS.h"

#define PWM_RANGE_MIN 1000
#define PWM_RANGE_MAX 2000
#define PWM_RANGE (PWM_RANGE_MAX - PWM_RANGE_MIN)
#define PWM_RANGE_MIDDLE (PWM_RANGE_MIN + (PWM_RANGE / 2))
#define PWM_RANGE_MIN_CHECK		1020

#define RECEIVER_MIN_THROTTLE	1100
#define RECEIVER_MAX_THROTTLE	2000

#define RECEIVER_PWM_MARGINE	50

#define CONTROL_RATE_CONFIG_RATE_LIMIT_MIN  200
#define CONTROL_RATE_CONFIG_RATE_LIMIT_MAX  1998

#define CK_RCC_SYSCFG_ENABLE	1u << 14;

// If a flight feature needs to change radio switch option
// then this force flag will be true to ignore receiver switch untill
// the mode is done.
// For example if i enter to gps rescue mode then the drone needs to
// get into level mode, altitude hold mode etc. Even the radio sends acro
// it will be ignored during the feature so overwrite flag will be used.

typedef enum {
    THROTTLE_LOW = 0,
    THROTTLE_HIGH
} throttleStatus_e;

typedef struct{

    bool FAILSAFE;
    bool ARMED;
    bool IS_FIRST_ARMING_DONE;

    bool ACRO_MODE;
    bool HORIZON_MODE;
    bool ANGLE_MODE;
    bool HEADFREE_MODE;

    bool BUZZER;

    bool ALTITUDE_HOLD;

    bool LANDING;

    bool GPS_RESCUE;

    bool GPS_POS_HOLD;

    bool MAG_HOLD;

    bool BOX3D;

    bool CRASH_FLIP;


}RECEIVER_FLAGS_t;

typedef struct{

    bool FORCE_TO_DISARM;

    bool FORCE_TO_ACRO_MODE;

    bool FORCE_TO_HORIZON_MODE;

    bool FORCE_TO_ANGLE_MODE;

    bool FORCE_TO_ALTITUDE_HOLD;

    bool FORCE_TO_LANDING;

    bool FORCE_TO_GPS_RESCUE;

    bool FORCE_TO_GPS_POS_HOLD;

    bool FORCE_TO_MAG_HOLD;


}OVERWRITE_FLAGS_t;

typedef struct{

	uint8_t aux1;
	uint8_t aux2;
	uint8_t aux3;
	uint8_t aux4;
	uint8_t aux5;
	uint8_t aux6;
	uint8_t aux7;
	uint8_t aux8;

}RECEIVER_AUX_POS_t;

typedef enum{

    RX_PWM                 = 0,
    RX_SBUS                = 1,
	RX_CRSF				   = 2

}CK_RC_Mode;

extern RECEIVER_FLAGS_t flags;

extern OVERWRITE_FLAGS_t overwrite_flags;

extern RECEIVER_AUX_POS_t receiver_aux;

void CK_RECEIVER_Init(CK_RC_Mode rx_md);

void CK_RECEIVER_WaitARM(void);

void CK_RECEIVER_Update(uint32_t current_time);

bool isRxReceivingSignal(void);

bool isAirmodeActivated(void);

bool isAirmodeEnabled(void);

throttleStatus_e calculateThrottleStatus(void);

bool wasThrottleRaised(void);

int8_t calculateThrottlePercent(void);

uint8_t calculateThrottlePercentAbs(void);

uint32_t CK_RECEIVER_GetInvalidDataCounter(void);

bool CK_RECEIVER_GetAndScaleChannels(void);

void CK_RECEIVER_CheckAndSetFlags(void);

uint8_t CK_RECEIVER_isArmed(void);

uint8_t isFailsafeActive(void);

uint8_t CK_RECEIVER_GetFlightMode(void);

uint8_t CK_RECEIVER_GetAltitudeMode(void);

uint8_t CK_RECEIVER_GetNavigationMode(void);

#endif /* CK_RECEIVER_H_ */
