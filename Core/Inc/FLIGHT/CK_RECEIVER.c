
#include "DRIVERS/CK_GPIO.h"
#include "DRIVERS/CK_UART.h"
#include "DRIVERS/CK_TIME_HAL.h"
#include "DRIVERS/CK_CIRCULARBUFFER.h"
#include "DRIVERS/CK_BUZZER.h"
#include "DRIVERS/CK_SYSTEM.h"

#include "FLIGHT/CK_RECEIVER.h"
#include "FLIGHT/CK_RC.h"
#include "FLIGHT/CK_SBUS.h"
#include "FLIGHT/CK_LAND.h"
#include "FLIGHT/CK_CRSF.h"
#include "FLIGHT/CK_PWM.h"
#include "FLIGHT/CK_PID.h"

#include "COMMON/maths.h"

#include "COMMUNICATION/CK_PRINTER.h"

#define SWITCH_MAX_POSITION                 2000

#define SWITCH_MID_POSITION                 1500

#define SWITCH_MIN_POSITION                 1000

#define SWITCH_POSITION_MARGIN              200

#define NUM_OF_START_ARMING                 2

GPIO_TypeDef* GPIOx = GPIOC;

int startTime1, pulseReceived1;
int startTime2, pulseReceived2;
int startTime3, pulseReceived3;
int startTime4, pulseReceived4;
int startTime5, pulseReceived5;
int startTime6, pulseReceived6;

static bool rxSignalReceived = false;
static bool rxSignalReceivedValid = false;

uint32_t invalid_data_counter = 0;

int rxData[TOTAL_CHANNEL_COUNT];

CK_RC_Mode rx_mode;

RECEIVER_FLAGS_t flags;

OVERWRITE_FLAGS_t overwrite_flags;

RECEIVER_AUX_POS_t receiver_aux;

DEBUG_TIME_t receiver_debug;

static bool airmodeEnabled;
static bool airmodeIsActivated;
static bool throttleRaised = false;

void CK_RECEIVER_Init(CK_RC_Mode rx_md){

	rx_mode = rx_md;
	if(rx_mode == RX_PWM){

		#if USE_F4 == 1
		/*
		 *	SYSCFG_EXTICR1 (Any ports)Pins0-3 can be configured with SYSCFG->EXTICR[0]
		 *	SYSCFG_EXTICR2 (Any ports)Pins4-7 can be configured with SYSCFG->EXTICR[1]
		 *	SYSCFG_EXTICR3 (Any ports)Pins8-11 can be configured with SYSCFG->EXTICR[2]
		 *	SYSCFG_EXTICR4 (Any ports)Pins12-15 can be configured with SYSCFG->EXTICR[3]
		 *
		 */

		RCC->APB2ENR |= CK_RCC_SYSCFG_ENABLE;//SYSCFG Clock Enable

		CK_GPIO_ClockEnable(GPIOx);
		CK_GPIO_Init(GPIOx, ROLL, CK_GPIO_INPUT, CK_GPIO_NOAF, CK_GPIO_VERYHIGH, CK_GPIO_NOPUPD);
		CK_GPIO_Init(GPIOx, PITCH, CK_GPIO_INPUT, CK_GPIO_NOAF, CK_GPIO_VERYHIGH, CK_GPIO_NOPUPD);
		CK_GPIO_Init(GPIOx, YAW, CK_GPIO_INPUT, CK_GPIO_NOAF, CK_GPIO_VERYHIGH, CK_GPIO_NOPUPD);
		CK_GPIO_Init(GPIOx, THROTTLE, CK_GPIO_INPUT, CK_GPIO_NOAF, CK_GPIO_VERYHIGH, CK_GPIO_NOPUPD);
		CK_GPIO_Init(GPIOx, AUX1, CK_GPIO_INPUT, CK_GPIO_NOAF, CK_GPIO_VERYHIGH, CK_GPIO_NOPUPD);
		CK_GPIO_Init(GPIOx, AUX2, CK_GPIO_INPUT, CK_GPIO_NOAF, CK_GPIO_VERYHIGH, CK_GPIO_NOPUPD);

		SYSCFG->EXTICR[0] |= (2u<<12) | (2u<<8) | (2u<<4) | (2u<<0);	//Pin0,1,2,3 connected to PC
		SYSCFG->EXTICR[1] |= (2u<<4) | (2u<<0);							//Pin4,5 connected to PC
		EXTI->IMR |= (1u<<5)|(1u<<4)|(1u<<3)|(1u<<2)|(1u<<1)|(1u<<0);	//Line0,1,2,3,4 Interrupt Mask Request
		EXTI->EMR |= (1u<<5)|(1u<<4)|(1u<<3)|(1u<<2)|(1u<<1)|(1u<<0);	//Line0,1,2,3,4 Event Mask Request
		EXTI->RTSR |= (1u<<5)|(1u<<4)|(1u<<3)|(1u<<2)|(1u<<1)|(1u<<0);	//Rising edge detection for each line
		EXTI->FTSR |= (1u<<5)|(1u<<4)|(1u<<3)|(1u<<2)|(1u<<1)|(1u<<0);	//Falling edge detection for each line

		NVIC_SetPriority(EXTI0_IRQn, 1);
		NVIC_SetPriority(EXTI2_IRQn, 2);
		NVIC_SetPriority(EXTI3_IRQn, 3);
		NVIC_SetPriority(EXTI4_IRQn, 4);
		NVIC_SetPriority(EXTI9_5_IRQn, 5);

		NVIC_EnableIRQ(EXTI0_IRQn);
		NVIC_EnableIRQ(EXTI1_IRQn);
		NVIC_EnableIRQ(EXTI2_IRQn);
		NVIC_EnableIRQ(EXTI3_IRQn);
		NVIC_EnableIRQ(EXTI4_IRQn);
		NVIC_EnableIRQ(EXTI9_5_IRQn);
		#endif

	}

	else if(rx_mode == RX_SBUS){

	    CK_SBUS_Init(SBUS_INTERRUPT);

	}
	else if(rx_mode == RX_CRSF){

		CK_CRSF_Init(CRSF_INTERRUPT);

	}

    flags.FAILSAFE                     	= false;
    flags.ARMED                        	= false;
    flags.IS_FIRST_ARMING_DONE			= false;

    flags.ACRO_MODE                    	= false;
    flags.HORIZON_MODE                 	= false;
    flags.ANGLE_MODE                   	= false;
    flags.HEADFREE_MODE					= false;

    flags.BUZZER                       	= false;

    flags.ALTITUDE_HOLD                	= false;
    flags.LANDING                      	= false;

    flags.GPS_RESCUE                   	= false;
    flags.GPS_POS_HOLD                 	= false;
    flags.MAG_HOLD                     	= false;
    flags.BOX3D                     	= false;

    overwrite_flags.FORCE_TO_DISARM         = false;

    overwrite_flags.FORCE_TO_ACRO_MODE      = false;
    overwrite_flags.FORCE_TO_HORIZON_MODE   = false;
    overwrite_flags.FORCE_TO_ANGLE_MODE     = false;

    overwrite_flags.FORCE_TO_ALTITUDE_HOLD  = false;
    overwrite_flags.FORCE_TO_LANDING        = false;

    overwrite_flags.FORCE_TO_GPS_RESCUE     = false;
    overwrite_flags.FORCE_TO_GPS_POS_HOLD   = false;

    overwrite_flags.FORCE_TO_MAG_HOLD       = false;

    receiver_aux.aux1						= 0; // 0 is idle state until receiver sets it to 1, 2 and 3 is pos indicator
    receiver_aux.aux2						= 0;
    receiver_aux.aux3						= 0;
    receiver_aux.aux4						= 0;
    receiver_aux.aux5						= 0;
    receiver_aux.aux6						= 0;
    receiver_aux.aux7						= 0;
    receiver_aux.aux8						= 0;

	#ifdef USE_AIRMODE
    airmodeEnabled = true;
	#else
    airmodeEnabled = false;
	#endif

}

void CK_RECEIVER_WaitARM(void){

	int num_of_arm = 0;
	int num_of_disarm = 0;

	uint32_t temp_timer = CK_TIME_GetMicroSec();

	while(num_of_arm < NUM_OF_START_ARMING){

		CK_RECEIVER_Update(temp_timer);

		//First Arming
		if(flags.ARMED && num_of_arm == 0){
			num_of_arm++;
			CK_PRINTER_PrintlnString("FIRST ARM");
		}

		//First Disarming
		if(!flags.ARMED && num_of_disarm == 0 && num_of_arm == 1){
			num_of_disarm++;
			CK_PRINTER_PrintlnString("FIRST DISARM");
		}

		//Second Arming
		if(flags.ARMED && num_of_disarm == 1 && num_of_arm == 1){
			num_of_arm++;
		}

		CK_TIME_DelayMilliSec(1);
		temp_timer = CK_TIME_GetMicroSec();
	}

	flags.IS_FIRST_ARMING_DONE = true;
	CK_PRINTER_PrintlnString("FULL ARMED");

	// Do not start if throttle is not at minumum
	CK_PRINTER_PrintString("Checking Throttle IDLE ");
	int throttleDotCounter = 0;

	while(getRCDataRaw(THROTTLE) >= RECEIVER_MIN_THROTTLE){
		CK_RECEIVER_Update(temp_timer);
		CK_TIME_DelayMilliSec(1);
		temp_timer = CK_TIME_GetMicroSec();

		throttleDotCounter++;
		if(throttleDotCounter == 1000){
			throttleDotCounter = 0;
			CK_PRINTER_PrintString(".");
		}
	}

	CK_PRINTER_PrintlnString("");
	CK_PRINTER_PrintlnString("Throttle is at IDLE");

	CK_BUZZER_Tone1();
}

void CK_RECEIVER_Update(uint32_t current_time){

    #if defined(DEBUG_TIMING)
    receiver_debug.start_time = CK_TIME_GetMicroSec();
    #endif

    if(rx_mode == RX_PWM){

    }
    else if(rx_mode == RX_SBUS){

    	rxSignalReceived = CK_SBUS_Update(current_time);

    }
    else if(rx_mode == RX_CRSF){

    	rxSignalReceived = CK_CRSF_Update(current_time);

    }

    // Every data received is not valid so first check then set new data flag for feedforward etc.
    if(rxSignalReceived){

    	rxSignalReceivedValid = CK_RECEIVER_GetAndScaleChannels();

        if(rxSignalReceivedValid){

        	updateRcRefreshRate(CK_TIME_GetMicroSec(), rxSignalReceivedValid);

    		CK_RECEIVER_CheckAndSetFlags();

        }
        else{

        	invalid_data_counter++;

        }

    }

    const bool throttleActive = calculateThrottleStatus() != THROTTLE_LOW;
	const uint8_t throttlePercent = calculateThrottlePercentAbs();
	//const bool launchControlActive = isLaunchControlActive();
	const bool launchControlActive = false;
	static bool isAirmodeActive;

    if (flags.ARMED) {
        if (throttlePercent >= 25) {
            throttleRaised = true; // Latch true until disarm
        }
        if (isAirmodeEnabled() && !launchControlActive) {
            isAirmodeActive = throttleRaised;
        }
    } else {
        throttleRaised = false;
        isAirmodeActive = false;
    }

    // Note: If Airmode is enabled, on arming, iTerm and PIDs will be off until throttle exceeds the threshold (OFF while disarmed)
    // If not, iTerm will be off at low throttle, with pidStabilisationState determining whether PIDs will be active
    if (flags.ARMED && (isAirmodeActive || throttleActive || launchControlActive)) {
        pidSetItermReset(false);
        pidStabilisationState(PID_STABILISATION_ON);
    } else {
        pidSetItermReset(true);
        pidStabilisationState(pidProfile.pidAtMinThrottle ? PID_STABILISATION_ON : PID_STABILISATION_OFF);
    }

    #if defined(DEBUG_TIMING)
    receiver_debug.update_time = CK_TIME_GetMicroSec() - receiver_debug.start_time;
    #endif
}

bool isRxReceivingSignal(void)
{
    return rxSignalReceivedValid;
}

bool isAirmodeActivated(void)
{
    return airmodeIsActivated;
}

bool isAirmodeEnabled(void)
{
    return airmodeEnabled;
}

throttleStatus_e calculateThrottleStatus(void)
{
	if (getRCDataRaw(THROTTLE) < PWM_RANGE_MIN_CHECK) {
        return THROTTLE_LOW;
    }

    return THROTTLE_HIGH;
}

bool wasThrottleRaised(void)
{
    return throttleRaised;
}

// calculate the throttle stick percent - integer math is good enough here.
// returns negative values for reversed thrust in 3D mode
int8_t calculateThrottlePercent(void)
{
    uint8_t ret = 0;
    int channelData = constrain(getRCDataRaw(THROTTLE), PWM_RANGE_MIN, PWM_RANGE_MAX);

	ret = constrain(((channelData - PWM_RANGE_MIN_CHECK) * 100) / ((PWM_RANGE_MAX - RECEIVER_PWM_MARGINE) - PWM_RANGE_MIN_CHECK), 0, 100);

    return ret;
}

uint8_t calculateThrottlePercentAbs(void)
{
    return abs(calculateThrottlePercent());
}

uint32_t CK_RECEIVER_GetInvalidDataCounter(void){
	return invalid_data_counter;
}

bool CK_RECEIVER_GetAndScaleChannels(void){

	bool is_new_data_valid = false;

	/*
	 * If FAILSAFE is activated due to signal loss
	 * FAILSAFE_ACTIVE_FLAG is set in sbus process method
	 */

	//SBUS Channel Order is [THROTTLE ROLL PITCH YAW AUX1 AUX2 AUX3 ...]
	//RC   Channel Order is [ROLL PITCH YAW THROTTLE AUX1 AUX2 AUX3 ...]
	//so i reorder according to RC_CHANNEL order (see CK_Definitions.h)
	for(int channel = 0; channel < TOTAL_CHANNEL_COUNT; channel++){

		if(rx_mode == RX_SBUS){

		    if(channel == AUX12_RSSI){
	            rxData[channel] = scaleRange(CK_SBUS_GetChannelRaw(16), 0, 2000, 195, 1900);
	        }
		    else{
		        rxData[channel] = scaleRange(CK_SBUS_GetChannelRaw(channel+1), PWM_RANGE_MIN, PWM_RANGE_MAX, CK_SBUS_MIN_VALUE, CK_SBUS_MAX_VALUE);
		    }
		}

		else if(rx_mode == RX_CRSF){

			rxData[channel] = CK_CRSF_GetChannelRaw(channel+1);
		}

	}

	// Some package are not usefull because of the either short failsafe or
	// uncorrect data package. Therefore i will eliminate it here and
	// better failsafe detection will be implemented.
	// A failsafe condition will be used when the dbm is below some level

	// Min value is 999 and max is 2000, 880 is when failsafe or no data

	if(rxData[4] >= 999 || rxData[5] >= 999 || rxData[6] >= 999){
		#if SCOPE_CHECK_RX_PACKET == 1
		CK_GPIO_TogglePin(SCOPE_CHECK_GPIO, SCOPE_CHECK_GPIO_PIN);
		#endif

		// Manual arrangement
		// RCData Holds Values in [1000:2000] range
		// AETR Order
		// Radio Mixer order is also AETR as well for first 4 channel and rest is aux in order

		setRCDataRaw(ROLL, 		rxData[0]);
		setRCDataRaw(PITCH, 	rxData[1]);
		setRCDataRaw(THROTTLE, 	rxData[2]);
		setRCDataRaw(YAW, 		rxData[3]);
		setRCDataRaw(AUX1, 		rxData[4]);
		setRCDataRaw(AUX2, 		rxData[5]);
		setRCDataRaw(AUX3, 		rxData[6]);
		setRCDataRaw(AUX4, 		rxData[7]);
		setRCDataRaw(AUX5, 		rxData[8]);
		setRCDataRaw(AUX6, 		rxData[9]);
		setRCDataRaw(AUX7, 		rxData[10]);
		setRCDataRaw(AUX8, 		rxData[11]);
		setRCDataRaw(AUX9, 		rxData[12]);
		setRCDataRaw(AUX10, 	rxData[13]);
		setRCDataRaw(AUX11, 	rxData[14]);
		setRCDataRaw(AUX12_RSSI,rxData[15]);	// rssi scaled between 0 to 2000

		is_new_data_valid = true;
	}
	else{
		is_new_data_valid = false;
	}

	return is_new_data_valid;

}

void CK_RECEIVER_CheckAndSetFlags(void){

	// ***************FAILSAFE***************

	if(rx_mode == RX_SBUS){

		if(CK_SBUS_GetChannelRaw(SBUS_FAILSAFE_CHANNEL)){
			flags.FAILSAFE = true;     // FAILSAFE ACTIVE
		}
		else{
			flags.FAILSAFE = false;    // FAILSAFE INACTIVE
		}
	}
	else if(rx_mode == RX_CRSF){

		if(CK_CRSF_GetRSSI_dBm() < (CRSF_RSSI_MIN + 10)){
			flags.FAILSAFE = true;     // FAILSAFE ACTIVE
		}
		else{
			flags.FAILSAFE = false;    // FAILSAFE INACTIVE
		}
	}


	// ***************AUX1***************
	// 2-WAY SWITHCH -> Idle(1000), End(2000)

	// (aux < 1200) && (aux > 950)
    if((getRCDataRaw(AUX1) < (SWITCH_MIN_POSITION + SWITCH_POSITION_MARGIN)) && (getRCDataRaw(AUX1) > (SWITCH_MIN_POSITION - SWITCH_POSITION_MARGIN))){
        flags.ARMED = false;       // DISARMED
        receiver_aux.aux1	 = 1;
    }

    // (aux > 1800) && (aux < 2050)
    else if((getRCDataRaw(AUX1) > (SWITCH_MAX_POSITION - SWITCH_POSITION_MARGIN)) && (getRCDataRaw(AUX1) < (SWITCH_MAX_POSITION + SWITCH_POSITION_MARGIN))){
	    flags.ARMED = true;        // ARMED
	    receiver_aux.aux1	 = 2;
	}

	// ***************AUX2***************
	// 2-WAY SWITHCH -> Idle(1000), End(2000)

	// (aux < 1200) && (aux > 950)
	if((getRCDataRaw(AUX2) < (SWITCH_MIN_POSITION + SWITCH_POSITION_MARGIN)) && (getRCDataRaw(AUX2) > (SWITCH_MIN_POSITION - SWITCH_POSITION_MARGIN))){
		flags.BUZZER = false;  // BUZZER INACTIVE
		receiver_aux.aux2	  = 1;
	}

	// (aux > 1800) && (aux < 2050)
	else if((getRCDataRaw(AUX2) > (SWITCH_MAX_POSITION - SWITCH_POSITION_MARGIN)) && (getRCDataRaw(AUX2) < (SWITCH_MAX_POSITION + SWITCH_POSITION_MARGIN))){
		flags.BUZZER = true;   // BUZZER ACTIVE
		receiver_aux.aux2	  = 2;
	}

	// ***************AUX3***************
	// 3-WAY SWITHCH -> Idle(1000), Middle(1500), End(2000)

	// (aux < 1200) && (aux > 950)
	if((getRCDataRaw(AUX3) < (SWITCH_MIN_POSITION + SWITCH_POSITION_MARGIN)) && (getRCDataRaw(AUX3) > (SWITCH_MIN_POSITION - SWITCH_POSITION_MARGIN))){
		flags.ACRO_MODE        = true;     // ACRO_MODE
		flags.HORIZON_MODE     = false;
		flags.ANGLE_MODE       = false;
		receiver_aux.aux3	   = 1;
	}

	// (aux < 1600) && (aux > 1400)
	else if((getRCDataRaw(AUX3) < (SWITCH_MID_POSITION + (SWITCH_POSITION_MARGIN/2))) && (getRCDataRaw(AUX3) > (SWITCH_MID_POSITION - (SWITCH_POSITION_MARGIN/2)))){
		flags.ACRO_MODE        = false;
		flags.HORIZON_MODE     = true;     // HORIZON_MODE
		flags.ANGLE_MODE       = false;
		receiver_aux.aux3	   = 2;
	}

	// (aux > 1800) && (aux < 2050)
	else if((getRCDataRaw(AUX3) > (SWITCH_MAX_POSITION - SWITCH_POSITION_MARGIN)) && (getRCDataRaw(AUX3) < (SWITCH_MAX_POSITION + SWITCH_POSITION_MARGIN))){
		flags.ACRO_MODE        = false;
		flags.HORIZON_MODE     = false;
		flags.ANGLE_MODE       = true;     // LEVEL_MODE
		receiver_aux.aux3	   = 3;
	}


	// ***************AUX4***************
	// 3-WAY SWITHCH -> Idle(1000), Middle(1500), End(2000)

	// (aux < 1200) && (aux > 950)
	if((getRCDataRaw(AUX4) < (SWITCH_MIN_POSITION + SWITCH_POSITION_MARGIN)) && (getRCDataRaw(AUX4) > (SWITCH_MIN_POSITION - SWITCH_POSITION_MARGIN))){
		flags.FAILSAFE  = false;
		flags.MAG_HOLD  	 = false;
		receiver_aux.aux4	 = 1;
	}

	// (aux < 1600) && (aux > 1400)
	else if((getRCDataRaw(AUX4) < (SWITCH_MID_POSITION + (SWITCH_POSITION_MARGIN/2))) && (getRCDataRaw(AUX4) > (SWITCH_MID_POSITION - (SWITCH_POSITION_MARGIN/2)))){
		flags.FAILSAFE  = false;
		flags.MAG_HOLD  	 = false;
		receiver_aux.aux4	 = 2;
	}

	// (aux > 1800) && (aux < 2050)
	else if((getRCDataRaw(AUX4) > (SWITCH_MAX_POSITION - SWITCH_POSITION_MARGIN)) && (getRCDataRaw(AUX4) < (SWITCH_MAX_POSITION + SWITCH_POSITION_MARGIN))){
		flags.FAILSAFE  = false;
		flags.MAG_HOLD  	 = false;
		receiver_aux.aux4	 = 3;
	}


	// ***************AUX7***************
	// 3-WAY SWITHCH -> Idle(1000), Middle(1500), End(2000)

	// (aux < 1200) && (aux > 950)
	if((getRCDataRaw(AUX7) < (SWITCH_MIN_POSITION + SWITCH_POSITION_MARGIN)) && (getRCDataRaw(AUX7) > (SWITCH_MIN_POSITION - SWITCH_POSITION_MARGIN))){
		flags.ALTITUDE_HOLD    = false;
		flags.LANDING          = false;
		receiver_aux.aux7	   = 1;
	}

	// (aux < 1600) && (aux > 1400)
	else if((getRCDataRaw(AUX7) < (SWITCH_MID_POSITION + (SWITCH_POSITION_MARGIN/2))) && (getRCDataRaw(AUX7) > (SWITCH_MID_POSITION - (SWITCH_POSITION_MARGIN/2)))){
		flags.ALTITUDE_HOLD    = true;     // ALTITUDE HOLD
		flags.LANDING          = false;
		receiver_aux.aux7	   = 2;
	}

	// (aux > 1800) && (aux < 2050)
	else if((getRCDataRaw(AUX7) > (SWITCH_MAX_POSITION - SWITCH_POSITION_MARGIN)) && (getRCDataRaw(AUX7) < (SWITCH_MAX_POSITION + SWITCH_POSITION_MARGIN))){
		flags.ALTITUDE_HOLD    = false;
		flags.LANDING          = true;     // LANDING
		receiver_aux.aux7	   = 3;
	}


	// ***************AUX8***************
	// 3-WAY SWITHCH -> Idle(1000), Middle(1500), End(2000)

	// (aux < 1200) && (aux > 950)
	if((getRCDataRaw(AUX8) < (SWITCH_MIN_POSITION + SWITCH_POSITION_MARGIN)) && (getRCDataRaw(AUX8) > (SWITCH_MIN_POSITION - SWITCH_POSITION_MARGIN))){
		flags.GPS_RESCUE       = false;
		flags.GPS_POS_HOLD     = false;
		receiver_aux.aux8	   = 1;
	}

	// (aux < 1600) && (aux > 1400)
	else if((getRCDataRaw(AUX8) < (SWITCH_MID_POSITION + (SWITCH_POSITION_MARGIN/2))) && (getRCDataRaw(AUX8) > (SWITCH_MID_POSITION - (SWITCH_POSITION_MARGIN/2)))){
		flags.GPS_RESCUE       = false;
		flags.GPS_POS_HOLD     = true;       // GPS POSITION HOLD
		receiver_aux.aux8	   = 2;
	}

	// (aux > 1800) && (aux < 2050)
	else if((getRCDataRaw(AUX8) > (SWITCH_MAX_POSITION - SWITCH_POSITION_MARGIN)) && (getRCDataRaw(AUX8) < (SWITCH_MAX_POSITION + SWITCH_POSITION_MARGIN))){
		flags.GPS_RESCUE       = true;       // GPS RESCUE
		flags.GPS_POS_HOLD     = false;
		receiver_aux.aux8	   = 3;
	}


    // **************FLAG OVERWRITE*********************
    // In this way the code will not change main flags received by radio.
    // The code will change overwrite_flags and then flags will be set here accordingly.
    // Whole code will have the same flags.

    if(overwrite_flags.FORCE_TO_DISARM){
        flags.ARMED = false;
    }

    // Force write flight mode related flags.
    if(overwrite_flags.FORCE_TO_ACRO_MODE){
        flags.ACRO_MODE        = true;
        flags.HORIZON_MODE     = false;
        flags.ANGLE_MODE       = false;
    }
    else if(overwrite_flags.FORCE_TO_HORIZON_MODE){
        flags.ACRO_MODE        = false;
        flags.HORIZON_MODE     = true;
        flags.ANGLE_MODE       = false;
    }
    else if(overwrite_flags.FORCE_TO_ANGLE_MODE){
        flags.ACRO_MODE        = false;
        flags.HORIZON_MODE     = false;
        flags.ANGLE_MODE       = true;
    }

    // Force write altitude related flags.
    if(overwrite_flags.FORCE_TO_ALTITUDE_HOLD){
        flags.ALTITUDE_HOLD    = true;
        flags.LANDING          = false;
    }
    else if(overwrite_flags.FORCE_TO_LANDING){
        flags.ALTITUDE_HOLD    = false;
        flags.LANDING          = true;
    }

    // Force write gps related flags.
    // Gps rescue uses landing and landing uses gps pos hold
    // so gps rescue and gps pos hold works together when drone arrives home.
    // do not disactivate other while activating one.
    if(overwrite_flags.FORCE_TO_GPS_RESCUE){
        flags.GPS_RESCUE       = true;
    }
    else if(overwrite_flags.FORCE_TO_GPS_POS_HOLD){
        flags.GPS_POS_HOLD     = true;
    }


    if(overwrite_flags.FORCE_TO_MAG_HOLD){
        flags.MAG_HOLD         = true;
    }


	// ***************FAILSAFE MODE OVERWRTIE***************

	if(flags.FAILSAFE && flags.IS_FIRST_ARMING_DONE){

	    // If failsafe the mode will be determined here so previous flags will be ignored */

		if(overwrite_flags.FORCE_TO_DISARM){
			flags.ARMED = false;
		}
		else{
			flags.ARMED = true;     // Drone should be armed until landing
		}

		flags.ACRO_MODE 		= false;
		flags.HORIZON_MODE 		= false;
		flags.ANGLE_MODE 	    = true;

		flags.BUZZER 		    = true;

		flags.GPS_RESCUE 	    = false;    // later activate this when gps rescue is tested and works.
		flags.MAG_HOLD   	    = false;

		flags.ALTITUDE_HOLD    	= false;
		flags.LANDING		    = true;     // LANDING

	}
}

uint8_t CK_RECEIVER_isArmed(void){

    if(flags.ARMED){
        return 1; // ARMED
    }
    return 0; // NOT ARMED

}

uint8_t isFailsafeActive(void){

    if(flags.FAILSAFE){
        return 1; // FAILSAFE ACTIVE
    }
    return 0; // NO FAILSAFE ACTIVE

}

uint8_t CK_RECEIVER_GetFlightMode(void){

    if(flags.ACRO_MODE){
        return 1;
    }
    else if(flags.HORIZON_MODE){
        return 2;
    }
    else if(flags.ANGLE_MODE){
        return 3;
    }
    return 0; // Error.

}

uint8_t CK_RECEIVER_GetAltitudeMode(void){

    if(flags.ALTITUDE_HOLD){
        return 1;
    }
    else if(flags.LANDING){
        return 2;
    }
    return 0; // Error.

}

uint8_t CK_RECEIVER_GetNavigationMode(void){

    if(flags.GPS_RESCUE){
        return 1;
    }
    else if(flags.GPS_POS_HOLD){
        return 2;
    }
    else if(flags.MAG_HOLD){
        return 3;
    }
    return 0; // Error.

}

#if USE_F4 == 1
//RECEIVER PWM PIN0 Handler
void EXTI0_IRQHandler(void){

	if(((GPIOx->IDR & (1u<<ROLL))>>ROLL) == 1){
		startTime1 = CK_TIME_GetMicroSec();

	}
	else if(((GPIOx->IDR & (1u<<ROLL))>>ROLL) == 0){
		if(startTime1 != 0){
			pulseReceived1 = CK_TIME_GetMicroSec() - startTime1;
			setRCDataRaw(ROLL, pulseReceived1);
			startTime1 = 0;
		}
	}
	EXTI->PR |= 1u<<ROLL;//Clear Interrupt
}

//RECEIVER PWM PIN1 Handler
void EXTI1_IRQHandler(void){


	if(((GPIOx->IDR & (1u<<PITCH))>>PITCH) == 1){
		startTime2 = CK_TIME_GetMicroSec();

	}
	else if(((GPIOx->IDR & (1u<<PITCH))>>PITCH) == 0){
		if(startTime2 != 0){
			pulseReceived2 = CK_TIME_GetMicroSec() - startTime2;
			setRCDataRaw(PITCH, pulseReceived2);
			startTime2 = 0;
		}
	}
	EXTI->PR |= 1u<<PITCH;//Clear Interrupt

}

//RECEIVER PWM PIN2 Handler
void EXTI2_IRQHandler(void){

	if(((GPIOx->IDR & (1u<<YAW))>>YAW) == 1){
		startTime3 = CK_TIME_GetMicroSec();

	}
	else if(((GPIOx->IDR & (1u<<YAW))>>YAW) == 0){
		if(startTime3 != 0){
			pulseReceived3 = CK_TIME_GetMicroSec() - startTime3;
			setRCDataRaw(YAW, pulseReceived3);
			startTime3 = 0;
		}
	}
	EXTI->PR |= 1u<<YAW;//Clear Interrupt
}

//RECEIVER PWM PIN3 Handler
void EXTI3_IRQHandler(void){

	if(((GPIOx->IDR & (1u<<THROTTLE))>>THROTTLE) == 1){
		startTime4 = CK_TIME_GetMicroSec();

	}
	else if(((GPIOx->IDR & (1u<<THROTTLE))>>THROTTLE) == 0){
		if(startTime4 != 0){
			pulseReceived4 = CK_TIME_GetMicroSec() - startTime4;
			setRCDataRaw(THROTTLE, pulseReceived4);
			startTime4 = 0;
		}
	}
	EXTI->PR |= 1u<<THROTTLE;//Clear Interrupt

}

//RECEIVER PWM PIN4 Handler
void EXTI4_IRQHandler(void){

	if(((GPIOx->IDR & (1u<<AUX1))>>AUX1) == 1){
		startTime5 = CK_TIME_GetMicroSec();

	}
	else if(((GPIOx->IDR & (1u<<AUX1))>>AUX1) == 0){
		if(startTime5 != 0){
			pulseReceived5 = CK_TIME_GetMicroSec() - startTime5;
			setRCDataRaw(AUX1, pulseReceived5);
			startTime5 = 0;
		}
	}
	EXTI->PR |= 1u<<AUX1;//Clear Interrupt

}

//RECEIVER PWM PIN5-9 Handler
void EXTI9_5_IRQHandler(void){

	if(((GPIOx->IDR & (1u<<AUX2))>>AUX2) == 1){
		startTime6 = CK_TIME_GetMicroSec();

	}
	else if(((GPIOx->IDR & (1u<<AUX2))>>AUX2) == 0){
		if(startTime6 != 0){
			pulseReceived6 = CK_TIME_GetMicroSec() - startTime6;
			setRCDataRaw(AUX2, pulseReceived6);
			startTime6 = 0;
		}
	}
	EXTI->PR |= 1u<<AUX2;//Clear Interrupt

}
#endif
