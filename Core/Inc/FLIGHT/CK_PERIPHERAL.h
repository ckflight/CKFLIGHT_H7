
#ifndef INC_FLIGHT_CK_PERIPHERAL_H_
#define INC_FLIGHT_CK_PERIPHERAL_H_

#include "CK_DEFINITIONS.h"

extern TIM_HandleTypeDef htim_main_interrupt;

void CK_PERIPHERAL_Init(targetFreq_e target_period);

void CK_PERIPHERAL_MainInterruptInit(targetFreq_e target_period);

void CK_PERIPHERAL_StartInterrupt(void);

#endif /* INC_FLIGHT_CK_PERIPHERAL_H_ */
