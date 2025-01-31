
#ifndef INC_CK_SYSTEM_H_
#define INC_CK_SYSTEM_H_

#include "CK_DEFINITIONS.h"

extern uint32_t F_CPU; 											    // CK_TIME_HAL uses

typedef enum{

	SYSTEM_CLK_480MHz,

	SYSTEM_CLK_400MHz

}systemClock_e;

void CK_SYSTEM_SetSystemClock(systemClock_e clk);

uint32_t CK_SYSTEM_GetAPB1Clock(void);

uint32_t CK_SYSTEM_GetAPB2Clock(void);

uint32_t CK_SYSTEM_GetHCLKClock(void);

uint32_t CK_SYSTEM_GetPCLK1Clock(void);

uint32_t CK_SYSTEM_GetPCLK2Clock(void);

void CK_SYSTEM_TIMER_ClockEnable(TIM_TypeDef* timer);

uint32_t CK_SYSTEM_GetTIMERClock(TIM_TypeDef* timer);

uint32_t CK_SYSTEM_GetUARTClock(USART_TypeDef* uart);


#endif /* INC_CK_SYSTEM_H_ */
