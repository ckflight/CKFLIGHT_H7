
#ifndef INC_DRIVERS_CK_RGB_H_
#define INC_DRIVERS_CK_RGB_H_

#include "CK_DEFINITIONS.h"

// 24 bit order = 8 bit G + 8 bit R + 8 bit B

typedef enum{
						//00GRB
	COLOR_WHITE 	= 0x00FFFFFF,
	COLOR_BLUE 		= 0x000000CC,
	COLOR_CYAN 		= 0x00CC00CC,
	COLOR_GREEN 	= 0x00CC0000,
	COLOR_RED 		= 0x0000CC00,
	COLOR_ORANGE	= 0x0080FF00,
	COLOR_ORANGE2	= 0x0066CC00,
	COLOR_BLUE2		= 0x000000CD,
	COLOR_NONE

}RGB_COLOR_;

void CK_RGB_Init(TIM_TypeDef* rgb_tim_, DMA_TypeDef* rgb_dma_, DMA_Stream_TypeDef* rgb_dma_stream_ ,RGB_COLOR_ color);

void CK_RGB_LoadDmaBuffer(uint32_t *dmaBuffer, int led , uint32_t packet);

void CK_RGB_Enable(void);

void CK_RGB_Start(int num);

#endif /* INC_DRIVERS_CK_RGB_H_ */
