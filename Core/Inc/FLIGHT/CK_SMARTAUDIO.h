
#ifndef INC_FLIGHT_CK_SMARTAUDIO_H_
#define INC_FLIGHT_CK_SMARTAUDIO_H_

#include "CK_DEFINITIONS.h"

typedef enum {
  BAND_A 	= 0,
  BAND_B 	= 1,
  BAND_E 	= 2,
  AIRWAVE 	= 3,
  RACEBAND 	= 4

}SMARTAUDIO_BAND;

typedef enum {
  CH1 	= 0,
  CH2 	= 1,
  CH3 	= 2,
  CH4 	= 3,
  CH5 	= 4,
  CH6 	= 5,
  CH7 	= 6,
  CH8 	= 7

}SMARTAUDIO_CHANNEL;

typedef enum {
	mW_25 	= 0,
	mW_100 	= 1,
	mW_400 	= 2,
	mW_1000 = 3,

}SMARTAUDIO_POWER;

typedef enum {
	CYCLONE,
	TBS_UNIFY_PRO32

}VTX_Type;

void CK_SMARTAUDIO_Init(SMARTAUDIO_BAND band, SMARTAUDIO_CHANNEL channel, SMARTAUDIO_POWER power, VTX_Type type);

void CK_SMARTAUDIO_Update(void);

void CK_SMARTAUDIO_TX_Packet(uint8_t cmd, uint16_t value);

bool CK_SMARTAUDIO_RX_Packet(uint8_t cmd, uint8_t size);

bool CK_SMARTAUDIO_DecodePacket(uint8_t cmd, uint8_t size);

uint8_t CK_SMARTAUDIO_Calculate_CRC8(uint8_t* buffer, uint8_t start_idx, uint8_t end_idx);



#endif /* INC_FLIGHT_CK_SMARTAUDIO_H_ */
