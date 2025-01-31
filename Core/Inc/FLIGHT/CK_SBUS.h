
#ifndef CK_SBUS_H_
#define CK_SBUS_H_

#include "CK_DEFINITIONS.h"

typedef enum
{
    SBUS_INTERRUPT,
    SBUS_POLLING,

}SBUS_Method;

#define CK_SBUS_MIN_VALUE           172

#define CK_SBUS_MAX_VALUE           1811

#define SBUS_STARTBYTE         		0x0F

#define SBUS_ENDBYTE           		0x00

#define SBUS_PACKET_SIZE            25 // 1 Start Byte + 23 Byte Payload + 1 End byte

#define SBUS_FAILSAFE_INACTIVE 		0

#define SBUS_FAILSAFE_ACTIVE   		1

#define SBUS_FAILSAFE_CHANNEL		19 // 0 to 18 makes 19 channels

#define SBUS_CHECK_INTERVAL			11000 // in uSec

void CK_SBUS_Init(SBUS_Method method);

bool CK_SBUS_Update(uint32_t current_time);

void CK_SBUS_Process(void);

void CK_SBUS_Decode(void);

void CK_SBUS_NewByte(uint8_t data);

bool CK_SBUS_IsReady(void);

int CK_SBUS_GetChannelRaw(int channel);

#endif /* CK_SBUS_H_ */
