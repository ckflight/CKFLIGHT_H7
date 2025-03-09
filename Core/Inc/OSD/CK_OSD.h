
#ifndef CK_OSD_H_
#define CK_OSD_H_

#include "CK_DEFINITIONS.h"

typedef struct{

    uint16_t gps_distanceToDestination;

    uint16_t gps_headingToDestination;

    uint16_t gps_headingOfMotion;

    uint16_t gps_groundSpeed;

    int32_t estimatedAltitude;

    uint8_t gpsNumOfSat;

    uint8_t gpsSatFix;

    uint16_t rssi;

    int16_t rssi_dBm;

    uint8_t rssi_link_quality;

    uint8_t currentFlightMode;

    uint8_t currentNavigationMode;

    uint8_t currentAltitudeMode;

    uint8_t isArmed;

    uint8_t isFailSafe;

    uint16_t freqResult;

    uint16_t system_percent;

    uint16_t mainLoopTime;

    uint16_t imu_heading;

    uint8_t pid_roll[5];

	uint8_t pid_pitch[5];

	uint8_t pid_yaw[5];

	uint8_t is_adjustment_on;

	uint8_t cpu_core_temperature;

	uint16_t tpa_breakpoint;

	uint8_t tpa_rate;

	uint16_t voltage;

	uint16_t current;

	syncTimer_t sync;

}OSD_PACKET_s;

extern OSD_PACKET_s osd_packet;

void CK_OSD_Init(uint32_t osdT, uint32_t mainT);

void CK_OSD_Update(uint32_t currentTime, uint32_t loopTime);

void CK_OSD_DJI_PacketSequence(void);

void CK_OSD_DJI_FormPacket(void);

void CK_OSD_DJI_InitLocationParameters(void);

void CK_OSD_SetFlightData(void);

#if USE_DMA_OSD
void CK_OSD_SendPacketDMA(void);
#endif

void CK_OSD_WriteUartBuffer(void);

void CK_OSD_SendPacketPolling(void);

void CK_OSD_SendPacketInterrupt(void);

void CK_OSD_ResetBuffer(void);

#endif /* CK_OSD_H_ */
