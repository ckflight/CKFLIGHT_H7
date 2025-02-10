
#ifndef CK_GPS_H_
#define CK_GPS_H_

#include "CK_DEFINITIONS.h"
#include "DRIVERS/CK_CIRCULARBUFFER.h"

#define GPS_DEGREES_DIVIDER 10000000L
#define EARTH_ANGLE_TO_CM (111.3195f * 1000 * 100 / GPS_DEGREES_DIVIDER) // 1.113195 cm per latitude unit at the equator (111.3195km/deg)
#define GPS_X 1
#define GPS_Y 0
#define GPS_MIN_SAT_COUNT 4     // number of sats to trigger low sat count sanity check

typedef struct gpsSensor_s{

	int         isGpsInit;

    int32_t     current_lon;
    int32_t     current_lat;
    int32_t     destination_lat;
    int32_t     destination_lon;

    int32_t     current_heightEllipsoid;
    int32_t     current_heightSeaLevel;
    int32_t     onGround_heightSeaLevel;
    int32_t     heightDifference;

    uint8_t     numOfSattelite;
    uint8_t     satteliteFix;
    bool		isSatFixed;

    uint32_t    distanceToDestination;
    int32_t     headingToDestination;
    int32_t     groundCourse;
    int32_t     headingOfVehicle;
    int32_t     groundSpeed;

    int         lat_deg;
    int         lat_min;
    float       lat_sec;

    int         lon_deg;
    int         lon_min;
    float       lon_sec;

}gpsSensor_t;

extern gpsSensor_t gps;

typedef struct UBX_NAV_POSLLH_t{
    uint8_t   nav_posllh_cls;
    uint8_t   nav_posllh_id;
    uint16_t  nav_posllh_len;
    uint32_t  nav_posllh_iTOW;
    int32_t   nav_posllh_lon;
    int32_t   nav_posllh_lat;
    int32_t   nav_posllh_height;
    int32_t   nav_posllh_hMSL;
    uint32_t  nav_posllh_hAcc;
    uint32_t  nav_posllh_vAcc;
}UBX_NAV_POSLLH_s;

typedef struct UBX_NAV_VELNED_t{
    uint8_t   nav_velned_cls;
    uint8_t   nav_velned_id;
    uint16_t  nav_velned_len;
    uint32_t  nav_velned_iTOW;
    int32_t   nav_velned_ve1N;
    int32_t   nav_velned_ve1E;
    int32_t   nav_velned_ve1D;
    uint32_t  nav_velned_speed;
    uint32_t  nav_velned_gSpeed;
    int32_t   nav_velned_heading;
    uint32_t  nav_velned_sAcc;
    uint32_t  nav_velned_cAcc;

}UBX_NAV_VELNED_s;

/*
 * Ublox 8 module includes ublox 7 and more information
 * It will be used to cover both.
 */
typedef struct UBX_NAV_PVT_t{
    uint8_t     nav_pvt_cls;
    uint8_t     nav_pvt_id;
    uint16_t    nav_pvt_len;
    uint32_t    nav_pvt_iTOW;
    uint16_t    nav_pvt_year;
    uint8_t     nav_pvt_month;
    uint8_t     nav_pvt_day;
    uint8_t     nav_pvt_hour;
    uint8_t     nav_pvt_min;
    uint8_t     nav_pvt_sec;
    uint8_t     nav_pvt_valid;
    uint32_t    nav_pvt_tAcc;
    int32_t     nav_pvt_nano;
    uint8_t     nav_pvt_fixType;
    uint8_t     nav_pvt_flags;
    uint8_t     nav_pvt_flags2;
    uint8_t     nav_pvt_numSv;
    int32_t     nav_pvt_lon;
    int32_t     nav_pvt_lat;
    int32_t     nav_pvt_height;
    int32_t     nav_pvt_hMSL;
    uint32_t    nav_pvt_hAcc;
    uint32_t    nav_pvt_vAcc;
    int32_t     nav_pvt_ve1N;
    int32_t     nav_pvt_ve1E;
    int32_t     nav_pvt_ve1D;
    int32_t     nav_pvt_gSpeed;
    int32_t     nav_pvt_headMot;
    uint32_t    nav_pvt_sAcc;
    uint32_t    nav_pvt_headAcc;
    uint16_t    nav_pvt_pDOP;
    uint8_t     nav_pvt_reserved[6];
    int32_t     nav_pvt_headVeh;
    int16_t     nav_pvt_magDec;
    uint16_t    nav_pvt_magAcc;

}UBX_NAV_PVT_s;

extern circularBuffer_t gps_cb;

void CK_GPS_Init(USART_TypeDef* uart_, sensorModel_e module_type);

void CK_GPS_Update(void);

float gpsRescueGetImuYawCogGain(void);

void CK_GPS_DecodePacket(void);

void CK_GPS_WaitSatteliteFix(void);

void CK_GPS_SaveDestinationLocationAndStartHeight(void);

void CK_GPS_UBXNewByte(uint8_t data);

void CK_GPS_CalculateDistanceAndHeading(float currentLat1, float currentLon1, float destinationLat2, float destinationLon2, uint32_t* dist, int32_t* bearing);

void CK_GPS_InitConfig(void);

void CK_GPS_SendUBXPacket(uint8_t* packet, int size);

#endif /* CK_GPS_H_ */






