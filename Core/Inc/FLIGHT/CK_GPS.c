
#include <COMMON/maths.h>
#include "DRIVERS/CK_UART.h"
#include "DRIVERS/CK_TIME_HAL.h"
#include "DRIVERS/CK_CIRCULARBUFFER.h"
#include "DRIVERS/CK_BUZZER.h"

#include "FLIGHT/CK_RECEIVER.h"
#include "FLIGHT/CK_GPS.h"

#include "COMMUNICATION/CK_PRINTER.h"

#define GPS_DEFAULT_BAUDRATE        9600
#define GPS_CONFIGURED_BAUDRATE     115200

#define SATTELITE_NUM_THRESHOLD     3
#define SATTELITE_FIX_THRESHOLD     2

typedef struct {
    float distanceToHomeCm;
    float distanceToHomeM;
    uint16_t groundSpeedCmS;
    int16_t directionToHome;
    bool healthy;
    float errorAngle;
    float gpsDataIntervalSeconds;
    float velocityToHomeCmS;
    float alitutudeStepCm;
    float maxPitchStep;
    float absErrorAngle;
    float imuYawCogGain;
} rescueSensorData_s;

typedef struct {
    //rescuePhase_e phase;
    //rescueFailureState_e failure;
    rescueSensorData_s sensor;
    //rescueIntent_s intent;
    bool isAvailable;
} rescueState_s;

rescueState_s rescueState;

UBX_NAV_PVT_s pvt;

gpsSensor_t gps;

sensorModel_e module;

//#define COMPUTE_DMS       // Degree minute sec

uint8_t UBX_PAYLOAD_SIZE;

uint8_t _step;
uint8_t _class;
uint8_t _ck_a;
uint8_t _ck_b;
uint8_t _msg_id;
uint16_t _payload_size;
uint16_t _payload_counter;
uint8_t _buffer[92];
uint16_t _receive_error;
bool gps_dataReceived;

DEBUG_TIME_t gps_debug;

USART_TypeDef* FLIGHT_GPS_UART;

circularBuffer_t gps_cb;

void CK_GPS_Init(USART_TypeDef* uart_, sensorModel_e module_type){

	FLIGHT_GPS_UART = uart_;

	gps.isGpsInit = false;

	gps.isSatFixed = false;

    module = module_type;

    if(module == GPS_UBLOX7){
        UBX_PAYLOAD_SIZE = 84;
    }
    else if(module == GPS_UBLOX8){
        UBX_PAYLOAD_SIZE = 92;
    }

    USART_CONFIGURATION_ config;
#if GPS_
    config.tx_gpio_type			= GPS_UART_TX_GPIO;
	config.tx_gpio_pin			= GPS_UART_TX_PIN;
	config.tx_af				= GPS_UART_TX_AF;

	config.rx_gpio_type			= GPS_UART_RX_GPIO;
	config.rx_gpio_pin			= GPS_UART_RX_PIN;
	config.rx_af				= GPS_UART_RX_AF;
#endif
	config.interrupt 			= RX_INTERRUPT;
    config.mode 				= RX_TX;
    config.parity 				= NO_PARITY;
    config.stop_bit 			= STOP_BIT1;
    config.baudrate 			= GPS_DEFAULT_BAUDRATE;
    config.usart 				= FLIGHT_GPS_UART;
    config.use_circular_buffer 	= true;


    CK_UART_Init(&config, &gps_cb);

    CK_GPS_InitConfig();

	CK_UART_Reset(FLIGHT_GPS_UART);

    config.interrupt 			= RX_INTERRUPT;
    config.mode 				= RX_TX;
    config.parity 				= NO_PARITY;
    config.stop_bit 			= STOP_BIT1;
    config.baudrate 			= GPS_CONFIGURED_BAUDRATE;
    config.usart				= FLIGHT_GPS_UART;
    config.use_circular_buffer 	= true;
    CK_UART_Init(&config, &gps_cb);

	#if USE_INTERRUPT_GPS
    CK_UART_RXInterruptEnable(FLIGHT_GPS_UART);
    CK_UART_RXEnable(FLIGHT_GPS_UART);
	#endif

	gps.isGpsInit = true;

}

void CK_GPS_Update(void){

	if(gps.isGpsInit){

	    if(gps_dataReceived){

	        #if defined(DEBUG_TIMING)
	        gps_debug.start_time = CK_TIME_GetMicroSec();
	        #endif

	        CK_GPS_DecodePacket();

	        gps.current_lat             = pvt.nav_pvt_lat;

	        gps.current_lon             = pvt.nav_pvt_lon;

	        gps.numOfSattelite          = pvt.nav_pvt_numSv;

	        gps.satteliteFix            = pvt.nav_pvt_fixType;

	        gps.current_heightEllipsoid = pvt.nav_pvt_height / 10;      // Convert mm to cm

	        gps.current_heightSeaLevel  = pvt.nav_pvt_hMSL / 10;        // Convert mm to cm

	        gps.groundCourse         = pvt.nav_pvt_headMot / 10000; // Heading 2D deg * 100000 rescaled to deg * 10 (because i multiply with 10.000 rather than 100.000)

	        gps.groundSpeed             = pvt.nav_pvt_gSpeed / 10;      // mm/sec to cm/sec

	        gps.heightDifference        = gps.current_heightSeaLevel - gps.onGround_heightSeaLevel; // in cm

	        if(gps.heightDifference <= 0){
	            gps.heightDifference = 0;
	        }

	        #if defined(COMPUTE_DMS)
	        /*
	         * Convert current latitude and longitude to degree min sec.
	         * Degree min sec is not used but converted anyway.
	         */

	        // Latitude part
	        gps.lat_deg = gps.current_lat / 10000000; // in decimal degrees

	        float lat_f = (gps.current_lat - (gps.lat_deg * 10000000)) / 10000000.0f;

	        gps.lat_min = lat_f * 60;

	        float lat_min_dec = (lat_f * 60) - (float)gps.lat_min;

	        gps.lat_sec = lat_min_dec * 60;

	        gps.lat_sec = roundf(gps.lat_sec * 1000) / 1000;

	        // Longitude part
	        gps.lon_deg = gps.current_lon / 10000000;

	        float lon_f = (gps.current_lon - (gps.lon_deg * 10000000)) / 10000000.0f;

	        gps.lon_min = lon_f * 60;

	        float lon_min_dec = (lon_f * 60) - (float)gps.lon_min;

	        gps.lon_sec = lon_min_dec * 60;

	        gps.lon_sec = roundf(gps.lon_sec * 1000) / 1000;
	        #endif

	        CK_GPS_CalculateDistanceAndHeading(gps.current_lat, gps.current_lon, gps.destination_lat, gps.destination_lon, &gps.distanceToDestination, &gps.headingToDestination);

	        gps_dataReceived = false;

	        #if defined(DEBUG_TIMING)
	        gps_debug.update_time = CK_TIME_GetMicroSec() - gps_debug.start_time;
	        #endif

	    }

	}

}

void CK_GPS_WaitSatteliteFix(void){

	if(gps.isGpsInit){

	    // Not enough sattelite and not fixed yet.
	    while(gps.numOfSattelite < SATTELITE_NUM_THRESHOLD || gps.satteliteFix < SATTELITE_FIX_THRESHOLD){

	    	CK_GPS_Update();

	    }
	    CK_GPS_SaveDestinationLocationAndStartHeight();

	    gps.isSatFixed = true;

	    CK_BUZZER_Tone1();
	}

}

/*
 * Save current location as destination coordinates.
 * Once the gps is fixed the start postion should be saved
 * by calling this method.
 */
void CK_GPS_SaveDestinationLocationAndStartHeight(void){

    // Read some number of data before saving.
    int counter = 10;
    while(counter){
        if(gps_dataReceived){
            CK_GPS_Update();
            counter--;
        }
    }

    // Save start location before take off to return when in gps rescue mode.
    gps.destination_lat = gps.current_lat;
    gps.destination_lon = gps.current_lon;

    // Measure sea level height before drone take off so later current height
    // relative to start height will provide altitude information.
    gps.onGround_heightSeaLevel = gps.current_heightSeaLevel;

}

float gpsRescueGetImuYawCogGain(void)
{
    return rescueState.sensor.imuYawCogGain; // to speed up the IMU orientation to COG when needed
}

void CK_GPS_DecodePacket(void){

    /*
     * Check ublox 8 and 7 datasheet to see payload byte index.
     * Ublox 8 has 92 bytes and Ublox 7 has 84.
     * Mostly similar but ublox8 has a few more information.
     * The data that i used are in the same location
     *
     */

    if(module == GPS_UBLOX8){
        // Fix type 2 is 2D fix, 3 is 3D fix
        pvt.nav_pvt_fixType = _buffer[20];

        // Num of sattelites connected
        pvt.nav_pvt_numSv   = _buffer[23];

        // Ground Speed 2D in mm/s
        pvt.nav_pvt_gSpeed  = (int32_t)((_buffer[63]<<24)+(_buffer[62]<<16)+(_buffer[61]<<8)+_buffer[60]);

        // Heading of Motion 2D in degrees * 100000
        pvt.nav_pvt_headMot  = (int32_t)((_buffer[67]<<24)+(_buffer[66]<<16)+(_buffer[65]<<8)+_buffer[64]);

        // Height ellipsoid in mm
        pvt.nav_pvt_height  = (int32_t)((_buffer[35]<<24)+(_buffer[34]<<16)+(_buffer[33]<<8)+_buffer[32]);

        // Height sea level in mm
        pvt.nav_pvt_hMSL    = (int32_t)((_buffer[39]<<24)+(_buffer[38]<<16)+(_buffer[37]<<8)+_buffer[36]);

        // Current longitude in decimal degrees
        pvt.nav_pvt_lon     = (int32_t)((_buffer[27]<<24)+(_buffer[26]<<16)+(_buffer[25]<<8)+_buffer[24]);

        // Current latitude in decimal degrees
        pvt.nav_pvt_lat     = (int32_t)((_buffer[31]<<24)+(_buffer[30]<<16)+(_buffer[29]<<8)+_buffer[28]);
    }
    else if(module == GPS_UBLOX7){

        // Fix type 2 is 2D fix, 3 is 3D fix
        pvt.nav_pvt_fixType = _buffer[20];

        // Num of sattelites connected
        pvt.nav_pvt_numSv   = _buffer[23];

        // Ground Speed 2D in mm/s
        pvt.nav_pvt_gSpeed  = (int32_t)((_buffer[63]<<24)+(_buffer[62]<<16)+(_buffer[61]<<8)+_buffer[60]);

        // Heading of Motion 2D in degrees * 100000
        pvt.nav_pvt_headMot  = (int32_t)((_buffer[67]<<24)+(_buffer[66]<<16)+(_buffer[65]<<8)+_buffer[64]);

        // Height ellipsoid in mm
        pvt.nav_pvt_height  = (int32_t)((_buffer[35]<<24)+(_buffer[34]<<16)+(_buffer[33]<<8)+_buffer[32]);

        // Height sea level in mm
        pvt.nav_pvt_hMSL    = (int32_t)((_buffer[39]<<24)+(_buffer[38]<<16)+(_buffer[37]<<8)+_buffer[36]);

        // Current longitude in decimal degrees
        pvt.nav_pvt_lon     = (int32_t)((_buffer[27]<<24)+(_buffer[26]<<16)+(_buffer[25]<<8)+_buffer[24]);

        // Current latitude in decimal degrees
        pvt.nav_pvt_lat     = (int32_t)((_buffer[31]<<24)+(_buffer[30]<<16)+(_buffer[29]<<8)+_buffer[28]);
    }


}

/*
 * In this way gps input will be decoded byte by byte not as a whole packet since it takes long.
 * Each received byte will be placed if it fits to ubx packet. And states will be updated.
 *
 * Genereal structure of ubx packet:
 *
 * Header(0xB5, 0x62) + Class(0x01) + ID(0x12) + Length(2 bytes (little endian) showing payload size) + Payload + Checksum(CKA + CKB)
 *
 */
void CK_GPS_UBXNewByte(uint8_t data){

    switch(_step){
    case 0: // Sync character 1 0xB5
        _receive_error = 0;
        if(data == 0xB5){
            _step++;
        }
    break;
    case 1: // Sync character 2 0x62
        if(data != 0x62){
            _step = 0; // Not a packet return to first state.
            break;
        }
        _step++;
    break;

    case 2: // Class
        _step++;
        _class = data;
        _ck_b = _ck_a = data; // reset checksum accumulator.
        break;
    case 3: // Id
        _step++;
        _ck_b += (_ck_a += data); // calculate checksums
        _msg_id = data;
        break;
    case 4: // Payload size low byte
        _step++;
        _ck_b += (_ck_a += data); // calculate checksum
        _payload_size = data;
        break;
    case 5: // Payload size high byte
        _ck_b += (_ck_a += data); // calculate checksum
        _payload_size += (uint16_t)(data << 8);
        if(_payload_size == UBX_PAYLOAD_SIZE){ // I write this to work with pvt for now since it has every information.
            _step++;
            _payload_counter = 0;
            break;
        }
        _step = 0;
        break;
    case 6:
        _ck_b += (_ck_a += data); // calculate checksum
        if(_payload_counter < UBX_PAYLOAD_SIZE){
            _buffer[_payload_counter++] = data;
        }
        if(_payload_counter >= UBX_PAYLOAD_SIZE){
            _step++;
        }
        break;
    case 7:
        _step++;
        if(_ck_a != data){
            _receive_error++; // did not receive correctly
        }
        break;
    case 8:
        _step = 0;
        if(_ck_b != data){
            _receive_error++;
            break;
        }
        if(_receive_error == 0){ // received checksum is true data correctly received.
            gps_dataReceived = true;
        }
        break;
    }

}

// Calculates distance between two points in cm.
// Calculates bearing from pos1 to pos2 return 1 deg = 100 precision. Ex 9300 = 93 degrees
#define TAN_89_99_DEGREES   5729.57795f
void CK_GPS_CalculateDistanceAndHeading(float currentLat1, float currentLon1, float destinationLat2, float destinationLon2, uint32_t* dist, int32_t* bearing){

    float dLat = destinationLat2 - currentLat1;
    float dLon = destinationLon2 - currentLon1;

    *dist = sqrtf(sq(dLat) + sq(dLon)) * EARTH_ANGLE_TO_CM;

    //atan2 has appr version on betaflight.
    *bearing = 9000.0f + atan2_approx(-dLat, dLon) * TAN_89_99_DEGREES; // convert the output radians to 100xdeg.
    if(*bearing < 0){
        *bearing += 36000;
    }

}

void CK_GPS_InitConfig(){

    // Commands for configuring gps.

    // Same for U-blox 8 and 7 series.
    uint8_t NMEA_GVTG_DISABLE[]        = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x05, 0x00, 0xFF, 0x19};
    CK_GPS_SendUBXPacket(NMEA_GVTG_DISABLE, 11);
    CK_TIME_DelayMilliSec(10);

    // Same for U-blox 8 and 7 series.
    uint8_t NMEA_GRMC_DISABLE[]        = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x04, 0x00, 0xFE, 0x17};
    CK_GPS_SendUBXPacket(NMEA_GRMC_DISABLE, 11);
    CK_TIME_DelayMilliSec(10);

    // Same for U-blox 8 and 7 series.
    uint8_t NMEA_GGSV_DISABLE[]        = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15};
    CK_GPS_SendUBXPacket(NMEA_GGSV_DISABLE, 11);
    CK_TIME_DelayMilliSec(10);

    // Same for U-blox 8 and 7 series.
    uint8_t NMEA_GGSA_DISABLE[]        = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x02, 0x00, 0xFC, 0x13};
    CK_GPS_SendUBXPacket(NMEA_GGSA_DISABLE, 11);
    CK_TIME_DelayMilliSec(10);

    // Same for U-blox 8 and 7 series.
    uint8_t NMEA_GGLL_DISABLE[]        = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x01, 0x00, 0xFB, 0x11};
    CK_GPS_SendUBXPacket(NMEA_GGLL_DISABLE, 11);
    CK_TIME_DelayMilliSec(10);

    // Same for U-blox 8 and 7 series.
    uint8_t NMEA_GGGA_DISABLE[]        = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x00, 0x00, 0xFA, 0x0F};
    CK_GPS_SendUBXPacket(NMEA_GGGA_DISABLE, 11);
    CK_TIME_DelayMilliSec(10);

    // Same for U-blox 8 and 7 series.
    uint8_t UBX_NAV_PVT_ENABLE[]     = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x07, 0x01, 0x13, 0x51};
    CK_GPS_SendUBXPacket(UBX_NAV_PVT_ENABLE, 11);
    CK_TIME_DelayMilliSec(10);

    //uint8_t UBX_NAV_POSLLH_ENABLE[]    = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x02, 0x01, 0x0E, 0x47};
    //CK_GPS_SendUBXPacket(UBX_NAV_POSLLH_ENABLE, 11);
    //CK_TIME_DelayMilliSec(10);

    //uint8_t UBX_NAV_VELNED_ENABLE[]  = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x12, 0x01, 0x1E, 0x67};
    //CK_GPS_SendUBXPacket(UBX_NAV_VELNED_ENABLE, 11);
    //CK_TIME_DelayMilliSec(10);

    //uint8_t UBX_NAV_STATUS_ENABLE[]  = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x03, 0x01, 0x0F, 0x49};
    //CK_GPS_SendUBXPacket(UBX_NAV_PVT_ENABLE, 11);
    //CK_TIME_DelayMilliSec(10);

    //uint8_t UBX_NAV_SOLUTION_ENABLE[] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x06, 0x01, 0x12, 0x4F};
    //CK_GPS_SendUBXPacket(UBX_NAV_SOLUTION_ENABLE, 11);
    //CK_TIME_DelayMilliSec(10);

    // Same for U-blox 8 and 7 series.
    uint8_t UBX_CFG_UPDATE_10HZ[]     = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01,
                                         0x00, 0x7A, 0x12};
    CK_GPS_SendUBXPacket(UBX_CFG_UPDATE_10HZ, 14);
    CK_TIME_DelayMilliSec(10);

    if(module == GPS_UBLOX8){
        uint8_t UBX_CFG_LED_10HZ[]        = {0xB5, 0x62, 0x06, 0x31, 0x20, 0x00, 0x00, 0x01, 0x00, 0x00, 0x32,
                                             0x00, 0x00, 0x00, 0x40, 0x42, 0x0F, 0x00, 0xA0, 0x86, 0x01, 0x00,
                                             0x00, 0x00, 0x00, 0x00, 0x50, 0xC3, 0x00, 0x00, 0x00, 0x00, 0x00,
                                             0x00, 0x77, 0x00, 0x00, 0x00, 0xCC, 0x1B};
        CK_GPS_SendUBXPacket(UBX_CFG_LED_10HZ, 40);
        CK_TIME_DelayMilliSec(10);
    }
    else if(module == GPS_UBLOX7){
        uint8_t UBX_CFG_LED_10HZ[]        = {0xB5, 0x62, 0x06, 0x31, 0x20, 0x00, 0x00, 0x01, 0x00, 0x00, 0x32,
                                             0x00, 0x00, 0x00, 0x40, 0x42, 0x0F, 0x00, 0xA0, 0x86, 0x01, 0x00,
                                             0x00, 0x00, 0x00, 0x00, 0x50, 0xC3, 0x00, 0x00, 0x00, 0x00, 0x00,
                                             0x00, 0xF7, 0x00, 0x00, 0x00, 0x4C, 0x1B};
        CK_GPS_SendUBXPacket(UBX_CFG_LED_10HZ, 40);
        CK_TIME_DelayMilliSec(10);
    }


    // Same for U-blox 8 and 7 series.
    uint8_t UBX_CFG_UART115200[]      = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0,
                                         0x08, 0x00, 0x00, 0x00, 0xC2, 0x01, 0x00, 0x07, 0x00, 0x03, 0x00,
                                         0x00, 0x00, 0x00, 0x00, 0xC0, 0x7E};
    CK_GPS_SendUBXPacket(UBX_CFG_UART115200, 28);
    CK_TIME_DelayMilliSec(10);


}

void CK_GPS_SendUBXPacket(uint8_t* packet, int size){
	for(int i = 0; i < size ;i++){
		CK_UART_SendPolling(FLIGHT_GPS_UART, packet[i]);
	}

}

#if GPS_

#if GPS_INTERRUPT_ == 1
void USART1_IRQHandler(void){
#endif

#if GPS_INTERRUPT_ == 2
void USART2_IRQHandler(void){
#endif

#if GPS_INTERRUPT_ == 3
void USART3_IRQHandler(void){
#endif

#if GPS_INTERRUPT_ == 4
void UART4_IRQHandler(void){
#endif

#if GPS_INTERRUPT_ == 5
void UART5_IRQHandler(void){
#endif

#if GPS_INTERRUPT_ == 6
void USART6_IRQHandler(void){
#endif

	#if USE_F4 == 1
	if(FLIGHT_GPS_UART->SR & CK_USART_SR_RXNE){
		uint8_t rxData = FLIGHT_GPS_UART->DR;
	#endif

	#if USE_H7 == 1
    if(FLIGHT_GPS_UART->ISR & CK_USART_SR_RXNE){
    	FLIGHT_GPS_UART->ICR = 0xFFFFFFFF;

    	uint8_t rxData = FLIGHT_GPS_UART->RDR;
    #endif

        CK_GPS_UBXNewByte(rxData);

        if(!CK_CIRCULARBUFFER_IsBufferFull(&gps_cb)){
            CK_CIRCULARBUFFER_BufferWrite(&gps_cb, rxData);
        }
    }
}

#endif











