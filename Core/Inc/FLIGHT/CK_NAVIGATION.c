
#include <COMMON/maths.h>
#include "DRIVERS/CK_TIME_HAL.h"

#include "MOTION/CK_BNO055.h"
#include "MOTION/CK_IMU.h"

#include "FLIGHT/CK_RECEIVER.h"
#include "FLIGHT/CK_RC.h"
#include "FLIGHT/CK_PID.h"
#include "FLIGHT/CK_GPS.h"
#include "FLIGHT/CK_NAVIGATION.h"
#include "FLIGHT/CK_LAND.h"

#include "COMMUNICATION/CK_PRINTER.h"

#define GPS_RESCUE_LANDING_DISTANCE                 25

#define GPS_POS_HOLD_MAX_MIN_CORRECTION_ANGLE       10.0f

float gps_heading_setpoint;
float imu_heading;
float gps_heading_error;
float gps_heading_correction;

float gps_distance_error_cm;
float gps_distance_correction;

bool gps_rescue_arrivedHome     = false;
bool gps_rescue_reset_once      = false;
bool ifRescueUsed               = false;

int32_t gps_hold_lat;
int32_t gps_hold_lon;
bool gps_pos_hold_isHoldPositionSaved   = false;
bool gps_pos_hold_reset_once            = false;
bool ifPosHoldUsed                      = false;

float mag_heading_setpoint = 20.0f; // this value is for testing if it turns to that angle
float mag_heading_error;
float mag_heading_correction;

DEBUG_TIME_t navigation_gps_rescue_debug;

DEBUG_TIME_t navigation_gps_poshold_debug;

void CK_NAVIGATION_GPSRescue(void){

    #if defined(DEBUG_TIMING)
    navigation_gps_rescue_debug.start_time = CK_TIME_GetMicroSec();
    #endif

    /*
     * When gps rescue is on, to do:
     *
     * 1. Take drone to level mode and altitude hold mode
     * 2. Turn drones head to destination by using CK_NAVIGATION_GPSHeadingHold
     * 3. Increase drone to a safe altitude to travel. (Later add it)
     * 4. Set drones pitch angle based on the distance to destination by using CK_NAVIGATION_GPSDistancePID
     * 5. Lower the drone's altitude based on the distace to destination (Later add it)
     * 6. Land the drone if the distance is smaller than 25 meters.
     *
     */

    // Later when failsafe is done the code will get here.
    // Now rescue only triggered with switch and failsafe lands drone.
    if(flags.GPS_RESCUE){

        gps_rescue_reset_once   = false;

        ifRescueUsed = true;

        if(!gps_rescue_arrivedHome){

            gps_rescue_arrivedHome  = false;

            if(!overwrite_flags.FORCE_TO_ANGLE_MODE || !overwrite_flags.FORCE_TO_ALTITUDE_HOLD){
                // Forcing will be active as of next loop cycle until cancelation.
                overwrite_flags.FORCE_TO_ANGLE_MODE     = true;
                overwrite_flags.FORCE_TO_ALTITUDE_HOLD  = true;
            }

            CK_NAVIGATION_GPSHeadingHold();

            CK_NAVIGATION_GPSDistancePID();
        }
        else{
            // Arrived home land the drone.
            if(!overwrite_flags.FORCE_TO_LANDING){
                // From now on Landing will set necessary flags for itself.
                // Altitude hold is closed by force landing.
                overwrite_flags.FORCE_TO_LANDING        = true;
                overwrite_flags.FORCE_TO_ANGLE_MODE     = false;
                overwrite_flags.FORCE_TO_ALTITUDE_HOLD  = false;
            }
        }
    }
    else{
        if(ifRescueUsed){
            if(!gps_rescue_reset_once){
                // Reset everything for once when rescue not used.
                gps_rescue_reset_once                   = true;
                ifRescueUsed                            = false;
                gps_rescue_arrivedHome                  = false;

                overwrite_flags.FORCE_TO_ANGLE_MODE     = false;
                overwrite_flags.FORCE_TO_ALTITUDE_HOLD  = false;
                overwrite_flags.FORCE_TO_LANDING        = false;

            }
        }
    }

    #if defined(DEBUG_TIMING)
    navigation_gps_rescue_debug.update_time = CK_TIME_GetMicroSec() - navigation_gps_rescue_debug.start_time;
    #endif
}

void CK_NAVIGATION_GPSDistancePID(void){

    // Distance in cm will be the error itself since as it gets closer it will decrease.
    gps_distance_error_cm = gps.distanceToDestination;

    // If the distance is close enough land drone, finish gps rescue and altitude hold.
    if(gps_distance_error_cm <= GPS_RESCUE_LANDING_DISTANCE){

        gps_rescue_arrivedHome = true;
    }

    // navigation will have its own parameters which will include pid related parameters as well
    //gps_distance_correction = gps_distance_error_cm * (float)PID_SELECTED_PROFILE[AXIS_NAVIGATION][PID_P] / 500.0f;

    // Level flag is raised in this mode so it cannot pass 50 degrees since pid level mode limits.
    // Therefore CK_RCCommand is at -500,500 range can be set to even maximum 500.
    // distance > 300m(30000cm) gives 30 degree which is good to test now.
    // 5 deg < Angle < 30 deg
    setRCCommand(PITCH, constrain((int)gps_distance_correction, 50, 300));

}

void CK_NAVIGATION_GPSHeadingHold(void){

    // error = imu heading - heading to destination(start point) which is our setpoint
    // the point where the drone needs to turn

    gps_heading_setpoint = gps.headingToDestination / 100.0f; // get setpoint in degrees. (it is 100xdeg)

    imu_heading = attitude.values.yaw;

    gps_heading_error = imu_heading - gps_heading_setpoint;

    if(gps_heading_error >= +180.0f){
        gps_heading_error -= 360.0f;
    }
    if(gps_heading_error <= -180.0f){
        gps_heading_error += 360.0f;
    }

    // navigation will have its own parameters which will include pid related parameters as well
    //gps_heading_correction = gps_heading_error * (float)PID_SELECTED_PROFILE[AXIS_NAVIGATION][PID_P] / 10.0f;

    int rc_temp = getRCCommand(YAW) - (int)gps_heading_correction;

    setRCCommand(YAW, rc_temp);

}

void CK_NAVIGATION_GPSPositionHold(void){

    #if defined(DEBUG_TIMING)
    navigation_gps_poshold_debug.start_time = CK_TIME_GetMicroSec();
    #endif

    // save gps hold position when pos. hold is activated
    // calculate deltaLat and deltaLon and convert them to distance in cm.
    // raise level flag and use error as trim value so level method will make it stay at pos.
    if(flags.GPS_POS_HOLD){

        gps_pos_hold_reset_once = false;

        ifPosHoldUsed = true;

        if(!gps_pos_hold_isHoldPositionSaved){
            // Save the gps position to hold.
            gps_hold_lat = gps.current_lat;
            gps_hold_lon = gps.current_lon;
            gps_pos_hold_isHoldPositionSaved = true;
        }

        if(!overwrite_flags.FORCE_TO_ANGLE_MODE){
            // Forcing will be active as of next loop cycle until cancelation.
            overwrite_flags.FORCE_TO_ANGLE_MODE     = true;
        }

        // If landing mode want to use gps pos hold then do not force to altitude hold
        if(!flags.LANDING){
            if(!overwrite_flags.FORCE_TO_ALTITUDE_HOLD){
                overwrite_flags.FORCE_TO_ALTITUDE_HOLD  = true;
            }
        }


        // Latitude is horizontal(parallel) cuts
        // If error positive moving forward, error negative moving bacward.
        float lat_position_error = gps.current_lat - gps_hold_lat;
        // Convert lat error to distance in cm
        float lat_error_cm = lat_position_error * EARTH_ANGLE_TO_CM;
        UNUSED(lat_error_cm);

        // PID at level mode uses trim to correct small drifts.
        // Imu nose down + pitch
        // Makes upto 10 degree correction
        //CK_RC_SetRCTrim(1, constrainf(lat_error_cm, -GPS_POS_HOLD_MAX_MIN_CORRECTION_ANGLE, GPS_POS_HOLD_MAX_MIN_CORRECTION_ANGLE));

        // Longitude is vertical cuts
        // If error positive moving right, error negative moving left.
        float lon_position_error = gps.current_lon - gps_hold_lon;
        // Convert lat error to distance in cm
        float lon_error_cm = lon_position_error * EARTH_ANGLE_TO_CM;
        UNUSED(lon_error_cm);

        // PID at level mode uses trim to correct small drifts.
        // Imu right down + roll
        // Makes upto 10 degree correction
        //CK_RC_SetRCTrim(0, constrainf(lon_error_cm, -GPS_POS_HOLD_MAX_MIN_CORRECTION_ANGLE, GPS_POS_HOLD_MAX_MIN_CORRECTION_ANGLE));

    }
    else{
        if(ifPosHoldUsed){
            if(!gps_pos_hold_reset_once){
                gps_pos_hold_reset_once                 = true;
                ifPosHoldUsed                           = false;
                gps_pos_hold_isHoldPositionSaved        = false;
                overwrite_flags.FORCE_TO_ANGLE_MODE     = false;
                overwrite_flags.FORCE_TO_ALTITUDE_HOLD  = false;
            }
        }
    }

    #if defined(DEBUG_TIMING)
    navigation_gps_poshold_debug.update_time = CK_TIME_GetMicroSec() - navigation_gps_poshold_debug.start_time;
    #endif
}

void CK_NAVIGATION_MAGHeadingHold(void){

    if(flags.MAG_HOLD){

        #if BNO055_
            mag_heading_error = bno055.eulerAngles[YAW] - mag_heading_setpoint;

            if(mag_heading_error >= +180.0f){
                mag_heading_error -= 360.0f;
            }
            if(mag_heading_error <= -180.0f){
                mag_heading_error += 360.0f;
            }

            //mag_heading_correction = mag_heading_error * (float)PID_SELECTED_PROFILE[AXIS_NAVIGATION][PID_P] / 10.0f;
            //rcCommand[YAW] -= (int)mag_heading_correction;

        #endif

        #if MAG_I2C_ || MAG_SPI_

            mag_heading_error = attitude.values.yaw - mag_heading_setpoint;

            if(mag_heading_error >= +180.0f){
                mag_heading_error -= 360.0f;
            }
            if(mag_heading_error <= -180.0f){
                mag_heading_error += 360.0f;
            }

            // navigation will have its own parameters which will include pid related parameters as well
            //mag_heading_correction = mag_heading_error * (float)PID_SELECTED_PROFILE[AXIS_NAVIGATION][PID_P] / 10.0f;

            int rc_temp = getRCCommand(YAW);
            rc_temp -= (int)mag_heading_correction;
            setRCCommand(YAW, rc_temp);
        #endif

    }

}

float CK_NAVIGATION_GetGPSHeadingSetpoint(void){
    return gps_heading_setpoint;
}

float CK_NAVIGATION_GetGPSHeadingError(void){
    return gps_heading_error;
}

float CK_NAVIGATION_GetGPSHeadingCorrection(void){
    return gps_heading_correction;
}

float CK_NAVIGATION_GetMAGHeadingSetpoint(void){
    return mag_heading_setpoint;
}

float CK_NAVIGATION_GetMAGHeadingError(void){
    return mag_heading_error;
}

float CK_NAVIGATION_GetMAGHeadingCorrection(void){
    return mag_heading_correction;
}
