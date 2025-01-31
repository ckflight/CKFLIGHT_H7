
#ifndef CK_IMU_H_
#define CK_IMU_H_

#include "CK_DEFINITIONS.h"
#include "common/vector.h"


typedef struct throttleCorrectionConfig_s {
    uint16_t throttle_correction_angle;     // the angle when the throttle correction is maximal. in 0.1 degres, ex 225 = 22.5 ,30.0, 450 = 45.0 deg
    uint8_t throttle_correction_value;      // the correction that will be applied at throttle_correction_angle.
} throttleCorrectionConfig_t;

// Exported symbols
extern bool canUseGPSHeading;

typedef union {
    float v[4];
    struct {
        float w, x, y, z;
    };
} quaternion_t;
#define QUATERNION_INITIALIZE  {.w=1, .x=0, .y=0,.z=0}

typedef struct {
    float ww,wx,wy,wz,xx,xy,xz,yy,yz,zz;
} quaternionProducts;
#define QUATERNION_PRODUCTS_INITIALIZE  {.ww=1, .wx=0, .wy=0, .wz=0, .xx=0, .xy=0, .xz=0, .yy=0, .yz=0, .zz=0}

typedef union {
    int16_t raw[XYZ_AXIS_COUNT];
    struct {
        // absolute angle inclination in multiple of 0.1 degree  eg attitude.values.yaw 180 deg = 1800
        int16_t roll;
        int16_t pitch;
        int16_t yaw;
    } values;
} attitudeEulerAngles_t;
#define EULER_INITIALIZE  { { 0, 0, 0 } }

extern attitudeEulerAngles_t attitude;
extern matrix33_t rMat;
extern quaternion_t imuAttitudeQuaternion; //attitude quaternion to use in blackbox

typedef struct imuConfig_s {
    uint16_t imu_dcm_kp;          // DCM filter proportional gain ( x 10000)
    uint16_t imu_dcm_ki;          // DCM filter integral gain ( x 10000)
    uint8_t small_angle;
    uint8_t imu_process_denom;
    int16_t mag_declination;      // Magnetic declination in degrees * 10
} imuConfig_t;

typedef struct imuRuntimeConfig_s {
    float imuDcmKi;
    float imuDcmKp;
} imuRuntimeConfig_t;

void imuConfigure(uint16_t throttle_correction_angle, uint8_t throttle_correction_value);

float getSinPitchAngle(void);
float getCosTiltAngle(void);
void getQuaternion(quaternion_t * q);
void imuUpdateAttitude(timeUs_t currentTimeUs);

void imuInit(uint32_t imuT, uint32_t mainT);

#ifdef SIMULATOR_BUILD
void imuSetAttitudeRPY(float roll, float pitch, float yaw);  // in deg
void imuSetAttitudeQuat(float w, float x, float y, float z);
#if defined(SIMULATOR_IMU_SYNC)
void imuSetHasNewData(uint32_t dt);
#endif
#endif

bool imuQuaternionHeadfreeOffsetSet(void);
void imuQuaternionHeadfreeTransformVectorEarthToBody(vector3_t *v);
bool isUpright(void);

#endif /* CK_IMU_H_ */
