
#ifndef INC_COMMON_AXIS_H_
#define INC_COMMON_AXIS_H_

#include "CK_DEFINITIONS.h"

#pragma once

typedef enum {
    X = 0,
    Y,
    Z
} axis_e;

#define XYZ_AXIS_COUNT 3

// See http://en.wikipedia.org/wiki/Flight_dynamics
typedef enum {
    FD_ROLL = 0,
    FD_PITCH,
    FD_YAW
} flight_dynamics_index_t;

#define FLIGHT_DYNAMICS_INDEX_COUNT 3

typedef enum {
    AI_ROLL = 0,
    AI_PITCH
} angle_index_t;

#define RP_AXIS_COUNT 2
#define EF_AXIS_COUNT 2

#define GET_DIRECTION(isReversed) ((isReversed) ? -1 : 1)

#endif
