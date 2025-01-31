
#ifndef CK_NAVIGATION_H_
#define CK_NAVIGATION_H_

#include "CK_DEFINITIONS.h"

void CK_NAVIGATION_GPSRescue(void);

void CK_NAVIGATION_GPSDistancePID(void);

void CK_NAVIGATION_GPSHeadingHold(void);

void CK_NAVIGATION_CheckLanding(void);

void CK_NAVIGATION_GPSPositionHold(void);

void CK_NAVIGATION_MAGHeadingHold(void);

float CK_NAVIGATION_GetGPSHeadingSetpoint(void);

float CK_NAVIGATION_GetGPSHeadingError(void);

float CK_NAVIGATION_GetGPSHeadingCorrection(void);

float CK_NAVIGATION_GetMAGHeadingSetpoint(void);

float CK_NAVIGATION_GetMAGHeadingError(void);

float CK_NAVIGATION_GetMAGHeadingCorrection(void);

#endif /* CK_NAVIGATION_H_ */
