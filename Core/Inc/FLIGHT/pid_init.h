
#ifndef INC_FLIGHT_PID_INIT_H_
#define INC_FLIGHT_PID_INIT_H_

#include "CK_DEFINITIONS.h"

#include "FLIGHT/CK_PID.h"

void pidInit(uint32_t mainT);
void pidInitFilters(const pidProfile_t *pidProfile);
void pidInitConfig(const pidProfile_t *pidProfile);

void pidSetItermAccelerator(float newItermAccelerator);
uint16_t pidGetDefaultProfile(uint8_t* copy_buffer);
uint16_t pidGetCurrentProfile(uint8_t* copy_buffer);
void pidLoadParameter(void);
void pidUpdateParameter(int pid_parameter, uint8_t n1, uint8_t n2);
void pidUpdateTPAParameters(int num, int parameter);
void pidInitializeParameters(void);

#endif
