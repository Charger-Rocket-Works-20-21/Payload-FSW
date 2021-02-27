#include "system.h"

#ifndef PID_H_
#define PID_H_

void pidInit(double setkp, double setki, double setkd, double target);

void pidSetGains(double setkp, double setki, double setkd);
void pidClearIntegral(void);

void pidUpdate (double xorient, double yorient, uint32_t millis);

uint16_t pidOutput();



#endif /* PID_H_ */