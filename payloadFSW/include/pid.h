#include "system.h"

#ifndef PID_H_
#define PID_H_

void pid_init(double setkp, double setki, double setkd, double setsetpoint);

void pid_set_gains(double setkp, double setki, double setkd);
void pid_set_setpoint(double setsetpoint);
void pid_clear_integral(void);

void pid_update (std::vector<double> heading, uint32_t millis);

uint16_t pid_output();



#endif /* PID_H_ */