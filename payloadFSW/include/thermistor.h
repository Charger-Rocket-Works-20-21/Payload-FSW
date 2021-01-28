#ifndef THERMISTOR_H_
#define THERMISTOR_H_

#include "system.h"

#define SERIESRESISTOR 10000

#define RREF 47000

#define AVAL -15.5322
#define BVAL 5229.973
#define CVAL -160451.00
#define DVAL -5.41409*pow(10,6)

#define A1VAL 3.354016*pow(10,-3)
#define B1VAL 2.519107*pow(10,-4)
#define C1VAL 3.510939*pow(10,-6)
#define D1VAL 1.105179*pow(10,-7)

double getSmoothTemp(double smoothingFactor, double smoothTemp);

#endif // THERMISTOR_H_
