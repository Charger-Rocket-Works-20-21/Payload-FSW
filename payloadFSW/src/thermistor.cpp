#include "thermistor.h"

double getSmoothTemp(double smoothingFactor, double smoothTemp) {
    double reading = analogRead(THERMISTORPIN);
    double resistance = SERIESRESISTOR / ((1023 / reading) - 1);
	double newTemp = 1 / (A1VAL + B1VAL*log(resistance/RREF) + C1VAL*pow(log(resistance/RREF),2) + D1VAL*pow(log(resistance/RREF),3));
	return smoothingFactor * newTemp + (1 - smoothingFactor) * smoothTemp;
}