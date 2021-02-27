#include "pid.h"

double kp, ki, kd;
double output, targetPoint;
double errSum, lastErr;

uint32_t last_millis = 0;

void pidInit(double setkp, double setki, double setkd, double target) {
	pidSetGains(setkp, setki, setkd);
	targetPoint = target;
}

void pidSetGains(double setkp, double setki, double setkd) {
	kp = setkp;
	ki = setki;
	kd = setkd;
}

void pidClearIntegral() {
	errSum = 0;
}

void pidUpdate (double xorient, double zorient, uint32_t millis) {
	double rorient = sqrt(pow((xorient+90), 2) + pow(zorient, 2)); // Resultant vector

	double dt = ((double)(millis - last_millis))/1000.0;
	
	double error = targetPoint - rorient;
	
	errSum += (error * dt);
	double dErr = (error - lastErr) / dt;
	
	//printf("P: = %f I: %f D: %f\n",kp*error, ki*errSum, kd*dErr);
	output = kp*error + ki*errSum + kd*dErr;
	
	last_millis = millis;
	lastErr = error;
}

uint16_t pidOutput() {
	return (uint16_t)(max(min(-output+500.0,1000.0),0.0)); // add 500 so it is centered

}