/*
 *
 * Main Project Functions
 * Implementation of the Control 
 *
 */ 

#include <asf.h>
#include "BeamBall.h"

void vReadSensor(void) {
	pio_clear(PIOA,PIO_TRIGGER);
	delay_us(2);
	
	pio_set(PIOA,PIO_TRIGGER);
	delay_us(10);
	pio_clear(PIOA,PIO_TRIGGER);
}

int last_error = 0;
int integral = 0;

void vMalhaControle(double distance) {
	int iDist = (int) distance;
	int target = 10;
	int motorPos;
	
	double error = target - iDist;
	integral = integral + (error*1);
	double derivative = (error - last_error)/1;
	int pid = getKP()*error + getKI()*integral + getKD()*derivative;// + bias
	last_error = error;
	motorPos = - pid + INIT_DUTY_VALUE;
	 
	if (motorPos < MIN_DUTY_VALUE)
		 motorPos = MIN_DUTY_VALUE;
	else if (motorPos > MAX_DUTY_VALUE)
		motorPos = MAX_DUTY_VALUE;

	// Update Motor position
	vRunMotor(motorPos);
	
	char buffer[50];
	sprintf (buffer, "%d", iDist);
	vWriteLCD(140, 100, (uint8_t*) buffer);
}

void vRunMotor(int pos) {
	vPWMUpdateDuty(pos);
}
