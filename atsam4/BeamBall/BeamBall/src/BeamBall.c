/*
 * BeamBall.c
 *
 * Created: 09/11/2017 16:36:53
 *  Author: ilangoldman
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

//static int flag = 0;
int last_error = 0;
int integral = 0;

void vMalhaControle(double distance) {

	int motorPos = 15;
	int iDist = (int) distance;
	
	char buffer[50];
	sprintf (buffer, "%d", iDist);
	vWriteLCD(140, 100, (uint8_t*) buffer);

 	int target = 10;

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
}

void vRunMotor(int pos) {
	vPWMUpdateDuty(pos);
}
