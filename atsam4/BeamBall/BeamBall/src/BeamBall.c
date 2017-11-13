/*
 * BeamBall.c
 *
 * Created: 09/11/2017 16:36:53
 *  Author: ilangoldman
 */ 

#include <asf.h>
#include "BeamBall.h"


void vReadSensor(void) {
	puts("Iniciando Leitura do Sensor\r\n");
	
	// clear timer
	//tc_get_status(TC_SENSOR,CHANNEL_SENSOR);
	vClearSensorCounter();
	
	gpio_set_pin_high(PIO_TRIGGER);
	delay_us(10);
	gpio_set_pin_low(PIO_TRIGGER);
}

void vMalhaControle(double distance) {
	puts("Executando Malha de Controle\r\n");
	double motorPos;

	// do some control shit!
	motorPos = distance;
	
	// Update Motor position
	vRunMotor(motorPos);
}

void vRunMotor(double pos) {
	vPWMUpdateDuty(pos);
}
