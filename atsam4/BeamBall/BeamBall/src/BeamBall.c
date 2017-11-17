/*
 * BeamBall.c
 *
 * Created: 09/11/2017 16:36:53
 *  Author: ilangoldman
 */ 

#include <asf.h>
#include "BeamBall.h"


void vReadSensor(void) {
	//puts("Iniciando Leitura do Sensor\r\n");
	
	// clear timer
	tc_get_status(TC_SENSOR,CHANNEL_SENSOR);
	vClearSensorCounter();
	
	gpio_set_pin_high(PIO_TRIGGER);
	delay_us(10);
	gpio_set_pin_low(PIO_TRIGGER);
}

static int flag = 0;

void vMalhaControle(double distance) {
	puts("Executando Malha de Controle\r\n");
	int motorPos = 15;

	int printVar = (int) (distance * 1000);
	printf("distance: %i / %f\r\n",printVar);

	//if (flag == 0) {
		//motorPos = 13;
		//flag = 1;
	//} else {
		//motorPos = 18;
		//flag = 0;
	//}
	//
	//printf("pos: %i \r\n", motorPos);

	//the control shit! VVVV
	//motorPos = (int) dCalculatePID(distance);
	
	// Update Motor position
	vRunMotor(motorPos);
}

void vRunMotor(int pos) {
	//vPWMUpdateDuty(pos);
}
