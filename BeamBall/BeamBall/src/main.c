
/*
 * Controle de Poscionamento de uma Bola em cima de uma Barra
 * 
 * Projeto de Microcontroladores e Controle
 * 
 */
#include <asf.h>
#include "Motor.c"
#include "Sensor.c"
#include "PID.c"


int main (void)
{
	sysclk_init();
	board_init();

	pwm_setup();
	
	adc_setup();
	tc_setup();
	
	while (1) {
		// Run code
	}	
}
