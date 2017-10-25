/**
 *
 *  Beam Ball Main File
 *
 */
#include <asf.h>
#include "BeamBall.h"

int main (void)
{
	sysclk_init();
	board_init();
	vConfigureUART();

	puts("Inicializing Beam Ball\r\n");

	vConfigurePWM();
	vConfigureSensorISR();
	vConfigureTimer();
	
	vConfigureLCD();
	drawLCD();
	
	while (1) { /* RUN APPLICATION */ }
	
	puts("Error!!\r\n");	
	return 0;
}
