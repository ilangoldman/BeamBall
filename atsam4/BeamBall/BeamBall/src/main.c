/**
 *
 *  Beam Ball Main File
 *
 */
#include <asf.h>
#include "BeamBall.h"             

int main (void) {

	sysclk_init();
	board_init();
	vConfigureUART();

	puts("Inicializing Beam Ball\r\n");

	double setpoint = 12;
	vConfigurePID (setpoint,KP,KI,KD,MAX_DUTY_VALUE,MIN_DUTY_VALUE);

	vConfigurePWM();
	vConfigureISR();
	vConfigureTimer();
	
	while (1) { /* RUN APPLICATION */ }
	
	puts("Erroooooooouu!!\r\n");	
	return 0;
}
