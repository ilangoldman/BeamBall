/**
 *
 *  Beam Ball Main File
 *
 *	Created By:
 *		Ilan Goldman
 *		Lucas Deliberato
 *		Carlos Eduardo
 *
 */
#include <asf.h>
#include "BeamBall.h"       

int main (void) {

	sysclk_init();
	board_init();
	vConfigureUART();
	vConfigurePWM();
	vConfigureISR();
	vConfigureTimer();
	
	vConfigureLCD();
	clear();
	print();
	
	while (1) { /* RUN APPLICATION */ }
	
	return 0;
}
