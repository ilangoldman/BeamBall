/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include <asf.h>

//#include "sam.h"

volatile uint32_t cnt;

int main(void)
{
	/* Disable the watchdog */
	WDT->WDT_MR = WDT_MR_WDDIS;

	/* Initialize the SAM system */
	board_init();
	//SystemInit();

	// disable the PIO (peripheral controls the pin)
	PIOA->PIO_PDR = PIO_PDR_P23;
	// select alternate function B (PWML0) for pin PA19
	PIOA->PIO_ABCDSR[0] |= PIO_ABCDSR_P23;
	PIOA->PIO_ABCDSR[1] &= ~PIO_ABCDSR_P23;
	// Enable the PWM peripheral from the Power Manger
	PMC->PMC_PCER0 = (1 << ID_PWM);
	// Select the Clock to run at the MCK (4MHz)
	PWM->PWM_CH_NUM[0].PWM_CMR = PWM_CMR_CPRE_MCK;
	// select the period 10msec
	PWM->PWM_CH_NUM[0].PWM_CPRD = 40000;
	// select the duty cycle
	PWM->PWM_CH_NUM[0].PWM_CDTY = 20000;
	// enable the channel
	PWM->PWM_ENA = PWM_ENA_CHID0;

	while (1)
	{
		cnt++;
		if ((cnt%40000) == 0) PWM->PWM_CH_NUM[0].PWM_CDTY = 10000;
		else if ((cnt%30000) == 0) PWM->PWM_CH_NUM[0].PWM_CDTY = 30000;
	}
}