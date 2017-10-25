/*
 * Sensor.c
 *
 * Created: 07/09/2017 16:55:10
 *  Author: ilangoldman
 */ 

#include "asf.h"
#include "conf_board.h"
#include "conf_clock.h"

//****** ADC **********//

#define ADC_CLOCK	6400000

//****** Timer Counter **********//
#define TC			TC0
#define CHANNEL		0
#define ID_TC		ID_TC0
#define TC_IRQn     TC0_IRQn

//****** I/O pin Sensor **********//
#define TRIG 0
#define TRIG_PIN ?


//****** Usando o doc do Sensor **********//

void Start_Sensor() {
	// Set trigger pin high to start the measure
	TRIG_PIN |= (1<<TRIG);
	// Sensor is busy
	Busy_measure =1 ;
	// Delay 10us to sensor finish measure
	//delay(10);
	// Set trigger pin to low
	TRIG_PIN &= ~(1<<TRIG);
}


//****** Usando Conversor ADC **********//


// TODO !!
int readSensor(uint32_t ADC_value) {
	// ler o valor do sensor (pos da bola em relacao a viga)

	// Formula
	// value = ticks / 58; // cm
	
	return 0;
}

// TODO !!
void ADC_Handler (void) {
	// Check the ADC conversion status
	if ((adc_get_status(ADC) & ADC_ISR_DRDY) == ADC_ISR_DRDY) {
		// Get latest digital data value from ADC and can be used by application
		uint32_t result = adc_get_latest_value(ADC);

		// O result vai sempre retornar o valor maximo??
		
		// Add PID calculation logic here
		// resultCM = readSensor (result);
		
		// double ul_duty =  (result*100)/4092;
		// pwm_update_duty(ul_duty);
	}
}


void adc_setup(void) {
	pmc_enable_periph_clk(ID_ADC);
	adc_init(ADC, sysclk_get_main_hz(), ADC_CLOCK , 8);
	adc_configure_timing(ADC, 0, ADC_SETTLING_TIME_3, 1);
	adc_set_resolution(ADC, ADC_MR_LOWRES_BITS_10);
	adc_enable_channel(ADC, ADC_CHANNEL_5);
	adc_enable_interrupt(ADC, ADC_IER_DRDY);
	adc_configure_trigger(ADC, ADC_TRIG_SW, 0);
	NVIC_EnableIRQ(ADC_IRQn);
}

void TC0_Handler(void) {
	tc_get_status(TC,CHANNEL);
	adc_start(ADC);
	//ticks++;
	// count num of tick from ADC start until ready
}

static void tc_setup(void)
{
	uint32_t ul_sysclk = sysclk_get_cpu_hz();
	uint32_t  ul_div;
	uint32_t  ul_tselecionado;
	
	pmc_enable_periph_clk(ID_TC);
	tc_find_mck_divisor(100,ul_sysclk,&ul_div,&ul_tselecionado,ul_sysclk);
	tc_init(TC0,0,TC_CMR_CPCTRG | ul_tselecionado);
	
	int valor = (ul_sysclk/ul_div)/100;
	tc_write_rc(TC0,CHANNEL, valor);
	
	tc_enable_interrupt(TC0,0,TC_IER_CPCS);
	NVIC_EnableIRQ(TC0_IRQn);
	tc_start(TC0,CHANNEL);
}
