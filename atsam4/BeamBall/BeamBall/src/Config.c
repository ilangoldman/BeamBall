/*
 * Config.c
 *
 * Created: 09/11/2017 16:38:06
 *  Author: ilangoldman
 */ 

#include <asf.h>
#include "BeamBall.h"


/* UART Configuration */

void vConfigureUART(void) {
	static usart_serial_options_t usart_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits = CONF_UART_STOP_BITS
	};
	usart_serial_init(CONF_UART, &usart_options);
	stdio_serial_init((Usart *)CONF_UART, &usart_options);
}

/* Counter Configuration */

unsigned int sensor_counter = 0;

void vClearSensorCounter() {
	sensor_counter = 0;
}

int iGetSensorCounter() {
	return sensor_counter;
}

void vAddSensorCounter() {
	sensor_counter++;
}

double dGetDistance() {
	printf("sensor: %u\r\n",sensor_counter);
	return (sensor_counter/58);
}

/* Timer Configuration */

// Essa funcao forca outra leitura da malha de controle
void TC0_Handler(void) {
	//puts("Timer Sensor\r\n");
	tc_get_status(TC,CHANNEL);
	vReadSensor();
}

// Essa funcao executa o contador de tempo entre o start do sensor e a sua resposta
void TC1_Handler(void) {
	//puts("Timer Contador\r\n");
	tc_get_status(TC_SENSOR,CHANNEL_SENSOR);
	vAddSensorCounter();
}


void vConfigureTimer() {
	uint32_t ul_tcclk;
	uint32_t ul_div;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();
	uint32_t RC;
	
	/* Configurando o timer da malha de controle */
	
	pmc_enable_periph_clk(ID_TC);
	tc_find_mck_divisor(TC_FREQ,ul_sysclk,&ul_div,&ul_tcclk,ul_sysclk);
	tc_init(TC,CHANNEL,TC_CMR_CPCTRG|ul_tcclk);
	RC = (ul_sysclk/ul_div)/TC_FREQ;
	tc_write_rc(TC,CHANNEL,RC);
	tc_enable_interrupt(TC,CHANNEL,TC_IER_CPCS);
	NVIC_SetPriority(TC_IRQn,TC_PRIORITY);
	NVIC_EnableIRQ(TC_IRQn);
	
	puts("Timer 0 Configurado para 100ms\r\n");
	
	
	/* Configurando o Timer do contador do Sensor */
	
	pmc_enable_periph_clk(ID_TC_SENSOR);
	tc_find_mck_divisor(TC_FREQ_SENSOR,ul_sysclk,&ul_div,&ul_tcclk,ul_sysclk);
	tc_init(TC_SENSOR,CHANNEL_SENSOR,TC_CMR_CPCTRG|ul_tcclk);
	RC = (ul_sysclk/ul_div)/TC_FREQ_SENSOR;
	tc_write_rc(TC_SENSOR,CHANNEL_SENSOR,RC);
	tc_enable_interrupt(TC_SENSOR,CHANNEL_SENSOR,TC_IER_CPCS);
	NVIC_SetPriority(TC_IRQn_SENSOR,TC_SENSOR_PRIORITY);
	NVIC_EnableIRQ(TC_IRQn_SENSOR);
	
	puts("Timer 1 Configurado para 10us\r\n");
	
	tc_start(TC,CHANNEL);
    tc_start(TC_SENSOR,CHANNEL_SENSOR);
}

/* ISR Configuration */

void vSensorISR(const uint32_t id, const uint32_t index) {
	puts("Sensor ISR \r\n");
	
	double distance = dGetDistance();
	vMalhaControle(distance);
}

uint32_t btn_duty = MIN_DUTY_VALUE;

// Alteram o PWM diretamente
void vButtonLeftISR(const uint32_t id, const uint32_t index) {
	// aumenta o duty
	if (btn_duty < MAX_DUTY_VALUE) btn_duty++;
	vPWMUpdateDuty(btn_duty);
	
	puts("Button Left ISR \r\n");
}

void vButtonRightISR(const uint32_t id, const uint32_t index) {
	// diminui o duty
	if (btn_duty > MIN_DUTY_VALUE) btn_duty--;
	vPWMUpdateDuty(btn_duty);
	
	puts("Button Right ISR \r\n");
}

void vConfigureISR() {
	puts("Configuracao Sensor ISR \r\n");
	
	/* Configuracao da ISR no PIO_ECHO do Sensor */
	pio_set_input(PIOA, PIO_ECHO, PIO_DEBOUNCE);
	pio_pull_down(PIOA,PIO_ECHO,1);
	pio_handler_set(PIOA,ID_PIOA,PIO_ECHO,PIO_IT_RISE_EDGE,vSensorISR);
	pio_enable_interrupt(PIOA,PIO_ECHO);
	NVIC_SetPriority(PIOA_IRQn, SENSOR_PRIORITY);
	NVIC_EnableIRQ(PIOA_IRQn);
	
	/* ISR no Botao */
	pio_set_input(PIOB, PIO_BUTTON_LEFT, PIO_PULLUP | PIO_DEBOUNCE);
	pio_handler_set(PIOB,ID_PIOB,PIO_BUTTON_LEFT,PIO_IT_RISE_EDGE,vButtonLeftISR);
	pio_enable_interrupt(PIOB,PIO_BUTTON_LEFT);
	NVIC_SetPriority(PIOB_IRQn, BUTTON_PRIORITY);
	NVIC_EnableIRQ(PIOB_IRQn);
	
	pio_set_input(PIOC, PIO_BUTTON_RIGTH, PIO_PULLUP | PIO_DEBOUNCE);
	pio_handler_set(PIOC,ID_PIOC,PIO_BUTTON_RIGTH,PIO_IT_RISE_EDGE,vButtonRightISR);
	pio_enable_interrupt(PIOC,PIO_BUTTON_RIGTH);
	NVIC_SetPriority(PIOC_IRQn, BUTTON_PRIORITY);
	NVIC_EnableIRQ(PIOC_IRQn);
}

/* PWM Configuration */
void vConfigurePWM() {
	pmc_enable_periph_clk(ID_PWM);
	pwm_channel_disable(PWM, PWM_CHANNEL);
	pwm_clock_t clock_setting = {
		.ul_clka = PWM_FREQUENCY * PERIOD_VALUE,
		.ul_clkb = 0,
		.ul_mck = sysclk_get_cpu_hz()
	};
	pwm_init(PWM, &clock_setting);
	g_pwm_channel.alignment = PWM_ALIGN_LEFT;
	g_pwm_channel.polarity = PWM_LOW;
	g_pwm_channel.ul_prescaler = PWM_CMR_CPRE_CLKA;
	g_pwm_channel.ul_period = PERIOD_VALUE;
	g_pwm_channel.ul_duty = INIT_DUTY_VALUE;
	g_pwm_channel.channel = PWM_CHANNEL;
	pwm_channel_init(PWM, &g_pwm_channel);
	pwm_channel_disable_interrupt(PWM, PWM_CHANNEL, 0);
	pwm_channel_enable(PWM, PWM_CHANNEL);
}

void vPWMUpdateDuty (uint32_t duty) {
	puts("Update Duty!\r\n");
	g_pwm_channel.channel = PWM_CHANNEL;
	pwm_channel_update_duty(PWM, &g_pwm_channel, duty);
}

// Descomente a funcao de baixo para ativar a interrupcao do PWM
/*
void PWM_Handler(void) {
	gpio_toggle_pin(LED0_GPIO);
	vPWMUpdateDuty(6);
}
*/