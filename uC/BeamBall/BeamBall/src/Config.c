/*
 * Config.c
 *
 * Created: 09/11/2017 16:38:06
 *  Author: ilangoldman
 */ 

#include <asf.h>
#include "BeamBall.h"

char buffer[50];

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

/* Timer Configuration */

unsigned int tc = 0;

// Essa funcao forca outra leitura da malha de controle
void TC0_Handler(void) {
	tc_get_status(TC,CHANNEL);
	vReadSensor();
	char b[50];
	tc++;
	sprintf (b, "%u", tc);
	vWriteLCD(140, 60, (uint8_t*) b);
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
	
	tc_start(TC,CHANNEL);
}

/* ISR Configuration */
unsigned int sensor_counter;
int last_dist = 0;

void vSensorISR(const uint32_t id, const uint32_t index) {
	
	sensor_counter = 0;
	while (pio_get(PIOA,PIO_INPUT,PIO_ECHO) == 1) {
		sensor_counter++;
	} 
	
	double uS_clock_cicles = (sysclk_get_cpu_hz() / 1000000);
	double uScounter = sensor_counter / uS_clock_cicles;
	double dist = (uScounter/58)*32;
	

// 	if (dist > 24) {
// 		if (last_dist > 6 ) dist = 24;
// 		else dist = 5;
// 	}
	vMalhaControle(dist);
	last_dist = dist;
}

uint32_t btn_duty = MIN_DUTY_VALUE;

double KP   =   0.3;
double KI   =   0.1;
double KD	=	0.6;

double getKP(void) {
	return KP;
}

double getKD(void) {
	return KD;
}

double getKI(void) {
	return KI;
}

void clear(void) {
	KP = 0.3;
	KI = 0.1;
	KD = 0.6;
}

void print(void) {
	vWriteLCD(10,160,(uint8_t *) "KP");
	sprintf (buffer, "%.1f", KP);
	vWriteLCD(100,160,(uint8_t *) buffer);
	vWriteLCD(10,180,(uint8_t *) "KI");
	sprintf (buffer, "%.1f", KI);
	vWriteLCD(100,180,(uint8_t *) buffer);
	vWriteLCD(10,200,(uint8_t *) "KD");
	sprintf (buffer, "%.1f", KD);
	vWriteLCD(100,200,(uint8_t *) buffer);
}

// Alteram o PWM diretamente
void vButtonLeftISR(const uint32_t id, const uint32_t index) {
	// aumenta o duty
	//if (btn_duty < MAX_DUTY_VALUE) btn_duty++;
	//vPWMUpdateDuty(btn_duty);
	KD += 0.1;
	vWriteLCD(10,200,(uint8_t *) "KD");
	sprintf (buffer, "%.1f", KD);
	vWriteLCD(100,200,(uint8_t *) buffer);
}

void vButtonRightISR(const uint32_t id, const uint32_t index) {
	// diminui o duty
	//if (btn_duty > MIN_DUTY_VALUE) btn_duty--;
	//vPWMUpdateDuty(btn_duty);
	KP += 0.1;
	vWriteLCD(10,150,(uint8_t *) "KP");
	sprintf (buffer, "%.1f", KP);
	vWriteLCD(100,150,(uint8_t *) buffer);
}

void vConfigureISR() {
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
	
	pio_set_output(PIOA,PIO_TRIGGER,1,0,0);
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
	g_pwm_channel.channel = PWM_CHANNEL;
	pwm_channel_update_duty(PWM, &g_pwm_channel, duty);
	sprintf (buffer, "%lu", duty);
	vWriteLCD(140,80,(uint8_t *) buffer);
}

void vWriteLCD(int x,int y, uint8_t* text) {
	ili93xx_set_foreground_color(COLOR_WHITE);
	ili93xx_draw_filled_rectangle(x-10, y-5, x+100, y+15);

	ili93xx_set_foreground_color(COLOR_BLACK);
	ili93xx_draw_string(x, y, text);
}

void vInitLCD (void) {
	ili93xx_set_foreground_color(COLOR_WHITE);
	ili93xx_draw_filled_rectangle(0, 0, ILI93XX_LCD_WIDTH,ILI93XX_LCD_HEIGHT);

	/** Turn on LCD */
	ili93xx_display_on();
	ili93xx_set_cursor_position(0, 0);
	
	vWriteLCD(10,20,(uint8_t *)"BeamBall\n");
	vWriteLCD(10,80,(uint8_t *)"PWM Duty: ");
	vWriteLCD(10,100,(uint8_t *)"Distancia: ");
	vWriteLCD(10,60,(uint8_t *)"Timer: ");

	vWriteLCD(160,260,(uint8_t *)"Ilan\nLucas\nCarlos");
// 	vWriteLCD(10,280,(uint8_t *)"Timer: ");
// 	vWriteLCD(10,300,(uint8_t *)"Timer: ");
}



void vConfigureLCD(void) {
	/** Enable peripheral clock */
	pmc_enable_periph_clk(ID_SMC);

	/** Configure SMC interface for Lcd */
	smc_set_setup_timing(SMC, ILI93XX_LCD_CS, SMC_SETUP_NWE_SETUP(2)
											| SMC_SETUP_NCS_WR_SETUP(2)
											| SMC_SETUP_NRD_SETUP(2)
											| SMC_SETUP_NCS_RD_SETUP(2));
	smc_set_pulse_timing(SMC, ILI93XX_LCD_CS, SMC_PULSE_NWE_PULSE(4)
											| SMC_PULSE_NCS_WR_PULSE(4)
											| SMC_PULSE_NRD_PULSE(10)
											| SMC_PULSE_NCS_RD_PULSE(10));
	smc_set_cycle_timing(SMC, ILI93XX_LCD_CS, SMC_CYCLE_NWE_CYCLE(10)
											| SMC_CYCLE_NRD_CYCLE(22));
	
	smc_set_mode(SMC, ILI93XX_LCD_CS, SMC_MODE_READ_MODE
									| SMC_MODE_WRITE_MODE);
	
	/** Initialize display parameter */
	g_ili93xx_display_opt.ul_width = ILI93XX_LCD_WIDTH;
	g_ili93xx_display_opt.ul_height = ILI93XX_LCD_HEIGHT;
	g_ili93xx_display_opt.foreground_color = COLOR_BLACK;
	g_ili93xx_display_opt.background_color = COLOR_WHITE;

	/** Switch off backlight */
	aat31xx_disable_backlight();

	/** Initialize LCD */
	ili93xx_init(&g_ili93xx_display_opt);

	/** Set backlight level */
	aat31xx_set_backlight(AAT31XX_AVG_BACKLIGHT_LEVEL);

	vInitLCD();
}