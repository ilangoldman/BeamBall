/*
 * Config.c
 *
 * Created: 24/10/2017 21:27:14
 *  Author: ilangoldman
 */ 

#include <asf.h>
#include "BeamBall.h"

static int sensor_counter = 0;

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
	return (sensor_counter/58);
}


/* UART Configuration */

void vConfigureUART(void) {
	
	// cria a variavel serial para envio de dados
	// usando os valores definidos acima

	static usart_serial_options_t usart_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits = CONF_UART_STOP_BITS
	};
	
	// define a porta serial para a UART e com a variavel definida acima
	usart_serial_init(CONF_UART, &usart_options);

	// define a saida generica para ser o uart
	stdio_serial_init((Usart *)CONF_UART, &usart_options);
}


/* LCD Configuration */

void SPI_Handler(void) {
	ili9225_spi_handler();
}

void vConfigureLCD(void) {
	/* Initialize display parameter */
	g_ili9225_display_opt.ul_width = ILI9225_LCD_WIDTH;
	g_ili9225_display_opt.ul_height = ILI9225_LCD_HEIGHT;
	g_ili9225_display_opt.foreground_color = COLOR_BLACK;
	g_ili9225_display_opt.background_color = COLOR_WHITE;

	/* Switch off backlight */
	aat31xx_disable_backlight();

	/* Initialize LCD */
	ili9225_init(&g_ili9225_display_opt);

	/* Set backlight level */
	aat31xx_set_backlight(AAT31XX_AVG_BACKLIGHT_LEVEL);
}

void drawLCD(void) {
	/* Draw filled rectangle with white color */
	ili9225_set_foreground_color(COLOR_WHITE);
	ili9225_draw_filled_rectangle(0, 0, ILI9225_LCD_WIDTH, ILI9225_LCD_HEIGHT);
	
	/* Turn on LCD */
	ili9225_display_on();
	ili9225_set_cursor_position(0,0);
	
	/* Draw text and basic shapes on the LCD */
	ili9225_set_foreground_color(COLOR_BLUE);
	ili9225_draw_string(10, 10, (uint8_t *)"Beam Ball");
	
	//ili9225_draw_line(0, 11, ILI9225_LCD_WIDTH, 12);

	/* Draw three circle with red, green and blue color */
	ili9225_set_foreground_color(COLOR_RED);
	ili9225_draw_circle(60, 80, 30);
	ili9225_set_foreground_color(COLOR_GREEN);
	ili9225_draw_circle(60, 120, 30);
	ili9225_set_foreground_color(COLOR_BLUE);
	ili9225_draw_circle(60, 160, 30);

}

/* Timer Configuration */

// Essa funcao forca outra leitura da malha de controle
void TC0_Handler(void) {
	tc_get_status(TC,CHANNEL);
	
	puts("Nova Leitura\r\n");
	
	vReadSensor();
}

// Essa funcao executa o contador de tempo entre o start do sensor e a sua resposta
void TC1_Handler(void) {
	tc_get_status(TC_SENSOR,CHANNEL_SENSOR);
	
	vAddSensorCounter();
	
	// Descomente as linhas de baixo para ver o contador no LCD
	
	//ili9225_set_foreground_color(COLOR_WHITE);
	//ili9225_draw_filled_rectangle(0, 160, ILI9225_LCD_WIDTH, ILI9225_LCD_HEIGHT);
	//ili9225_set_foreground_color(COLOR_BLUE);
	//
	//char buffer[50];
	////int n = sprintf (buffer, "%d", iGetSensorCounter());
	//
	//ili9225_draw_string(10,165,(uint8_t *) "RC estourou");
	
	puts("Contando\r\n");
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
	NVIC_SetPriority(TC_IRQn_SENSOR,TC_PRIORITY);
	NVIC_EnableIRQ(TC_IRQn);
    tc_start(TC,CHANNEL);

	puts("Timer Configurado para 100ms\r\n");
	
	
	/* Configurando o Timer do contador do Sensor */
	
	pmc_enable_periph_clk(ID_TC_SENSOR);
	tc_find_mck_divisor(TC_FREQ_SENSOR,ul_sysclk,&ul_div,&ul_tcclk,ul_sysclk);
	tc_init(TC_SENSOR,CHANNEL_SENSOR,TC_CMR_CPCTRG|ul_tcclk);
	RC = (ul_sysclk/ul_div)/TC_FREQ_SENSOR;
    tc_write_rc(TC_SENSOR,CHANNEL_SENSOR,RC);
	
	//	TC_IER_CPCS = RC Compare
	tc_enable_interrupt(TC_SENSOR,CHANNEL_SENSOR,TC_IER_CPCS);
	NVIC_SetPriority(TC_IRQn_SENSOR,TC_SENSOR_PRIORITY);
	NVIC_EnableIRQ(TC_IRQn_SENSOR);
    tc_start(TC_SENSOR,CHANNEL_SENSOR);
	
	puts("Timer Configurado para 10us\r\n");
}

/* ISR Configuration */

void vConfigureSensorISR() {
	puts("Configuracao Sensor ISR \r\n");
	
	/* Configuracao da ISR no PIO_ECHO do Sensor */
	
	// Descomente essas linhas para ativar a interrupcao do Sensor
	// OBS: Altere o PIO_ECHO para um PIO apropriado
	/*
	pio_set_input(PIOA, PIO_ECHO, PIO_DEBOUNCE);
	pio_pull_down(PIOA,PIO_ECHO,1);
	pio_handler_set(PIOA,ID_PIOA,PIO_ECHO,PIO_IT_RISE_EDGE,vSensorISR);
	pio_enable_interrupt(PIOA,PIO_ECHO);
	NVIC_SetPriority(PIOA_IRQn, SENSOR_PRIORITY);
	NVIC_EnableIRQ(PIOA_IRQn);
	*/

	/* DEBUG -- ISR no Botao */
	
	// Comente as linhas de baixo para ativar a ISR pelo botao
	
	pio_set_input(PIOA, PIO_BUTTON_LEFT, PIO_PULLUP | PIO_DEBOUNCE);
	pio_handler_set(PIOA,ID_PIOA,PIO_BUTTON_LEFT,PIO_IT_RISE_EDGE,vButtonLeftISR);
	pio_enable_interrupt(PIOA,PIO_BUTTON_LEFT);
	NVIC_SetPriority(PIOA_IRQn, BUTTON_PRIORITY);
	NVIC_EnableIRQ(PIOA_IRQn);
	
	pio_set_input(PIOA, PIO_BUTTON_RIGTH, PIO_PULLUP | PIO_DEBOUNCE);
	pio_handler_set(PIOA,ID_PIOA,PIO_BUTTON_RIGTH,PIO_IT_RISE_EDGE,vButtonRightISR);
	pio_enable_interrupt(PIOA,PIO_BUTTON_RIGTH);
	NVIC_SetPriority(PIOA_IRQn, BUTTON_PRIORITY);
	NVIC_EnableIRQ(PIOA_IRQn);
	
	
	puts("Fim Config Sensor ISR \r\n");
}

void vSensorISR(const uint32_t id, const uint32_t index) {
	double distance = dGetDistance();
	vMalhaControle(distance);
	
	puts("Fim Sensor ISR \r\n");
}

uint32_t btn_duty = MIN_DUTY_VALUE;

// Alteram o PWM diretamente
void vButtonLeftISR(const uint32_t id, const uint32_t index) {
	// aumenta o duty
	if (btn_duty < MAX_DUTY_VALUE) btn_duty++;
	vPWMUpdateDuty(btn_duty);
	
	puts("Fim Button Left ISR \r\n");
}

void vButtonRightISR(const uint32_t id, const uint32_t index) {
	// diminui o duty
	if (btn_duty > MIN_DUTY_VALUE) btn_duty--;
	vPWMUpdateDuty(btn_duty);
	
	puts("Fim Button Right ISR \r\n");
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

	/* Initialize PWM channel for LED0 */
	/* Period is left-aligned */
	g_pwm_channel_led.alignment = PWM_ALIGN_LEFT;
	/* Output waveform starts at a low level */
	g_pwm_channel_led.polarity = PWM_HIGH;
	/* Use PWM clock A as source clock */
	g_pwm_channel_led.ul_prescaler = PWM_CMR_CPRE_CLKA;
	/* Period value of output waveform */
	g_pwm_channel_led.ul_period = PERIOD_VALUE;
	/* Duty cycle value of output waveform */
	g_pwm_channel_led.ul_duty = MIN_DUTY_VALUE;
	g_pwm_channel_led.channel = PWM_CHANNEL;

	pwm_channel_init(PWM, &g_pwm_channel_led);
	
	// Descomente as linhas de baixo para colocar a interrupcao no PWM
	/*
	pwm_channel_enable_interrupt(PWM, PWM_CHANNEL, 0);
	
	NVIC_DisableIRQ(PWM_IRQn);
	NVIC_ClearPendingIRQ(PWM_IRQn);
	NVIC_SetPriority(PWM_IRQn, PWM_PRIORITY);
	NVIC_EnableIRQ(PWM_IRQn);
	*/

	pwm_channel_enable(PWM, PWM_CHANNEL);
}

void vPWMUpdateDuty (double duty) {
	g_pwm_channel_led.channel = PWM_CHANNEL;
	pwm_channel_update_duty(PWM, &g_pwm_channel_led, duty);
}

// Descomente a funcao de baixo para ativar a interrupcao do PWM
/*
void PWM_Handler(void) {
	gpio_toggle_pin(LED0_GPIO);
	vPWMUpdateDuty(6);
}
*/