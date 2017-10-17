/*
 * BeamBall.c
 *
 * Created: 08/10/2017 14:53:05
 *  Author: ilangoldman
 */ 

#include <asf.h>
#include <BeamBall.h>

/* Beam Ball Tasks */

/* LCD */

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

/* Start Sensor Functions */


void TC0_Handler(void) {
	tc_get_status(TC,CHANNEL);

	//sensor_counter++;
	
	/** Muda o estado do LED 1*/
	gpio_toggle_pin(LED1_GPIO);
	//
	//ili9225_set_foreground_color(COLOR_WHITE);
	//ili9225_draw_filled_rectangle(0, 160, ILI9225_LCD_WIDTH, ILI9225_LCD_HEIGHT);
	//ili9225_set_foreground_color(COLOR_BLUE);
	//
	//char buffer[50];
	////int n = sprintf (buffer, "%d", sensor_counter);
	//
	//ili9225_draw_string(10,165,(uint8_t *) "RC estourou");
	printf("RC estourou/r/n");
}

void vConfigureTimer() {
	// t = 10us (fs = 100000)
	// x = 22cm => tmax = 1276 uS (fmax = 783.7Hz)
	uint32_t freq_desired = 783.7; 
	uint32_t ul_tcclk;
	uint32_t ul_div;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();
	
	pmc_enable_periph_clk(ID_TC);

	// trocar para ser a cada 10uS
	tc_find_mck_divisor(freq_desired,ul_sysclk,&ul_div,&ul_tcclk,ul_sysclk);
	tc_init(TC,CHANNEL,TC_CMR_CPCTRG|ul_tcclk);
	uint32_t RC = (ul_sysclk/ul_div)/freq_desired;
    tc_write_rc(TC,CHANNEL,RC);
	
	/********************************************************
	 *	        TC_IER_CPCS  : 	RC Compare					*
	 ********************************************************/
	tc_enable_interrupt(TC,CHANNEL,TC_IER_CPCS);
	NVIC_EnableIRQ(TC_IRQn);
	
	
	
    tc_start(TC,CHANNEL);
	printf("timer configurado\r");
}


/* Start Motor Functions */

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
	g_pwm_channel_led.polarity = PWM_LOW;
	/* Use PWM clock A as source clock */
	g_pwm_channel_led.ul_prescaler = PWM_CMR_CPRE_CLKA;
	/* Period value of output waveform */
	g_pwm_channel_led.ul_period = PERIOD_VALUE;
	/* Duty cycle value of output waveform */
	g_pwm_channel_led.ul_duty = INIT_DUTY_VALUE;
	g_pwm_channel_led.channel = PWM_CHANNEL;

	pwm_channel_init(PWM, &g_pwm_channel_led);

	pwm_channel_enable(PWM, PWM_CHANNEL);
}

void vPWMUpdateDuty (double duty) {
	g_pwm_channel_led.channel = PWM_CHANNEL;
	pwm_channel_update_duty(PWM, &g_pwm_channel_led, duty);
	//gpio_toggle_pin(LED0_GPIO);
}


/* End Motor Functions */