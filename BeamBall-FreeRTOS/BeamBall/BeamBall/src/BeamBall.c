/*
 * BeamBall.c
 *
 * Created: 08/10/2017 14:53:05
 *  Author: ilangoldman
 */ 

#include <asf.h>
#include <BeamBall.h>

//uint32_t sensor_counter = 0;

xTaskHandle *pxTaskSensor = NULL;
xTaskHandle *pxTaskControle = NULL;
xTaskHandle *pxTaskMotor = NULL;


xQueueHandle xQueueControle = NULL;
xQueueHandle xQueueMotor = NULL;


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

//void vSensorISR(const uint32_t id, const uint32_t index) {
	//// write in queue sensorISR
	//gpio_toggle_pin(LED1_GPIO);
	////if (xQueueSendFromISR(xQueueSensor,&sensor_counter,NULL) != pdPASS)
		////printf("Error Sending data to xQueueSensor\n");
//}
//
//#define PIO_ECHO PIO_PA15
//
//void vConfigureSensorISR() {
	//// configure sensor interrupt
	//printf("Configuracao Sensor ISR \n");
	//
	//// pio no btn BP2 (LEFT) trocar para outro PIO
	//
	//pio_set_input(PIOA, PIO_ECHO, PIO_PULLUP | PIO_DEBOUNCE);
	//pio_handler_set(PIOA,ID_PIOA,PIO_ECHO,PIO_IT_RISE_EDGE,vSensorISR);
	//pio_enable_interrupt(PIOA,PIO_ECHO);
	//NVIC_SetPriority(PIOA_IRQn, 1 );
	//NVIC_EnableIRQ(PIOA_IRQn);
//}

void TC0_Handler(void) {
	tc_get_status(TC,CHANNEL);

	//sensor_counter++;
	
	/** Muda o estado do LED 1*/
	gpio_toggle_pin(LED1_GPIO);
	
	ili9225_set_foreground_color(COLOR_WHITE);
	ili9225_draw_filled_rectangle(0, 160, ILI9225_LCD_WIDTH, ILI9225_LCD_HEIGHT);
	ili9225_set_foreground_color(COLOR_BLUE);
	
	char buffer[50];
	//int n = sprintf (buffer, "%d", sensor_counter);
	
	ili9225_draw_string(10,165,(uint8_t *) buffer);
}

void vConfigureTimer() {
	uint32_t freq_desired = 100000; // 10us
	uint32_t ul_tcclk;
	uint32_t ul_div;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();
	
	pmc_enable_periph_clk(ID_TC);
	tc_find_mck_divisor(freq_desired,ul_sysclk,&ul_div,&ul_tcclk,ul_sysclk);
	tc_init(TC,CHANNEL,TC_CMR_CPCTRG|ul_tcclk);
	uint32_t counter = (ul_sysclk/ul_div)/freq_desired;
    tc_write_rc(TC,CHANNEL,counter);
	
	/********************************************************
	 *	        TC_IER_CPCS  : 	RC Compare					*
	 ********************************************************/
	tc_enable_interrupt(TC,CHANNEL,TC_IER_CPCS);
	NVIC_EnableIRQ(TC_IRQn);
    tc_start(TC,CHANNEL);
	printf("timer configurado\r");
}


//void vTaskReadSensor(void *pvParameters)
//{
	//UNUSED(pvParameters);
	//int counter = 0;
	//int distanceCM = 0;
	//for (;;) {
		//sensor_counter = 0;
		//gpio_pin_is_high(PIO_PA17);
		//// vTaskDelay(1us); // implement taskDelay as a for loop
		//delay_us(10);
		//gpio_pin_is_low(PIO_PA17);
		//
		//// read from queue ISR
		//if (xQueueReceiveFromISR(&xQueueSensor,&counter,NULL) != pdPASS)
			//printf("Error Receiving data from xQueueSensor\n");
			//
		//distanceCM = counter/58;
		//
//// 		if (xQueueSend(xQueueControle,&distanceCM,2) != pdPASS)
//// 			printf("Error Sending data to xQueueControle\n");
		//
		////vTaskDelay(1);
		//// go for a new read
	//}
//}

/* End Sensor Functions */


/* Start Malha de Controle Functions */

void vTaskMalhaControle(void *pvParameters)
{
	UNUSED(pvParameters);
	int sensorDistance = 0;
	double motorPos = 0;
	for (;;) {
		// read from sensor queue distanceCM
		if (xQueueReceive(xQueueControle,&sensorDistance,2) != pdPASS)
			printf("Error Receiving data from xQueueSensor\n");
		
		// do some control shit!

		// writes to queue Motor Position
		if (xQueueSend(xQueueMotor,&motorPos,2) != pdPASS)
			printf("Error Sending data to xQueueMotor\n");
	}
}

/* End Malha de Controle Functions */


/* Start Motor Functions */

void vConfigurePWM() {
	pmc_enable_periph_clk(ID_PWM);
	pwm_channel_disable(PWM, PWM_CHANNEL_0);
	pwm_clock_t clock_setting = {
		.ul_clka = PWM_FREQUENCY * PERIOD_VALUE,
		.ul_clkb = 0,
		.ul_mck = sysclk_get_cpu_hz()
	};

	pwm_init(PWM, &clock_setting);
	g_pwm_channel_led.ul_prescaler = PWM_CMR_CPRE_CLKA;
	g_pwm_channel_led.ul_period = PERIOD_VALUE;
	g_pwm_channel_led.ul_duty = 50;
	g_pwm_channel_led.channel = PWM_CHANNEL_0;
	pwm_channel_init(PWM, &g_pwm_channel_led);
	pwm_channel_enable(PWM, PWM_CHANNEL_0);
}

void vPWMUpdateDuty (double duty) {
	g_pwm_channel_led.channel = PIN_PWM_LED0_CHANNEL;
	pwm_channel_update_duty(PWM, &g_pwm_channel_led, duty);
}


void vTaskRunMotor(void *pvParameters)
{
	UNUSED(pvParameters);
	double pos = 0;
	for (;;) {
		// read from queue Malha de Controle Motor Pos
		if (xQueueReceive(xQueueMotor,&pos,2) != pdPASS)
			printf("Error Receiving data from xQueueMotor\n");
		
		vPWMUpdateDuty(pos);
	}
}

/* End Motor Functions */