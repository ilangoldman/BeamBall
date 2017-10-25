/*
 * BeamBall.c
 *
 * Created: 24/10/2017 21:27:03
 *  Author: ilangoldman
 */ 

#include <asf.h>
#include "BeamBall.h"


void vReadSensor(void) {
	puts("Iniciando Leitura do Sensor\r\n");
	
	// clear timer
	tc_get_status(TC_SENSOR,CHANNEL_SENSOR);
	vClearSensorCounter();
	
	gpio_pin_is_high(PIO_TRIGGER);
	delay_us(10);
	gpio_pin_is_low(PIO_TRIGGER);
}

void vMalhaControle(double distance) {
	puts("Executando Malha de Controle\r\n");
	
	double motorPos;
	char buffer[50];
	
	// DEBUG LCD
	ili9225_set_foreground_color(COLOR_WHITE);
	ili9225_draw_filled_rectangle(0, 70, ILI9225_LCD_WIDTH, ILI9225_LCD_HEIGHT);
	ili9225_set_foreground_color(COLOR_BLUE);
	ili9225_draw_string(10,75,(uint8_t *) "SD: ");

	sprintf (buffer, "%.2f", distance);
	ili9225_draw_string(95,75,(uint8_t *) buffer);

	// do some control shit!
	motorPos = distance;
	
	// DEBUG LCD
	ili9225_set_foreground_color(COLOR_WHITE);
	ili9225_draw_filled_rectangle(0, 90, ILI9225_LCD_WIDTH, ILI9225_LCD_HEIGHT);
	ili9225_set_foreground_color(COLOR_BLUE);
	ili9225_draw_string(10,95,(uint8_t *) "MP: ");

	sprintf (buffer, "%.2f", motorPos);
	ili9225_draw_string(95,95,(uint8_t *) buffer);

	// Update Motor position
	vRunMotor(motorPos);
}

void vRunMotor(double pos) {
	
	char buffer[50];
	ili9225_set_foreground_color(COLOR_WHITE);
	ili9225_draw_filled_rectangle(0, 110, ILI9225_LCD_WIDTH, ILI9225_LCD_HEIGHT);
	ili9225_set_foreground_color(COLOR_BLUE);
	ili9225_draw_string(10,115,(uint8_t *) "P: ");
	sprintf (buffer, "%.2f", pos);
	ili9225_draw_string(95,115,(uint8_t *) buffer);

	vPWMUpdateDuty(pos);
}
