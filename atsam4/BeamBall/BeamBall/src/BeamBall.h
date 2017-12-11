
/*
 * BeamBall.h
 *
 * Created: 24/10/2017 21:27:26
 *  Author: ilangoldman
 */ 

#include "stdio_serial.h"
#include "conf_board.h"
#include "conf_clock.h"

#ifndef BEAMBALL_H_
#define BEAMBALL_H_

double getKP(void);
double getKI(void);
double getKD(void);
void clear(void);
void print(void);

/* BeamBall Priority */

#define SENSOR_PRIORITY		1
#define TC_PRIORITY			3
#define PWM_PRIORITY		2
#define BUTTON_PRIORITY		2

/* UART Defines */

#define CONF_UART_CHAR_LENGTH  US_MR_CHRL_8_BIT		// define o tamanho de dados a ser enviado
#define CONF_UART_STOP_BITS    US_MR_NBSTOP_1_BIT	// define o tamanho do bit de parada

/* Timer Defines */

// Timer da Malha de Controle
#define TC					TC0
#define CHANNEL				0
#define ID_TC				ID_TC0
#define TC_IRQn				TC0_IRQn
#define TC_FREQ				10							

/* PWM Defines */

#define PWM_CHANNEL			PIN_PWM_LED1_CHANNEL	//PA20
#define PWM_FREQUENCY		50
#define PERIOD_VALUE		200
#define MIN_DUTY_VALUE		10						// Posicao 0 graus
#define MAX_DUTY_VALUE		18						// Posicao 180 graus
#define INIT_DUTY_VALUE		13

pwm_channel_t g_pwm_channel;


/* PIO Defines */

#define PIO_BUTTON_LEFT		PIO_PB3		//PB3
#define PIO_BUTTON_RIGTH	PIO_PC12	//PC12

/* Sensor Defines */

#define PIO_ECHO			PIO_PA15	//PA15
#define PIO_TRIGGER			PIO_PA19	//PA19

/* LCD Defines */

#define ILI93XX_LCD_CS      1
struct ili93xx_opt_t g_ili93xx_display_opt;


/* End Defines */


/* BEAM BALL FUNCTIONS */


/* Sensor Functions */

void vConfigureISR(void);
void vSensorISR(const uint32_t id, const uint32_t index);
void vButtonLeftISR(const uint32_t id, const uint32_t index);
void vButtonRightISR(const uint32_t id, const uint32_t index);

void vClearSensorCounter(void);
int iGetSensorCounter(void);
void vAddSensorCounter(void);
double dGetDistance(void);


// This task will start the sensor
void vReadSensor(void);

/* Malha de Controle Functions */

/**
 * This task, when activated, will calculate the new position 
 * of the motor based on the sensor data
 */
void vMalhaControle(double distance);

/* Motor Functions */

//This task, when activated, will change the position of servo motor
void vRunMotor(int pos);


/* CONFIGURATION FUNCTIONS */

/* UART Functions */
void vConfigureUART(void);

/* Timer Functions */

// 10us Timer
void vConfigureTimer(void);

/* PWM Function */

void vConfigurePWM(void);
void vPWMUpdateDuty (uint32_t duty);

/* LCD Functions */

void vInitLCD (void);
void vWriteLCD(int x,int y, uint8_t* text);
void vConfigureLCD(void);


#endif /* BEAMBALL_H_ */