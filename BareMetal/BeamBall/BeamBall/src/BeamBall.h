/*
 * BeamBall.h
 *
 * Created: 24/10/2017 21:27:26
 *  Author: ilangoldman
 */ 


#ifndef BEAMBALL_H_
#define BEAMBALL_H_

/* BeamBall Priority */

#define SENSOR_PRIORITY		1
#define TC_PRIORITY			2
#define TC_SENSOR_PRIORITY	5
#define PWM_PRIORITY		6
#define BUTTON_PRIORITY		6

/* UART Defines */

#define CONF_UART              UART0				// definicao da porta
#define CONF_UART_BAUDRATE     115200				// define o baud rate (velocidade)
#define CONF_UART_CHAR_LENGTH  US_MR_CHRL_8_BIT		// define o tamanho de dados a ser enviado
#define CONF_UART_PARITY       US_MR_PAR_NO			// define a paridade
#define CONF_UART_STOP_BITS    US_MR_NBSTOP_1_BIT	// define o tamanho do bit de parada

/* Timer Defines */

// Timer da Malha de Controle
#define TC					TC0
#define CHANNEL				0
#define ID_TC				ID_TC0
#define TC_IRQn				TC0_IRQn
#define TC_FREQ				1000							//100mS

// Timer do Contador do Sensor
#define TC_SENSOR			TC1
#define CHANNEL_SENSOR		1
#define ID_TC_SENSOR		ID_TC1
#define TC_IRQn_SENSOR		TC1_IRQn
#define TC_FREQ_SENSOR		100000					// t = 10uS (fs = 100000)

/* PWM Defines */

#define PWM_CHANNEL			PWM_CHANNEL_0
#define PWM_FREQUENCY		50
#define PERIOD_VALUE		100
#define MIN_DUTY_VALUE		1						// Posicao 0 graus
#define MAX_DUTY_VALUE		9						// Posicao 180 graus

pwm_channel_t g_pwm_channel_led;


/* LCD Display Defines */

struct ili9225_opt_t g_ili9225_display_opt;

/* PIO Defines */

#define PIO_BUTTON_LEFT		PIO_PA15
#define PIO_BUTTON_RIGTH	PIO_PA16

/* Sensor Defines */

#define PIO_ECHO			PIO_PA15
#define PIO_TRIGGER			LED0_GPIO

/* End Defines */


/* BEAM BALL FUNCTIONS */


/* Sensor Functions */

void vConfigureSensorISR(void);
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
void vRunMotor(double pos);


/* CONFIGURATION FUNCTIONS */

/* UART Functions */
void vConfigureUART(void);

/* LCD Display Functions */

void vConfigureLCD(void);
void drawLCD(void);

/* Timer Functions */

// 10us Timer
void vConfigureTimer(void);

/* PWM Function */

void vConfigurePWM(void);
void vPWMUpdateDuty (double duty);

/* Buttons Function */
void vConfigureButton(void);

#endif /* BEAMBALL_H_ */