/*
 * BeamBall.h
 *
 * Created: 08/10/2017 14:54:17
 *  Author: ilangoldman
 */ 


#ifndef BEAMBALL_H_
#define BEAMBALL_H_

/* Start Defines */

#define TC			TC0
#define CHANNEL		0
#define ID_TC		ID_TC0
#define TC_IRQn     TC0_IRQn

#define PWM_FREQUENCY      50
#define PERIOD_VALUE       100
#define INIT_DUTY_VALUE    0

pwm_channel_t g_pwm_channel_led;
struct ili9225_opt_t g_ili9225_display_opt;

#define PIO_ECHO PIO_PA15
#define PIO_TRIGGER LED0_GPIO

/* End Defines */


/* Start LCD Functions */

void vConfigureLCD(void);

void drawLCD(void);

/* End LCD Functions */


/* Start Sensor Functions */

//void vSensorISR(const uint32_t id, const uint32_t index);
//
//void vConfigureSensorISR();
//
//// 10us Timer
//void vConfigureTimer();
//
////This task, when activated, will read the ultrassonic sensor data
//void vTaskReadSensor(void *pvParameters);

/* End Sensor Functions */


/* Start Malha de Controle Functions */

/**
 * This task, when activated, will calculate the new position 
 * of the motor based on the sensor data
 */
void vTaskMalhaControle(void *pvParameters);

/* End Malha de Controle Functions */


/* Start Motor Functions */

void vConfigurePWM();
void vPWMUpdateDuty (double duty);

//This task, when activated, will change the position of servo motor
void vTaskRunMotor(void *pvParameters);

/* End Motor Functions */

#endif /* BEAMBALL_H_ */