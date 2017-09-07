/*
 * Motor.c
 * Implementation of PWM functionalities in the control of the Servo Motor
 *
 * Created: 07/09/2017 15:57:24
 *  Author: ilangoldman
 */ 

#include "asf.h"
#include "conf_board.h"

/** PWM frequency in Hz */
#define PWM_FREQUENCY      20000
/** Period value of PWM output waveform */
#define PERIOD_VALUE       100
/** Initial duty cycle value */
#define INIT_DUTY_VALUE    0

/** PWM channel instance for LEDs */
pwm_channel_t g_pwm_channel_led;


void pwm_setup(void) {
	/* Enable PWM peripheral clock */
	pmc_enable_periph_clk(ID_PWM);

	/* Disable PWM channels for LEDs */
	pwm_channel_disable(PWM, PWM_CHANNEL_0);
	//pwm_channel_disable(PWM, PIN_PWM_LED1_CHANNEL);

	/* Set PWM clock A as PWM_FREQUENCY*PERIOD_VALUE (clock B is not used) */
	pwm_clock_t clock_setting = {
		.ul_clka = PWM_FREQUENCY * PERIOD_VALUE,
		.ul_clkb = 0,
		.ul_mck = sysclk_get_cpu_hz()
	};

	pwm_init(PWM, &clock_setting);

	g_pwm_channel_led.ul_prescaler = PWM_CMR_CPRE_CLKA;
	/* Period value of output waveform */
	g_pwm_channel_led.ul_period = PERIOD_VALUE;
	/* Duty cycle value of output waveform */
	g_pwm_channel_led.ul_duty = 50;
	g_pwm_channel_led.channel = PWM_CHANNEL_0;

	pwm_channel_init(PWM, &g_pwm_channel_led);

	/* Enable PWM channels for LEDs */
	pwm_channel_enable(PWM, PWM_CHANNEL_0);
	//pwm_channel_enable(PWM, PIN_PWM_LED1_CHANNEL);
}


void pwm_update_duty (double duty) {
	g_pwm_channel_led.channel = PIN_PWM_LED0_CHANNEL;
	pwm_channel_update_duty(PWM, &g_pwm_channel_led, duty);
}


// TODO !!
void runMotor (int direction, int value) {
	// girar motor direita/esquerda no respectivo valor
}