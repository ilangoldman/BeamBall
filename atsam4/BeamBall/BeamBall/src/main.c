/**
 *
 *  Beam Ball Main File
 *
 */
#include <asf.h>
#include "BeamBall.h"


int main (void)
{
	sysclk_init();
	board_init();
	vConfigureUART();

	puts("Inicializing Beam Ball\r\n");

	vConfigurePWM();
	vConfigureISR();
	vConfigureTimer();
	
	while (1) { /* RUN APPLICATION */ }
	
	puts("Erroooooooouu!!\r\n");	
	return 0;
}

//
//
//#include "asf.h"
//#include "stdio_serial.h"
//#include "conf_board.h"
//#include "conf_clock.h"
//
//#define PWM_FREQUENCY      50
//#define PERIOD_VALUE       100
//#define INIT_DUTY_VALUE    5
//#define PWM_CHANNEL		   PIN_PWM_LED1_CHANNEL
//
//#define PIO_BUTTON_LEFT		PIO_PB3
//#define PIO_BUTTON_RIGTH	PIO_PC12
//#define BUTTON_PRIORITY		2
//
///** PWM channel instance for LEDs */
//pwm_channel_t g_pwm_channel;
//
//static void configure_console(void)
//{
	//const usart_serial_options_t uart_serial_options = {
		//.baudrate = CONF_UART_BAUDRATE,
		//.paritytype = CONF_UART_PARITY,
	//};
//
	///* Configure console UART. */
	//sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	//stdio_serial_init(CONF_UART, &uart_serial_options);
//}
//
//void vPWMUpdateDuty (uint32_t duty) {
	//puts("Update Duty!\r\n");
	//g_pwm_channel.channel = PWM_CHANNEL;
	//pwm_channel_update_duty(PWM, &g_pwm_channel, duty);
//}
//
//uint32_t btn_duty = INIT_DUTY_VALUE;
//
//// Alteram o PWM diretamente
//void vButtonLeftISR(const uint32_t id, const uint32_t index) {
	//// aumenta o duty
	//if (btn_duty < 10) btn_duty++;
	//vPWMUpdateDuty(btn_duty);
	//
	//puts("Button Left ISR \r\n");
//}
//
//void vButtonRightISR(const uint32_t id, const uint32_t index) {
	//// diminui o duty
	//if (btn_duty > 5) btn_duty--;
	//vPWMUpdateDuty(btn_duty);
	//
	//puts("Button Right ISR \r\n");
//}
//
//void vConfigureISR() {
	//puts("Configuracao Sensor ISR \r\n");
	//
	///* ISR no Botao */
	//pio_set_input(PIOB, PIO_BUTTON_LEFT, PIO_PULLUP | PIO_DEBOUNCE);
	//pio_handler_set(PIOB,ID_PIOB,PIO_BUTTON_LEFT,PIO_IT_RISE_EDGE,vButtonLeftISR);
	//pio_enable_interrupt(PIOB,PIO_BUTTON_LEFT);
	//NVIC_SetPriority(PIOB_IRQn, BUTTON_PRIORITY);
	//NVIC_EnableIRQ(PIOB_IRQn);
	//
	//pio_set_input(PIOC, PIO_BUTTON_RIGTH, PIO_PULLUP | PIO_DEBOUNCE);
	//pio_handler_set(PIOC,ID_PIOC,PIO_BUTTON_RIGTH,PIO_IT_RISE_EDGE,vButtonRightISR);
	//pio_enable_interrupt(PIOC,PIO_BUTTON_RIGTH);
	//NVIC_SetPriority(PIOC_IRQn, BUTTON_PRIORITY);
	//NVIC_EnableIRQ(PIOC_IRQn);
//}
//
//void vConfigurePWM() {
	//pmc_enable_periph_clk(ID_PWM);
	//pwm_channel_disable(PWM, PWM_CHANNEL);
	//pwm_clock_t clock_setting = {
		//.ul_clka = PWM_FREQUENCY * PERIOD_VALUE,
		//.ul_clkb = 0,
		//.ul_mck = sysclk_get_cpu_hz()
	//};
	//pwm_init(PWM, &clock_setting);
	//g_pwm_channel.alignment = PWM_ALIGN_LEFT;
	//g_pwm_channel.polarity = PWM_LOW;
	//g_pwm_channel.ul_prescaler = PWM_CMR_CPRE_CLKA;
	//g_pwm_channel.ul_period = PERIOD_VALUE;
	//g_pwm_channel.ul_duty = INIT_DUTY_VALUE;
	//g_pwm_channel.channel = PWM_CHANNEL;
	//pwm_channel_init(PWM, &g_pwm_channel);
	//pwm_channel_disable_interrupt(PWM, PWM_CHANNEL, 0);
	//pwm_channel_enable(PWM, PWM_CHANNEL);
//}
//
///**
 //* \brief Application entry point for PWM with LED example.
 //* Output PWM waves on LEDs to make them fade in and out.
 //*
 //* \return Unused (ANSI-C compatibility).
 //*/
//int main(void)
//{
	///* Initialize the SAM system */
	//sysclk_init();
	//board_init();
	//configure_console();
	//
	//vConfigurePWM();
	//vConfigureISR();
//
	///* Infinite loop */
	//while (1) {
	//}
//}