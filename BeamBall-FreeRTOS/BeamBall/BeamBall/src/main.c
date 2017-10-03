/**
 * \file
 *
 * \brief FreeRTOS Real Time Kernel example.
 *
 * Copyright (c) 2012-2016 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

/**
 * \mainpage FreeRTOS Real Time Kernel example
 *
 * \section Purpose
 *
 * The FreeRTOS example will help users how to use FreeRTOS in SAM boards.
 * This basic application shows how to create task and get information of
 * created task.
 *
 * \section Requirements
 *
 * This package can be used with SAM boards.
 *
 * \section Description
 *
 * The demonstration program create two task, one is make LED on the board
 * blink at a fixed rate, and another is monitor status of task.
 *
 * \section Usage
 *
 * -# Build the program and download it inside the evaluation board. Please
 *    refer to the
 *    <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6224.pdf">
 *    SAM-BA User Guide</a>, the
 *    <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6310.pdf">
 *    GNU-Based Software Development</a>
 *    application note or to the
 *    <a href="ftp://ftp.iar.se/WWWfiles/arm/Guides/EWARM_UserGuide.ENU.pdf">
 *    IAR EWARM User Guide</a>,
 *    depending on your chosen solution.
 * -# On the computer, open and configure a terminal application
 *    (e.g. HyperTerminal on Microsoft Windows) with these settings:
 *   - 115200 bauds
 *   - 8 bits of data
 *   - No parity
 *   - 1 stop bit
 *   - No flow control
 * -# Start the application.
 * -# LED should start blinking on the board. In the terminal window, the
 *    following text should appear (values depend on the board and chip used):
 *    \code
	-- Freertos Example xxx --
	-- xxxxxx-xx
	-- Compiled: xxx xx xxxx xx:xx:xx --
\endcode
 *
 */

#include <asf.h>
#include "conf_board.h"

#define TASK_MONITOR_STACK_SIZE            (2048/sizeof(portSTACK_TYPE))
#define TASK_MONITOR_STACK_PRIORITY        (tskIDLE_PRIORITY)
#define TASK_LED_STACK_SIZE                (1024/sizeof(portSTACK_TYPE))
#define TASK_LED_STACK_PRIORITY            (tskIDLE_PRIORITY)

#define TASK_STACK_SIZE						(1024/sizeof(portSTACK_TYPE))
#define TASK_SENSOR_PRIORITY				(tskIDLE_PRIORITY)
#define TASK_MOTOR_PRIORITY					(tskIDLE_PRIORITY)
#define TASK_CONTROL_PRIORITY				(tskIDLE_PRIORITY)

#define PWM_FREQUENCY      20000
#define PERIOD_VALUE       100
#define INIT_DUTY_VALUE    0

pwm_channel_t g_pwm_channel_led;
uint32_t sensor_counter = 0;

xTaskHandle *pTaskSensor = NULL;
xTaskHandle *pTaskControle = NULL;
xTaskHandle *pTaskMotor = NULL;


extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
		signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);


#if !(SAMV71 || SAME70)
/**
 * \brief Handler for System Tick interrupt.
 */
void SysTick_Handler(void)
{
	xPortSysTickHandler();
}
#endif

/**
 * \brief Called if stack overflow during execution
 */
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
		signed char *pcTaskName)
{
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	/* If the parameters have been corrupted then inspect pxCurrentTCB to
	 * identify which task has overflowed its stack.
	 */
	for (;;) {
	}
}

/**
 * \brief This function is called by FreeRTOS idle task
 */
extern void vApplicationIdleHook(void)
{
}

/**
 * \brief This function is called by FreeRTOS each tick
 */
extern void vApplicationTickHook(void)
{
	// count the time ellapsed since last sensor trigger
	//sensor_counter++;
}

extern void vApplicationMallocFailedHook(void)
{
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

	/* Force an assert. */
	configASSERT( ( volatile void * ) NULL );
}


/**
 * \brief Configure the console UART.
 */
static void vConfigureUart(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
#if (defined CONF_UART_CHAR_LENGTH)
		.charlength = CONF_UART_CHAR_LENGTH,
#endif
		.paritytype = CONF_UART_PARITY,
#if (defined CONF_UART_STOP_BITS)
		.stopbits = CONF_UART_STOP_BITS,
#endif
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
#if defined(__GNUC__)
	setbuf(stdout, NULL);
#else
	/* Already the case in IAR's Normal DLIB default configuration: printf()
	 * emits one character at a time.
	 */
#endif
}

/**
 * \brief This task, when activated, send every ten seconds on debug UART
 * the whole report of free heap and total tasks status
 */
static void task_monitor(void *pvParameters)
{
	static portCHAR szList[256];
	UNUSED(pvParameters);

	for (;;) {
		printf("--- task ## %u", (unsigned int)uxTaskGetNumberOfTasks());
		vTaskList((signed portCHAR *)szList);
		printf(szList);
		vTaskDelay(1000);
	}
}

/**
 * \brief This task, when activated, make LED blink at a fixed rate
 */
static void task_led(void *pvParameters)
{
	UNUSED(pvParameters);
	for (;;) {
	#if SAM4CM
		LED_Toggle(LED4);
	#else
		LED_Toggle(LED0);
	#endif
		vTaskDelay(1000);
	}
}

/**
 * \brief This task, when activated, will read the ultrassonic sensor data
 */
static void vTaskReadSensor(void *pvParameters)
{
	UNUSED(pvParameters);
	for (;;) {
		sensor_counter = 0;
		gpio_pin_is_high(PIO_PA16);
		// vTaskDelay(1); // implement taskDelay as a for loop
		gpio_pin_is_low(PIO_PA16);
		
		// read from queue ISR

		// use semaphore to read sensor_counter
		// distanceCM = queueISRCounter/58;
		
		vTaskDelay(1);
		// vtaskSuspend() // suspend task until 'malha de controle' calls for a new read
	}
}

static void vConfigureSensorISR() {
	// configure sensor interrupt
}

static void vSensorISR() {
	// write in queue sensorISR
}

static void vConfigurePWM() {
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

/**
 * \brief This task, when activated, will change the position of servo motor
 */
static void vTaskRunMotor(void *pvParameters)
{
	UNUSED(pvParameters);
	for (;;) {

		// read from queue Malha de Controle MotorPos

		//vPWMUpdateDuty(motorPos); // execute motor
	}
}

// create timer with 10us (timer Handler)
// timer handler counts the time from the sensor start up


/**
 * \brief This task, when activated, will calculate the new position of the motos based on the sensor data
 */
static void vTaskMalhaControle(void *pvParameters)
{
	UNUSED(pvParameters);
	for (;;) {
		// read from sensor queue distanceCM

		// do some control shit!

		// writes to queue Motor Position
	}
}

/**
 *  \brief FreeRTOS Real Time Kernel example entry point.
 *
 *  \return Unused (ANSI-C compatibility).
 */
int main(void)
{
	/* Initialize the SAM system */
	sysclk_init();
	board_init();
	vConfigureUart();

	/* Output demo information. */
	printf("-- Freertos Example --\n\r");
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);


	/* Create task to monitor processor activity */
	if (xTaskCreate(task_monitor, "Monitor", TASK_MONITOR_STACK_SIZE, NULL,
			TASK_MONITOR_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create Monitor task\r\n");
	}

	/* Create task to make led blink */
	if (xTaskCreate(task_led, "Led", TASK_LED_STACK_SIZE, NULL,
			TASK_LED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test led task\r\n");
	}
	
	/* Create task to read sensor */
	if (xTaskCreate(vTaskReadSensor, "Read Sensor", TASK_STACK_SIZE, NULL,
			TASK_SENSOR_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create Read Sensor task\r\n");
	}
	
	/* Create task to change motor position */
	if (xTaskCreate(vTaskRunMotor, "Run Motor", TASK_STACK_SIZE, NULL,
			TASK_MOTOR_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create Run Motor task\r\n");
	}
	
	/* Create task to calculate the control */
	if (xTaskCreate(vTaskMalhaControle, "Malha de Controle", TASK_STACK_SIZE, NULL,
			TASK_CONTROL_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create Malha de Controle task\r\n");
	}

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}


