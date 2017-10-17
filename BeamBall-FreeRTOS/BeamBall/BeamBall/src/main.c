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
#include <BeamBall.h>


#define TASK_MONITOR_STACK_SIZE            (2048/sizeof(portSTACK_TYPE))
#define TASK_MONITOR_STACK_PRIORITY        (tskIDLE_PRIORITY)
#define TASK_LED_STACK_SIZE                (1024/sizeof(portSTACK_TYPE))
#define TASK_LED_STACK_PRIORITY            (tskIDLE_PRIORITY)

#define TASK_STACK_SIZE						(1024/sizeof(portSTACK_TYPE))
#define TASK_SENSOR_PRIORITY				(1)
#define TASK_MOTOR_PRIORITY					(1)
#define TASK_CONTROL_PRIORITY				(1)

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
		.paritytype = CONF_UART_PARITY
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);
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


/* Beam Ball Start */

//xTaskHandle *pxTaskSensor = NULL;
//xTaskHandle *pxTaskControle = NULL;
//xTaskHandle *pxTaskMotor = NULL;

xQueueHandle xQueueSensor = NULL;
xQueueHandle xQueueControle = NULL;
xQueueHandle xQueueMotor = NULL;

uint32_t sensor_counter = 0;
void vSensorISR(const uint32_t id, const uint32_t index) {
	// write in queue sensorISR
	// int sensor_counter = timer_value_RC;
	printf("Entrei ISR \r\n");

	sensor_counter += 58;
	//sensor_counter = tc_read_cv(TC,CHANNEL);

	//char buffer[50];
	//ili9225_set_foreground_color(COLOR_WHITE);
	//ili9225_draw_filled_rectangle(0, 30, ILI9225_LCD_WIDTH, ILI9225_LCD_HEIGHT);
	//ili9225_set_foreground_color(COLOR_BLUE);
	//ili9225_draw_string(10,35,(uint8_t *) "C: ");
	//int n = sprintf (buffer, "%d", sensor_counter);
	//ili9225_draw_string(95,35,(uint8_t *) buffer);
	
	gpio_toggle_pin(LED1_GPIO);
	if (xQueueSendFromISR(xQueueSensor,&sensor_counter,NULL) != pdPASS)
		printf("Error Sending data to xQueueSensor\r\n");

	printf("Terminei ISR \r\n");
}

void vConfigureSensorISR() {
	// configure sensor interrupt
	printf("Configuracao Sensor ISR \r\n");
	
	// pio no btn BP2 (LEFT) trocar para outro PIO
	
	pio_set_input(PIOA, PIO_ECHO,PIO_PULLUP | PIO_DEBOUNCE); // pull down
	//pio_pull_down(PIOA,PIO_ECHO,1);
	pio_handler_set(PIOA,ID_PIOA,PIO_ECHO,PIO_IT_RISE_EDGE,vSensorISR);
	pio_enable_interrupt(PIOA,PIO_ECHO);
	NVIC_SetPriority(PIOA_IRQn, 1 );
	NVIC_EnableIRQ(PIOA_IRQn);

	printf("Terminei Config Sensor ISR \r\n");
}

static void vTaskReadSensor(void *pvParameters)
{
	UNUSED(pvParameters);
	uint32_t counter = 0;
	double distanceCM = 0;
	char buffer[50];

	for (;;) {
		//sensor_counter = 0;
		// zerar timer tc
		//tc_get_status(TC,CHANNEL);
		gpio_pin_is_high(PIO_TRIGGER);
		delay_us(10);
		gpio_pin_is_low(PIO_TRIGGER);

		// read from queue ISR
		if (xQueueReceive(xQueueSensor,&counter,portMAX_DELAY) != pdPASS)
			printf("Error Receiving data from xQueueSensor\r\n");

		// print no LCD
		ili9225_set_foreground_color(COLOR_WHITE);
		ili9225_draw_filled_rectangle(0, 30, ILI9225_LCD_WIDTH, ILI9225_LCD_HEIGHT);
		ili9225_set_foreground_color(COLOR_BLUE);
		ili9225_draw_string(10,35,(uint8_t *) "C: ");

		int n = sprintf (buffer, "%d", counter);
		ili9225_draw_string(95,35,(uint8_t *) buffer);

		distanceCM = counter/58;

		ili9225_set_foreground_color(COLOR_WHITE);
		ili9225_draw_filled_rectangle(0, 50, ILI9225_LCD_WIDTH, ILI9225_LCD_HEIGHT);
		ili9225_set_foreground_color(COLOR_BLUE);
		ili9225_draw_string(10,55,(uint8_t *) "D: ");

		n = sprintf (buffer, "%.2f", distanceCM);
		ili9225_draw_string(95,55,(uint8_t *) buffer);
		
		if (xQueueSend(xQueueControle,&distanceCM,portMAX_DELAY) != pdPASS)
	 		printf("Error Sending data to xQueueControle\n");
	
		taskYIELD();
	}
}

/* Start Malha de Controle Functions */

void vTaskMalhaControle(void *pvParameters)
{
	UNUSED(pvParameters);
	double sensorDistance = 0;
	double motorPos = 0;
	char buffer[50];
	int n,i;
	i = 0;
	for (;;) {
		// read from sensor queue distanceCM
		if (xQueueReceive(xQueueControle,&sensorDistance,portMAX_DELAY) != pdPASS)
			printf("Error Receiving data from xQueueSensor\n");
		
		ili9225_set_foreground_color(COLOR_WHITE);
		ili9225_draw_filled_rectangle(0, 70, ILI9225_LCD_WIDTH, ILI9225_LCD_HEIGHT);
		ili9225_set_foreground_color(COLOR_BLUE);
		ili9225_draw_string(10,75,(uint8_t *) "SD: ");

		n = sprintf (buffer, "%.2f", sensorDistance);
		ili9225_draw_string(95,75,(uint8_t *) buffer);

		// do some control shit!
		double sd = sensorDistance*10;
		motorPos = (sd < 100) ? sd : sd-10*i;
		i++;

		ili9225_set_foreground_color(COLOR_WHITE);
		ili9225_draw_filled_rectangle(0, 90, ILI9225_LCD_WIDTH, ILI9225_LCD_HEIGHT);
		ili9225_set_foreground_color(COLOR_BLUE);
		ili9225_draw_string(10,95,(uint8_t *) "MP: ");

		n = sprintf (buffer, "%.2f", motorPos);
		ili9225_draw_string(95,95,(uint8_t *) buffer);

		// writes to queue Motor Position
		if (xQueueSend(xQueueMotor,&motorPos,portMAX_DELAY) != pdPASS)
			printf("Error Sending data to xQueueMotor\n");
	}
}

/* End Malha de Controle Functions */

void vTaskRunMotor(void *pvParameters)
{
	UNUSED(pvParameters);
	double pos = 0;
	char buffer[50];

	for (;;) {
		// read from queue Malha de Controle Motor Pos
		if (xQueueReceive(xQueueMotor,&pos,portMAX_DELAY) != pdPASS)
			printf("Error Receiving data from xQueueMotor\n");
		
		ili9225_set_foreground_color(COLOR_WHITE);
		ili9225_draw_filled_rectangle(0, 110, ILI9225_LCD_WIDTH, ILI9225_LCD_HEIGHT);
		ili9225_set_foreground_color(COLOR_BLUE);
		ili9225_draw_string(10,115,(uint8_t *) "P: ");

		int n = sprintf (buffer, "%.2f", pos);
		ili9225_draw_string(95,115,(uint8_t *) buffer);

		vPWMUpdateDuty(pos);
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

	/* Create Queues */
	xQueueSensor = xQueueCreate(5,sizeof (uint32_t));
	xQueueControle = xQueueCreate(5,sizeof (double));
	xQueueMotor = xQueueCreate(5,sizeof(double));

	printf("Queue Done!\r\n");
	
	// ?? - funciona, mas da conflito com o LCD
	vConfigurePWM();
	
	// ?? - funcionando, mas a frequencia é tao rapida q nunca sai desse ponto do codigo
	//vConfigureTimer();
	
	// ?? - Funcionando, mas perde a funcionalidade do LCD
	vConfigureSensorISR();

	vConfigureLCD();
	drawLCD();

	ili9225_set_foreground_color(COLOR_WHITE);
	ili9225_draw_filled_rectangle(0, 30, ILI9225_LCD_WIDTH, ILI9225_LCD_HEIGHT);
	ili9225_set_foreground_color(COLOR_BLUE);
	ili9225_draw_string(10,35,(uint8_t *)"Config Done!");
	
	/* Output demo information. */
	printf("-- FreeRTOS Example --\n\r");
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);
	
	//xPortGetFreeHeapSize();

	/* Create task to monitor processor activity */
	if (xTaskCreate(task_monitor, "Monitor", TASK_MONITOR_STACK_SIZE, NULL,
			TASK_MONITOR_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create Monitor task\r\n");
	}

	/* Create task to make led blink */
	/*if (xTaskCreate(task_led, "Led", TASK_LED_STACK_SIZE, NULL,
			TASK_LED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test led task\r\n");
	}*/
	
	/* Create task to read sensor */
	if (xTaskCreate(vTaskReadSensor, "Read Sensor", TASK_STACK_SIZE * 3, NULL,
			TASK_SENSOR_PRIORITY, /*pxTaskSensor*/ NULL) != pdPASS) {
		printf("Failed to create Read Sensor task\r\n");
	}
	
	/* Create task to calculate the control */
	if (xTaskCreate(vTaskMalhaControle, "Malha de Controle", TASK_STACK_SIZE * 3, NULL,
		TASK_CONTROL_PRIORITY, /*pxTaskControle*/ NULL) != pdPASS) {
		printf("Failed to create Malha de Controle task\r\n");
	}
	
	/* Create task to change motor position */
	if (xTaskCreate(vTaskRunMotor, "Run Motor", TASK_STACK_SIZE * 3, NULL,
		TASK_MOTOR_PRIORITY, /*pxTaskMotor*/ NULL) != pdPASS) {
		printf("Failed to create Run Motor task\r\n");
	}
	
	//xPortGetFreeHeapSize();
	
	ili9225_set_foreground_color(COLOR_WHITE);
	ili9225_draw_filled_rectangle(0, 70, ILI9225_LCD_WIDTH, ILI9225_LCD_HEIGHT);
	ili9225_set_foreground_color(COLOR_BLUE);
	ili9225_draw_string(10,75,(uint8_t *)"Tasks Created!");

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}


