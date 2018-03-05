
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
#include "fsl_port.h"
#include "fsl_gpio.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

SemaphoreHandle_t b_led_semaphore;
SemaphoreHandle_t g_led_semaphore;

typedef struct
{
	SemaphoreHandle_t semaphore;
	GPIO_Type * GPIO_port;
	uint8_t shuffle;
}led_struct_t;

void PORTA_IRQHandler()
{
	BaseType_t xHigherPriorityTaskWoken;
	PORT_ClearPinsInterruptFlags(PORTA, 1<<4);
	xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR( b_led_semaphore, &xHigherPriorityTaskWoken );
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void PORTC_IRQHandler()
{
	BaseType_t xHigherPriorityTaskWoken;
	PORT_ClearPinsInterruptFlags(PORTC, 1<<6);
	xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR( g_led_semaphore, &xHigherPriorityTaskWoken );
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void b_led_task(void *arg)
{
	for(;;)
	{
		xSemaphoreTake(b_led_semaphore,portMAX_DELAY);
		GPIO_TogglePinsOutput(GPIOB,1<<21);
	}
}

void g_led_task(void *arg)
{
	for(;;)
	{
		if (10 == uxSemaphoreGetCount(g_led_semaphore))
		{
			for(uint8_t i = 0; i < 10; i++)
			{
				xSemaphoreTake(g_led_semaphore,portMAX_DELAY);
			}
			GPIO_TogglePinsOutput(GPIOE,1<<26);
		}
	}
}


int main(void)
{

	/* Init board hardware. */
	BOARD_InitBootPins();
	BOARD_InitBootClocks();
	BOARD_InitBootPeripherals();
	/* Init FSL debug console. */
	BOARD_InitDebugConsole();

	CLOCK_EnableClock(kCLOCK_PortA);
	CLOCK_EnableClock(kCLOCK_PortB);
	CLOCK_EnableClock(kCLOCK_PortC);
	CLOCK_EnableClock(kCLOCK_PortE);


	port_pin_config_t config_led =
	{ kPORT_PullDisable, kPORT_SlowSlewRate, kPORT_PassiveFilterDisable,
			kPORT_OpenDrainDisable, kPORT_LowDriveStrength, kPORT_MuxAsGpio,
			kPORT_UnlockRegister, };

	PORT_SetPinConfig(PORTB, 21, &config_led);
	PORT_SetPinConfig(PORTE, 26, &config_led);

	port_pin_config_t config_switch =
	{ kPORT_PullDisable, kPORT_SlowSlewRate, kPORT_PassiveFilterDisable,
			kPORT_OpenDrainDisable, kPORT_LowDriveStrength, kPORT_MuxAsGpio,
			kPORT_UnlockRegister};

	PORT_SetPinInterruptConfig(PORTA, 4, kPORT_InterruptFallingEdge);
	PORT_SetPinInterruptConfig(PORTC, 6, kPORT_InterruptFallingEdge);

	PORT_SetPinConfig(PORTA, 4, &config_switch);
	PORT_SetPinConfig(PORTC, 6, &config_switch);

	gpio_pin_config_t led_config_gpio =
	{ kGPIO_DigitalOutput, 1 };

	GPIO_PinInit(GPIOB, 21, &led_config_gpio);
	GPIO_PinInit(GPIOE, 26, &led_config_gpio);

	gpio_pin_config_t switch_config_gpio =
	{ kGPIO_DigitalInput, 1 };

	GPIO_PinInit(GPIOA, 4, &switch_config_gpio);
	GPIO_PinInit(GPIOC, 6, &switch_config_gpio);

	NVIC_EnableIRQ(PORTA_IRQn);
	NVIC_SetPriority(PORTA_IRQn,5);
	NVIC_EnableIRQ(PORTC_IRQn);
	NVIC_SetPriority(PORTC_IRQn,5);

	GPIO_WritePinOutput(GPIOB,21,0);
	b_led_semaphore = xSemaphoreCreateBinary();
	g_led_semaphore = xSemaphoreCreateCounting(10,0);

	xTaskCreate(b_led_task, "BLED task", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES-1, NULL);
	xTaskCreate(g_led_task, "GLED task", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES-2, NULL);
	vTaskStartScheduler();
	while (1)
	{

	}
	return 0;
}
