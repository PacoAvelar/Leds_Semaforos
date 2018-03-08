/*
 * Copyright (c) 2017, NXP Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file    Semaforos_LEDs_Tarea.c
 * @brief   Application entry point.
 */
#include <stdio.h>
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

#define GREEN_LED_MAX_COUNTING 10
#define GREEN_LED_START_COUNTING 0



#define LED_RED_PIN 22
#define LED_BLUE_PIN 21
#define LED_GREEN_PIN 26
#define LED_BLUE_PORT PORTB
#define LED_RED_PORT PORTB
#define LED_GREEN_PORT PORTE
#define LED_BLUE_GPIO GPIOB
#define LED_GREEN_GPIO GPIOE
#define LED_RED_CLOCK_PORT kCLOCK_PortB
#define LED_BLUE_CLOCK_PORT kCLOCK_PortB
#define LED_GREEN_CLOCK_PORT  kCLOCK_PortE

#define SW2_PIN 4
#define SW3_PIN 6
#define SW2_PORT PORTA
#define SW3_PORT PORTC
#define SW2_CLOCK_PORT kCLOCK_PortA
#define SW3_CLOCK_PORT kCLOCK_PortC
#define SW2_INTERRUPT_FLAG_PIN (1 << SW2_PIN)
#define SW3_INTERRUPT_FLAG_PIN (1 << SW3_PIN)


SemaphoreHandle_t BLUE_semaphore;
SemaphoreHandle_t GREEN_semaphore;


void PORTA_IRQHandler()
{


	BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
    GPIO_ClearPinsInterruptFlags(GPIOA, SW2_INTERRUPT_FLAG_PIN);
    xSemaphoreGiveFromISR(GREEN_semaphore, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );


}

void PORTC_IRQHandler()
{

	BaseType_t xHigherPriorityTaskWoken;
    GPIO_ClearPinsInterruptFlags(GPIOC, SW3_INTERRUPT_FLAG_PIN);
	xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(BLUE_semaphore, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );


}

void blue_led (void* args){

	for(;;){
		xSemaphoreTake(BLUE_semaphore,portMAX_DELAY);
		GPIO_TogglePinsOutput(LED_BLUE_GPIO, (1 <<LED_BLUE_PIN));
	}
}

void green_led (void* args){
	uint8_t cuenta = 0;
	for(;;){
	cuenta = uxSemaphoreGetCount(GREEN_semaphore);

	if(GREEN_LED_MAX_COUNTING == cuenta){
		GREEN_semaphore = xSemaphoreCreateCounting(GREEN_LED_MAX_COUNTING, GREEN_LED_START_COUNTING);

		GPIO_TogglePinsOutput(LED_GREEN_GPIO, (1 <<LED_GREEN_PIN));
		}

	}


}

int main(void) {

  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
  	/* Init FSL debug console. */
    BOARD_InitDebugConsole();

/*
 *
 *
 */


    /*
     * Se habilitan los relojes de cada uno de los periféricos.
     */
    CLOCK_EnableClock(LED_BLUE_CLOCK_PORT);
    CLOCK_EnableClock(LED_GREEN_CLOCK_PORT);

    /* Output pin PORT configuration*/
    port_pin_config_t config_blue_led = { kPORT_PullDisable, kPORT_FastSlewRate,
            kPORT_PassiveFilterDisable, kPORT_OpenDrainDisable,
            kPORT_LowDriveStrength, kPORT_MuxAsGpio, kPORT_UnlockRegister, };

    port_pin_config_t config_green_led = { kPORT_PullDisable,
            kPORT_FastSlewRate, kPORT_PassiveFilterDisable,
            kPORT_OpenDrainDisable, kPORT_LowDriveStrength, kPORT_MuxAsGpio,
            kPORT_UnlockRegister, };

    /* Sets the configuration*/
    PORT_SetPinConfig(LED_BLUE_PORT, LED_BLUE_PIN, &config_blue_led);
    PORT_SetPinConfig(LED_GREEN_PORT, LED_GREEN_PIN, &config_green_led);

	gpio_pin_config_t led_config = { kGPIO_DigitalOutput, 1, };
	/* Sets the configuration
	 */
	GPIO_PinInit(GPIOB, LED_BLUE_PIN, &led_config);
	GPIO_PinInit(GPIOE, LED_GREEN_PIN, &led_config);
    /*
     * Configuración de los botones.
     */

    CLOCK_EnableClock(SW2_CLOCK_PORT);
    CLOCK_EnableClock(SW3_CLOCK_PORT);

    port_pin_config_t config_SW3 = { kPORT_PullDisable, kPORT_FastSlewRate,
            kPORT_PassiveFilterDisable, kPORT_OpenDrainDisable,
            kPORT_LowDriveStrength, kPORT_MuxAsGpio, kPORT_UnlockRegister, };
    port_pin_config_t config_SW2 = { kPORT_PullDisable, kPORT_FastSlewRate,
            kPORT_PassiveFilterDisable, kPORT_OpenDrainDisable,
            kPORT_LowDriveStrength, kPORT_MuxAsGpio, kPORT_UnlockRegister, };

    PORT_SetPinConfig(SW2_PORT, SW2_PIN, &config_SW2);
    PORT_SetPinConfig(SW3_PORT, SW3_PIN, &config_SW3);


	gpio_pin_config_t switch_config = { kGPIO_DigitalInput, 1, };
	GPIO_PinInit(GPIOA, SW2_PIN, &switch_config);
	GPIO_PinInit(GPIOC,SW3_PIN, &switch_config);

	PORT_SetPinInterruptConfig(PORTA, 4, kPORT_InterruptFallingEdge);
	PORT_SetPinInterruptConfig(PORTC, 6, kPORT_InterruptFallingEdge);



    NVIC_EnableIRQ(PORTA_IRQn);
    NVIC_EnableIRQ(PORTC_IRQn);

	NVIC_SetPriority(PORTA_IRQn,5);

	NVIC_SetPriority(PORTC_IRQn,6);
    /*
     *
     *
     *
     *
     */
    xTaskCreate(blue_led, "LED AZUL", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES -1, NULL);
    xTaskCreate(green_led, "LED VERDE", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES - 2, NULL);

    BLUE_semaphore = xSemaphoreCreateBinary();
    GREEN_semaphore = xSemaphoreCreateCounting(GREEN_LED_MAX_COUNTING, 0);


	GPIO_WritePinOutput(GPIOB,21,0);


	vTaskStartScheduler();
    while(1) {
    }
    return 0 ;
}
