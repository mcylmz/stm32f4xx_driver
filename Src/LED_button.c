/*
 * LED_button.c
 *
 *  Created on: 16 Oca 2022
 *      Author: murat.yilmaz
 */
#include "stm32f407xx.h"

#define BUTTON_PRESSED 1

void delay(void)
{
	for (uint32_t i = 0; i < 250000; i++);
}

int main(void)
{
	gpio_handle_t gpio_led;
	gpio_handle_t gpio_button;

	// Pin LED configurations
	gpio_led.p_gpiox = GPIOD;
	gpio_led.gpio_pin_config.gpio_pin_number = GPIO_PIN_NO_12;
	gpio_led.gpio_pin_config.gpio_pin_mode = GPIO_MODE_OUTPUT;
	gpio_led.gpio_pin_config.gpio_pin_speed = GPIO_SPEED_FAST;
	gpio_led.gpio_pin_config.gpio_pin_optype = GPIO_OP_TYPE_PP;
	gpio_led.gpio_pin_config.gpio_pin_pupd_control = GPIO_NO_PUPD;

	// Enable clock
	gpio_pclk_control(GPIOD, ENABLE);

	// Init. GPIO
	gpio_init(&gpio_led);

	// GPIO button configurations
	gpio_button.p_gpiox = GPIOA;
	gpio_button.gpio_pin_config.gpio_pin_number = GPIO_PIN_NO_0;
	gpio_button.gpio_pin_config.gpio_pin_mode = GPIO_MODE_INPUT;
	gpio_button.gpio_pin_config.gpio_pin_speed = GPIO_SPEED_FAST;
	gpio_button.gpio_pin_config.gpio_pin_pupd_control = GPIO_NO_PUPD;

	// Enable clock
	gpio_pclk_control(GPIOA, ENABLE);

	// Init. GPIO
	gpio_init(&gpio_button);


	while (1)
	{
		if (gpio_read_from_input_pin(GPIOA, GPIO_PIN_NO_0) == BUTTON_PRESSED)
		{
			delay();
			gpio_toggle_output_pin(GPIOD, GPIO_PIN_NO_12);
		}
	}

	return 0;
}
