/*
 * LED_toggle.c
 *
 *  Created on: 16 Oca 2022
 *      Author: murat.yilmaz
 */
#include "stm32f407xx.h"

void delay(void)
{
	for (uint32_t i = 0; i < 500000; i++);
}

int main(void)
{
	gpio_handle_t gpio_led;

	// Pin configurations
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

	while (1)
	{
		gpio_toggle_output_pin(GPIOD, GPIO_PIN_NO_12);
		delay();
	}

	return 0;
}
