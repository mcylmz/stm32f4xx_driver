/*
 * button_interrupt.c
 *
 *  Created on: 16 Oca 2022
 *      Author: murat.yilmaz
 */
#include "stm32f407xx.h"

/*
 * TODO Fix the interrupt bug.
 */

int main()
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

	// Initialize GPIO
	gpio_init(&gpio_led);

	// GPIO button configurations
	gpio_button.p_gpiox = GPIOA;
	gpio_button.gpio_pin_config.gpio_pin_number = GPIO_PIN_NO_0;
	gpio_button.gpio_pin_config.gpio_pin_mode = GPIO_MODE_IT_FT;
	gpio_button.gpio_pin_config.gpio_pin_speed = GPIO_SPEED_FAST;
	gpio_button.gpio_pin_config.gpio_pin_pupd_control = GPIO_NO_PUPD;

	// Enable clock
	gpio_pclk_control(GPIOA, ENABLE);

	// Initialize GPIO
	gpio_init(&gpio_button);

	// IRQ configurations
	gpio_irq_priority_config(IRQ_NO_EXTI0, NVIC_IRQ_PRI15);
	gpio_irq_interrupt_config(IRQ_NO_EXTI0, ENABLE);

	while (1);

	return 0;
}

void EXTI0_IRQHandler(void)
{
	gpio_irq_handling(GPIO_PIN_NO_0);
	gpio_toggle_output_pin(GPIOD, GPIO_PIN_NO_12);
}
