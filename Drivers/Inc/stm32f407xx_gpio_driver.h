/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: 16 Ara 2021
 *      Author: murat.yilmaz
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

// Structure to hold pin configuration settings.
typedef struct
{
	uint8_t gpio_pin_number;
	uint8_t gpio_pin_mode;
	uint8_t gpio_pin_speed;
	uint8_t gpio_pin_pupd_control;
	uint8_t gpio_pin_optype;
	uint8_t gpio_pin_alt_func_mode;
} gpio_pin_config_t;

// GPIO pin handle structure.
typedef struct
{
	gpio_regdef_t *p_gpiox; // Base address of the GPIO port.
	gpio_pin_config_t gpio_pin_config; // GPIO pin configurations settings.
} gpio_handle_t;

/*******************************************************************************
 * 						APIs supported by GPIO driver						   *
 ******************************************************************************/

// Peripheral clock setup
void gpio_pclk_control(gpio_regdef_t *p_gpiox, uint8_t enable);

// Initialization and uninitialization
void gpio_init(gpio_handle_t *p_gpio_handle);
void gpio_uninit(gpio_regdef_t *p_gpiox);

// Data read and write
uint8_t gpio_read_from_input_pin(gpio_regdef_t *p_gpiox, uint8_t pin_number);
uint16_t gpio_read_from_input_port(gpio_regdef_t *p_gpiox);
void gpio_write_to_output_pin(gpio_regdef_t *p_gpiox, uint8_t pin_number, int8_t value);
void gpio_write_to_output_port(gpio_regdef_t *p_gpiox, int16_t value);
void gpio_toggle_output_pin(gpio_regdef_t *p_gpiox, uint8_t pin_number);

// 	IRQ configuration and ISR handling
void gpio_irq_config(uint8_t irq_number, uint8_t irq_priority, uint8_t enable);
void gpio_irq_handling(uint8_t pin_number);

#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
