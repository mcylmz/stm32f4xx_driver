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
	gpio_pinconfig_t gpio_pin_config; // GPIO pin configurations settings.
} gpio_handle_t;

#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
