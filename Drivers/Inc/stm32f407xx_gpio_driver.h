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

// GPIO pin numbers.
#define GPIO_PIN_NO_0			0
#define GPIO_PIN_NO_1			1
#define GPIO_PIN_NO_2			2
#define GPIO_PIN_NO_3			3
#define GPIO_PIN_NO_4			4
#define GPIO_PIN_NO_5			5
#define GPIO_PIN_NO_6			6
#define GPIO_PIN_NO_7			7
#define GPIO_PIN_NO_8			8
#define GPIO_PIN_NO_9			9
#define GPIO_PIN_NO_10			10
#define GPIO_PIN_NO_11			11
#define GPIO_PIN_NO_12			12
#define GPIO_PIN_NO_13			13
#define GPIO_PIN_NO_14			14
#define GPIO_PIN_NO_15			15

// GPIO pins possible modes.
#define GPIO_MODE_INPUT			0
#define GPIO_MODE_OUTPUT		1
#define GPIO_MODE_ALT_FUNC		2
#define GPIO_MODE_ANALOG		3
#define GPIO_MODE_IT_FT			4
#define GPIO_MODE_IT_RT			5
#define GPIO_MODE_IT_RFT		6

// GPIO pin possible output type.
#define GPIO_OP_TYPE_PP			0
#define GPIO_OP_TYPE_OD			1

// GPIO pin possible output speeds.
#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_FAST			2
#define GPIO_SPEED_HIGH			3

// GPIO pin pull up - pull down configuration macros.
#define GPIO_NO_PUPD			0
#define GPIO_PU					1
#define GPIO_PD					2

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
void gpio_irq_interrupt_config(uint8_t irq_number, uint8_t enable);
void gpio_irq_priority_config(uint8_t irq_number, uint8_t irq_priority);
void gpio_irq_handling(uint8_t pin_number);

#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
