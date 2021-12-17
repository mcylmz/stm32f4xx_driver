/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: 16 Ara 2021
 *      Author: murat.yilmaz
 */

#include "stm32f407xx_gpio_driver.h"

/*******************************************************************************
 * @fn			- gpio_pclk_control
 *
 * @brief		- This function enables or disables peripheral clock for the
 * 				  given GPIO port.
 *
 * @param[in]	- Base address of the GPIO peripheral.
 * @param[in]	- ENABLE or DISABLE macros (defined in stm32f407xx.h)
 *
 * @return		- none
 *
 * @Note		- none
 ******************************************************************************/
void gpio_pclk_control(gpio_regdef_t *p_gpiox, uint8_t enable)
{

}

/*******************************************************************************
 * @fn			- gpio_init
 *
 * @brief		- This function initializes the GPIO.
 *
 * @param[in]	- Pointer to GPIO handle structure.
 *
 * @return		- none
 *
 * @Note		- none
 ******************************************************************************/
void gpio_init(gpio_handle_t *p_gpio_handle)
{

}

/*******************************************************************************
 * @fn			- gpio_uninit
 *
 * @brief		- This function uninitializes the GPIO.
 *
 * @param[in]	- Pointer to GPIO register definition structure.
 *
 * @return		- none
 *
 * @Note		- none
 ******************************************************************************/
void gpio_uninit(gpio_regdef_t *p_gpiox){

}

/*******************************************************************************
 * @fn			- gpio_read_from_input_pin
 *
 * @brief		- This function reads value from specified pin number.
 *
 * @param[in]	- Pointer to register definition structure.
 * @param[in]	- Pin number.
 *
 * @return		- Read value.
 *
 * @Note		- none
 ******************************************************************************/
uint8_t gpio_read_from_input_pin(gpio_regdef_t *p_gpiox, uint8_t pin_number)
{

}

/*******************************************************************************
 * @fn			- gpio_read_from_input_port
 *
 * @brief		- This function reads value from specified GPIO port.
 *
 * @param[in]	- Pointer to register definition structure.
 *
 * @return		- Value read from specific GPIO port.
 *
 * @Note		- none
 ******************************************************************************/
uint16_t gpio_read_from_input_port(gpio_regdef_t *p_gpiox)
{

}

/*******************************************************************************
 * @fn			- gpio_write_to_output_pin
 *
 * @brief		- This function writes specified value to specified pin.
 *
 * @param[in]	- Pointer to register definition structure.
 * @param[in]	- Pin number.
 * @param[in]	- Value to write to pin.
 *
 * @return		- none
 *
 * @Note		- none
 ******************************************************************************/
void gpio_write_to_output_pin(gpio_regdef_t *p_gpiox, uint8_t pin_number, int8_t value)
{

}

/*******************************************************************************
 * @fn			- gpio_write_to_output_port
 *
 * @brief		- This function writes specified value to specified port.
 *
 * @param[in]	- Pointer to register definition structure.
 * @param[in]	- Value to write to port.
 *
 * @return		- none
 *
 * @Note		- none
 ******************************************************************************/
void gpio_write_to_output_port(gpio_regdef_t *p_gpiox, int16_t value)
{

}

/*******************************************************************************
 * @fn			- gpio_toggle_output_pin
 *
 * @brief		- This function toggles the current value of specified pin.
 *
 * @param[in]	- Pointer to register definition structure.
 * @param[in]	- Pin number.
 *
 * @return		- none
 *
 * @Note		- none
 ******************************************************************************/
void gpio_toggle_output_pin(gpio_regdef_t *p_gpiox, uint8_t pin_number)
{

}

/*******************************************************************************
 * @fn			- gpio_irq_config
 *
 * @brief		- This function configures the GPIO IRQ.
 *
 * @param[in]	- Interrupt number.
 * @param[in]	- Interrupt priority number.
 * @param[in]	- ENABLE or DISABLE macros.
 *
 * @return		- none
 *
 * @Note		- none
 ******************************************************************************/
void gpio_irq_config(uint8_t irq_number, uint8_t irq_priority, uint8_t enable)
{

}

/*******************************************************************************
 * @fn			- gpio_irq_handling
 *
 * @brief		- Interrupt handling function.
 *
 * @param[in]	- Pin number.
 *
 * @return		- none
 *
 * @Note		- none
 ******************************************************************************/
void gpio_irq_handling(uint8_t pin_number)
{

}
