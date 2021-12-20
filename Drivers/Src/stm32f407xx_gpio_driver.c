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
	if (enable == ENABLE)
	{
		// Enable specified GPIO port's clock.
		if (p_gpiox == GPIOA)
		{
			GPIOA_PCLK_ENABLE();
		}
		else if (p_gpiox == GPIOB)
		{
			GPIOB_PCLK_ENABLE();
		}
		else if (p_gpiox == GPIOC)
		{
			GPIOC_PCLK_ENABLE();
		}
		else if (p_gpiox == GPIOD)
		{
			GPIOD_PCLK_ENABLE();
		}
		else if (p_gpiox == GPIOE)
		{
			GPIOE_PCLK_ENABLE();
		}
		else if (p_gpiox == GPIOF)
		{
			GPIOF_PCLK_ENABLE();
		}
		else if (p_gpiox == GPIOG)
		{
			GPIOG_PCLK_ENABLE();
		}
		else if (p_gpiox == GPIOH)
		{
			GPIOH_PCLK_ENABLE();
		}
		else if (p_gpiox == GPIOI)
		{
			GPIOI_PCLK_ENABLE();
		}
	}
	else
	{
		// Disable specified GPIO port's clock.
		if (p_gpiox == GPIOA)
		{
			GPIOA_PCLK_DISABLE();
		}
		else if (p_gpiox == GPIOB)
		{
			GPIOB_PCLK_DISABLE();
		}
		else if (p_gpiox == GPIOC)
		{
			GPIOC_PCLK_DISABLE();
		}
		else if (p_gpiox == GPIOD)
		{
			GPIOD_PCLK_DISABLE();
		}
		else if (p_gpiox == GPIOE)
		{
			GPIOE_PCLK_DISABLE();
		}
		else if (p_gpiox == GPIOF)
		{
			GPIOF_PCLK_DISABLE();
		}
		else if (p_gpiox == GPIOG)
		{
			GPIOG_PCLK_DISABLE();
		}
		else if (p_gpiox == GPIOH)
		{
			GPIOH_PCLK_DISABLE();
		}
		else if (p_gpiox == GPIOI)
		{
			GPIOI_PCLK_DISABLE();
		}
	}
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
	uint32_t temp = 0; // Temporary register

	// Configure the mode of GPIO pin.
	if (p_gpio_handle->gpio_pin_config.gpio_pin_mode <= GPIO_MODE_ANALOG)
	{
		// Non-interrupt mode
		temp = p_gpio_handle->gpio_pin_config.gpio_pin_mode
				<< (2 * p_gpio_handle->gpio_pin_config.gpio_pin_number);

		// Clear actual register and store the value into the actual register.
		p_gpio_handle->p_gpiox->MODER &= ~(0x3 <<
				p_gpio_handle->gpio_pin_config.gpio_pin_number); // Clearing.
		p_gpio_handle->p_gpiox->MODER |= temp; // Setting.
	}
	else
	{
		// Interrupt mode (wil be coded later.)
	}

	// Configure the speed.
	temp = 0;
	temp = p_gpio_handle->gpio_pin_config.gpio_pin_speed
			<< (2 * p_gpio_handle->gpio_pin_config.gpio_pin_number);

	p_gpio_handle->p_gpiox->OSPEEDR &= ~(0x3 <<
			p_gpio_handle->gpio_pin_config.gpio_pin_number); // Clearing.
	p_gpio_handle->p_gpiox->OSPEEDR |= temp; // Setting.

	// Configure the pull up - pull down settings.
	temp = 0;
	temp = p_gpio_handle->gpio_pin_config.gpio_pin_pupd_control
			<< (2 * p_gpio_handle->gpio_pin_config.gpio_pin_number);

	p_gpio_handle->p_gpiox->PUPDR &= ~(0x3 <<
			p_gpio_handle->gpio_pin_config.gpio_pin_number); // Clearing.
	p_gpio_handle->p_gpiox->PUPDR |= temp; // Setting.

	// Configure the output type.
	temp = 0;
	temp = p_gpio_handle->gpio_pin_config.gpio_pin_optype
			<< p_gpio_handle->gpio_pin_config.gpio_pin_number;

	p_gpio_handle->p_gpiox->OTYPER &= ~(0x1 <<
			p_gpio_handle->gpio_pin_config.gpio_pin_number); // Clearing.
	p_gpio_handle->p_gpiox->OTYPER |= temp; // Setting.

	// Configure the alternate functionality
	temp = 0;

	if (p_gpio_handle->gpio_pin_config.gpio_pin_mode == GPIO_MODE_ALT_FUNC)
	{
		// Configure the alternate function registers.
		uint32_t low_or_high_reg; // 0 : low register - 1 : high register
		uint32_t location;

		low_or_high_reg = p_gpio_handle->gpio_pin_config.gpio_pin_number / 8;
		location = p_gpio_handle->gpio_pin_config.gpio_pin_number % 8;

		p_gpio_handle->p_gpiox->AFR[low_or_high_reg] &=
				~(0xF << (4 * location)); // Clearing.
		p_gpio_handle->p_gpiox->AFR[low_or_high_reg] |=
				p_gpio_handle->gpio_pin_config.gpio_pin_alt_func_mode << (4 * location); // Setting.
	}
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
void gpio_uninit(gpio_regdef_t *p_gpiox)
{
	if (p_gpiox == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if (p_gpiox == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if (p_gpiox == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if (p_gpiox == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if (p_gpiox == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if (p_gpiox == GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if (p_gpiox == GPIOG)
	{
		GPIOG_REG_RESET();
	}
	else if (p_gpiox == GPIOH)
	{
		GPIOH_REG_RESET();
	}
	else if (p_gpiox == GPIOI)
	{
		GPIOI_REG_RESET();
	}
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
