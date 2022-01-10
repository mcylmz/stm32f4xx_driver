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
		// Interrupt mode
		if (p_gpio_handle->gpio_pin_config.gpio_pin_mode == GPIO_MODE_IT_FT)
		{
			// Configure the FTSR(Falling Trigger Selection Register).
			EXTI->FTSR |= 1 << p_gpio_handle->gpio_pin_config.gpio_pin_number;

			// Clear the corresponding RTSR bit.
			EXTI->RTSR &= ~(1 << p_gpio_handle->gpio_pin_config.gpio_pin_number);
		}
		else if (p_gpio_handle->gpio_pin_config.gpio_pin_mode == GPIO_MODE_IT_RT)
		{
			// Configure the RTSR(Rising Trigger Selection Register).
			EXTI->RTSR |= 1 << p_gpio_handle->gpio_pin_config.gpio_pin_number;

			// Clear the corresponding FTSR bit.
			EXTI->FTSR &= ~(1 << p_gpio_handle->gpio_pin_config.gpio_pin_number);
		}
		else if (p_gpio_handle->gpio_pin_config.gpio_pin_mode == GPIO_MODE_IT_RFT)
		{
			// Configure both FTSR and RTSR.
			EXTI->FTSR |= 1 << p_gpio_handle->gpio_pin_config.gpio_pin_number;
			EXTI->RTSR |= 1 << p_gpio_handle->gpio_pin_config.gpio_pin_number;
		}

		// Configure the GPIO port selection in SYSCFG_EXTICR.
		uint8_t reg_index = p_gpio_handle->gpio_pin_config.gpio_pin_number / 4;
		uint8_t loc = p_gpio_handle->gpio_pin_config.gpio_pin_number % 4;

		SYSCFG_PCLK_ENABLE();
		uint8_t port_code = GPIO_BASE_ADDR_TO_PORT_CODE(p_gpio_handle);
		SYSCFG->EXTICR[reg_index] = port_code << (loc * 4);


		// Enable the EXTI interrupt delivery using IMR(Interrupt Mask Register).
		EXTI->IMR |= 1 << p_gpio_handle->gpio_pin_config.gpio_pin_mode;
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
 * @return		- Read value. - 0 or 1.
 *
 * @Note		- none
 ******************************************************************************/
uint8_t gpio_read_from_input_pin(gpio_regdef_t *p_gpiox, uint8_t pin_number)
{
	// Shift the read value to least significant bit location and read from that
	// location.
	uint8_t value;
	value = (uint8_t)((p_gpiox->IDR >> pin_number) & (0x00000001));

	return value;
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
	uint16_t value;
	value = (uint16_t)p_gpiox->IDR;

	return value;
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
	if (value == GPIO_PIN_SET)
	{
		// Write 1 to the output data register of specified pin number.
		p_gpiox->ODR |= (1 << pin_number);
	}
	else
	{
		// Write 0 to the output data register of specified pin number.
		p_gpiox->ODR &= ~(1 << pin_number);
	}
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
	p_gpiox->ODR = value;
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
	p_gpiox->ODR ^= (1 << pin_number);
}

/*******************************************************************************
 * @fn			- gpio_irq_interrupt_config
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
void gpio_irq_interrupt_config(uint8_t irq_number, uint8_t enable)
{
	if (enable == ENABLE)
	{
		if (irq_number <= 31)
		{
			// Configure ISER0 register.
			*NVIC_ISER0 |= 1 << irq_number;

		}
		else if (irq_number > 31 && irq_number < 64)
		{
			// Configure ISER1 register.
			*NVIC_ISER1 |= 1 << (irq_number % 32);

		}
		else if (irq_number >= 64 && irq_number < 96)
		{
			// Configure ISER2 register.
			*NVIC_ISER2 |= 1 << (irq_number % 64);
		}
	}
	else
	{
		if (irq_number <= 31)
		{
			// Configure ICER0 register.
			*NVIC_ICER0 |= 1 << irq_number;
		}
		else if (irq_number > 31 && irq_number < 64)
		{
			// Configure ICER1 register.
			*NVIC_ICER1 |= 1 << (irq_number % 32);
		}
		else if (irq_number >= 64 && irq_number < 96)
		{
			// Configure ICER2 register.
			*NVIC_ICER2 |= 1 << (irq_number % 64);
		}
	}
}

/*******************************************************************************
 * @fn			- gpio_irq_priority_config
 *
 * @brief		- Interrupt priority configuration function.
 *
 * @param[in]	- Interrupt priority number.
 *
 * @return		- none
 *
 * @Note		- none
 ******************************************************************************/
void gpio_irq_priority_config(uint8_t irq_number, uint8_t irq_priority)
{
	uint8_t iprx = irq_number / 4;
	uint8_t iprx_section = irq_number % 4;

	uint8_t shift_amount = (iprx_section * 8) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_IPR_BASE_ADDR + (iprx * 4)) |= irq_number << shift_amount;
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
	// Clear the EXTI PR register corresponding to the pin number.
	if (EXTI->PR & (1 << pin_number))
	{
		EXTI->PR |= (1 << pin_number);
	}
}
