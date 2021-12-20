/*
 * stm32f407xx.h
 *
 *  Created on: Dec 14, 2021
 *      Author: murat.yilmaz
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

// Device memory base addresses (Flash and SRAM)
#define FLASH_BASE_ADDR		0x08000000U
#define SRAM1_BASE_ADDR		0x20000000U
#define SRAM2_BASE_ADDR		0x2001C000U
#define ROM_BASE_ADDR		0x1FFF0000U

// Device bus base addresses
#define APB1_BASE_ADDR		0x40000000
#define APB2_BASE_ADDR		0x40010000
#define AHB1_BASE_ADDR		0x40020000

/*******************************************************************************
 * 						Peripheral base addresses							   *
 ******************************************************************************/

// RCC peripheral base address
#define RCC_BASE_ADDR		(AHB1_BASE_ADDR + 0x3800)

// GPIO peripheral base addresses
#define GPIOA_BASE_ADDR		(AHB1_BASE_ADDR + 0x0000)
#define GPIOB_BASE_ADDR		(AHB1_BASE_ADDR + 0x0400)
#define GPIOC_BASE_ADDR		(AHB1_BASE_ADDR + 0x0800)
#define GPIOD_BASE_ADDR		(AHB1_BASE_ADDR + 0x0C00)
#define GPIOE_BASE_ADDR		(AHB1_BASE_ADDR + 0x1000)
#define GPIOF_BASE_ADDR		(AHB1_BASE_ADDR + 0x1400)
#define GPIOG_BASE_ADDR		(AHB1_BASE_ADDR + 0x1800)
#define GPIOH_BASE_ADDR		(AHB1_BASE_ADDR + 0x1C00)
#define GPIOI_BASE_ADDR		(AHB1_BASE_ADDR + 0x2000)

/*******************************************************************************
 * 						Peripheral register definitions						   *
 ******************************************************************************/

// RCC register definition structure
typedef struct
{
	volatile uint32_t CR;
	volatile uint32_t PLLCFGR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t AHB1RSTR;
	volatile uint32_t AHB2RSTR;
	volatile uint32_t AHB3RSTR;
	volatile uint32_t RESERVED1;
	volatile uint32_t APB1RSTR;
	volatile uint32_t APB2RSTR;
	volatile uint32_t RESERVED2[2];
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	volatile uint32_t AHB3ENR;
	volatile uint32_t RESERVED3;
	volatile uint32_t APB1ENR;
	volatile uint32_t APB2ENR;
	volatile uint32_t RESERVED4[2];
	volatile uint32_t AHB1LPENR;
	volatile uint32_t AHB2LPENR;
	volatile uint32_t AHB3LPENR;
	volatile uint32_t RESERVED5;
	volatile uint32_t APB1LPENR;
	volatile uint32_t APB2LPENR;
	volatile uint32_t RESERVED6[2];
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	volatile uint32_t RESERVED[2];
	volatile uint32_t SSCGR;
	volatile uint32_t PLLI2SCFGR;
	volatile uint32_t PLLSAICFGR;
	volatile uint32_t DCKCFGR;

} rcc_regdef_t;

// GPIO register definition structure
typedef struct
{
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSSR;
	volatile uint32_t LCKR;
	volatile uint32_t AFR[2]; // Alternate function low and high registers
} gpio_regdef_t;

/*******************************************************************************
 * 						Peripheral definitions						           *
 ******************************************************************************/

// RCC
#define RCC		((rcc_regdef_t *)RCC_BASE_ADDR)

// GPIO peripherals
#define GPIOA		((gpio_regdef_t *)GPIOA_BASE_ADDR)
#define GPIOB		((gpio_regdef_t *)GPIOB_BASE_ADDR)
#define GPIOC		((gpio_regdef_t *)GPIOC_BASE_ADDR)
#define GPIOD		((gpio_regdef_t *)GPIOD_BASE_ADDR)
#define GPIOE		((gpio_regdef_t *)GPIOE_BASE_ADDR)
#define GPIOF		((gpio_regdef_t *)GPIOF_BASE_ADDR)
#define GPIOG		((gpio_regdef_t *)GPIOG_BASE_ADDR)
#define GPIOH		((gpio_regdef_t *)GPIOH_BASE_ADDR)
#define GPIOI		((gpio_regdef_t *)GPIOI_BASE_ADDR)

/*******************************************************************************
 * 						Clock enable and disable macros						   *
 ******************************************************************************/

// GPIO clock enable
#define GPIOA_PCLK_ENABLE()			(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_ENABLE()			(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_ENABLE()			(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_ENABLE()			(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_ENABLE()			(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_ENABLE()			(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_ENABLE()			(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_ENABLE()			(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_ENABLE()			(RCC->AHB1ENR |= (1 << 8))

// GPIO clock disable
#define GPIOA_PCLK_DISABLE()		(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DISABLE()		(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DISABLE()		(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DISABLE()		(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DISABLE()		(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DISABLE()		(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DISABLE()		(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DISABLE()		(RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DISABLE()		(RCC->AHB1ENR &= ~(1 << 8))

/*******************************************************************************
 * 						MACROS TO RESET PERIPHERALS							   *
 ******************************************************************************/

// Macros to reset GPIOx peripherals.
#define GPIOA_REG_RESET()			do { (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); } while(0)
#define GPIOB_REG_RESET()			do { (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); } while(0)
#define GPIOC_REG_RESET()			do { (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); } while(0)
#define GPIOD_REG_RESET()			do { (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); } while(0)
#define GPIOE_REG_RESET()			do { (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); } while(0)
#define GPIOF_REG_RESET()			do { (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); } while(0)
#define GPIOG_REG_RESET()			do { (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); } while(0)
#define GPIOH_REG_RESET()			do { (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); } while(0)
#define GPIOI_REG_RESET()			do { (RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8)); } while(0)

/*******************************************************************************
 * 						Generic macros						   				   *
 ******************************************************************************/
#define ENABLE		1
#define DISABLE		0
#define	SET			ENABLE
#define RESET		DISABLE

#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET

#endif /* INC_STM32F407XX_H_ */
