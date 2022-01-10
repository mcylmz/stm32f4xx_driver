/*
 * stm32f407xx.h
 *
 *  Created on: Dec 14, 2021
 *      Author: murat.yilmaz
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

/*******************************************************************************
 * 						Processor specific details							   *
 ******************************************************************************/

// ARM Cortex-Mx processor NVIC ISERx register addresses
#define NVIC_ISER0			(volatile uint32_t *)0xE000E100
#define NVIC_ISER1			(volatile uint32_t *)0xE000E104
#define NVIC_ISER2			(volatile uint32_t *)0xE000E108
#define NVIC_ISER3			(volatile uint32_t *)0xE000E11C

// ARM Cortex-Mx processor NVIC ICERx register addresses
#define NVIC_ICER0			(volatile uint32_t *)0XE000E180
#define NVIC_ICER1			(volatile uint32_t *)0XE000E184
#define NVIC_ICER2			(volatile uint32_t *)0XE000E188
#define NVIC_ICER3			(volatile uint32_t *)0XE000E19C

// ARM Cortex-Mx processor IPR - priority register address calculation
#define NVIC_IPR_BASE_ADDR	(volatile uint32_t *)0xE000E400

// Generic macros
#define NO_PR_BITS_IMPLEMENTED		4

/*******************************************************************************
 * 						Device memory base addresses (Flash and SRAM)		   *
 ******************************************************************************/
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

// EXTI peripheral base address
#define EXTI_BASE_ADDR		(APB2_BASE_ADDR + 0x3C00)

// SYSCFG peripheral base address
#define SYSCFG_BASE_ADDR	(APB2_BASE_ADDR + 0x3800)

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

// EXTI register definition structure
typedef struct
{
	volatile uint32_t IMR;
	volatile uint32_t EMR;
	volatile uint32_t RTSR;
	volatile uint32_t FTSR;
	volatile uint32_t SWIER;
	volatile uint32_t PR;
} exti_regdef_t;

// SYSCFG register definition structure
typedef struct
{
	volatile uint32_t MEMRMP;
	volatile uint32_t PMC;
	volatile uint32_t EXTICR[4];
	uint32_t RESERVED1[2];
	volatile uint32_t CMPCR;
	volatile uint32_t RESERVED2[2];
	uint32_t CFGR;
} syscfg_regdef_t;


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

// EXTI
#define EXTI 		((exti_regdef_t *)EXTI_BASE_ADDR)

// SYSCFG
#define SYSCFG		((syscfg_regdef_t *)SYSCFG_BASE_ADDR)

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

// SYSCFG clock enable
#define SYSCFG_PCLK_ENABLE()		(RCC->APB2ENR |= (1 << 14))

// SYSCFG clock disable
#define SYSCFG_PCLK_DISABLE()		(RCC->APB2ENR &= ~(1 << 14))

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
 * 						Utility macros						   				   *
 ******************************************************************************/
#define GPIO_BASE_ADDR_TO_PORT_CODE(x)		(x == GPIOA) ? 0 : \
											(x == GPIOB) ? 1 : \
											(x == GPIOC) ? 2 : \
											(x == GPIOD) ? 3 : \
											(x == GPIOE) ? 4 : \
											(x == GPIOF) ? 5 : \
											(x == GPIOG) ? 6 : \
											(x == GPIOH) ? 7 : 0

/*******************************************************************************
 * 						IRQ(Interrupt Request) numbers of STM32F407x MCU	   *
 ******************************************************************************/
#define IRQ_NO_EXTI0			6
#define IRQ_NO_EXTI1			7
#define IRQ_NO_EXTI2			8
#define IRQ_NO_EXTI3			9
#define IRQ_NO_EXTI4			10
#define IRQ_NO_EXTI9_5			23
#define IRQ_NO_EXTI15_10		40

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
