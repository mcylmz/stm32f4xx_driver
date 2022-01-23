/*
 * stm32f407xx_usart_driver.h
 *
 *  Created on: 16 Oca 2022
 *      Author: murat.yilmaz
 */

#ifndef INC_STM32F407XX_USART_DRIVER_H_
#define INC_STM32F407XX_USART_DRIVER_H_

#include "stm32f407xx.h"

// Configuration structure for USART peripheral
typedef struct
{
    uint8_t usart_mode;
    uint32_t usart_baudrate;
    uint8_t usart_no_stop_bits;
    uint8_t usart_word_length;
    uint8_t usart_parity_control;
    uint8_t usart_hw_flow_control;
} usart_config_t;

// USART handle structure
typedef struct
{
    usart_regdef_t *p_usartx;
    usart_config_t usart_config;
} usart_handle_t;

// Possible options for usart_mode
#define USART_MODE_ONLY_TX		0
#define USART_MODE_ONLY_RX		1
#define USART_MODE_TXRX			2

// Possible options for usart_baudrate
#define USART_BAUDRATE_1200			1200
#define USART_BAUDRATE_2400			2400
#define USART_BAUDRATE_9600			9600
#define USART_BAUDRATE_19200 		19200
#define USART_BAUDRATE_38400 		38400
#define USART_BAUDRATE_57600 		57600
#define USART_BAUDRATE_115200 		115200
#define USART_BAUDRATE_230400 		230400
#define USART_BAUDRATE_460800 		460800
#define USART_BAUDRATE_921600 		921600
#define USART_BAUDRATE_2M 			2000000
#define USART_BAUDRATE_3M 			3000000

// Possible options for usart_no_stop_bits
#define USART_STOP_BITS_1		0
#define USART_STOP_BITS_0_5		1
#define USART_STOP_BITS_2		2
#define USART_STOP_BITS_1_5		3

// Possible options for usart_world_length
#define USART_WORD_LENGTH_8_BITS		0
#define USART_WORD_LENGTH_9_BITS		1

// Possible options for usart_parity_control
#define USART_PARITY_DISABLE		0
#define USART_PARITY_EN_EVEN		1
#define USART_PARITY_EN_ODD			2

// Possible options for usart_hw_flow_control
#define USART_HW_FLOW_CONTROL_NONE			0
#define USART_HW_FLOW_CONTROL_CTS			1
#define USART_HW_FLOW_CONTROL_RTS			2
#define USART_HW_FLOW_CONTROL_CTS_RTS		3

/*******************************************************************************
 * 						APIs supported by USART driver						   *
 ******************************************************************************/
// Peripheral clock setup
void usart_pclk_control(usart_regdef_t *p_usartx, uint8_t enable);

// Initialization and uninitialization
void usart_init(usart_handle_t *p_usart_handle);
void usart_uninit(usart_regdef_t *p_usartx);

// Data send and receive
void usart_send_data(usart_regdef_t *p_usartx, uint8_t *p_tx_buffer, uint32_t length);
void usart_receive_data(usart_regdef_t *p_usartx, uint8_t *p_rx_buffer, uint32_t length);
uint8_t usart_send_data_interrupt(usart_handle_t *p_usart_handle, uint8_t *p_tx_buffer, uint32_t length);
uint8_t usart_receive_data_interrupt(usart_handle_t *p_usart_handle, uint8_t *p_rx_buffer, uint32_t length);

// IRQ configuration and ISR handling
void usart_irq_interrupt_config(uint8_t irq_number, uint8_t enable);
void usart_irq_priority_config(uint8_t irq_number, uint32_t irq_priority);
void usart_irq_handling(usart_handle_t *p_usart_handle);

// Other peripheral control APIs
void usart_peripheral_control(usart_regdef_t *p_usartx, uint8_t enable);
uint8_t usart_get_flag_status(usart_regdef_t *p_usartx, uint32_t flag_name);
void usart_clear_flag(usart_regdef_t *p_usartx, uint16_t status_flag_name);

// Application callback
void usart_app_event_callback(usart_handle_t *p_usart_handle, uint8_t app_event);

#endif /* INC_STM32F407XX_USART_DRIVER_H_ */
