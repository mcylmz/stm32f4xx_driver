/*
 * stm32407xx_usart_driver.c
 *
 *  Created on: 16 Oca 2022
 *      Author: murat.yilmaz
 */

#include "stm32f407xx_usart_driver.h"

void usart_pclk_control(usart_regdef_t *p_usartx, uint8_t enable)
{
    if (enable == ENABLE)
    {
        // Enable specified USART peripheral clock
        if (p_usartx == USART2)
        {
            USART2_PCLK_ENABLE();
        }
        else if (p_usartx == USART3)
        {
            USART3_PCLK_ENABLE();
        }
        else if (p_usartx == UART4)
        {
            UART4_PCLK_ENABLE();
        }
        else if (p_usartx == UART5)
        {
            UART5_PCLK_ENABLE();
        }
        else if (p_usartx == UART7)
        {
            UART7_PCLK_ENABLE();
        }
        else if (p_usartx == UART8)
        {
            UART8_PCLK_ENABLE();
        }
        else if (p_usartx == USART1)
        {
            USART1_PCLK_ENABLE();
        }
        else if (p_usartx == USART6)
        {
            USART6_PCLK_ENABLE();
        }
    }
    else
    {
        // Disable specified USART peripheral clock
        if (p_usartx == USART2)
        {
            USART2_PCLK_DISABLE();
        }
        else if (p_usartx == USART3)
        {
            USART3_PCLK_DISABLE();
        }
        else if (p_usartx == UART4)
        {
            UART4_PCLK_DISABLE();
        }
        else if (p_usartx == UART5)
        {
            UART5_PCLK_DISABLE();
        }
        else if (p_usartx == UART7)
        {
            UART7_PCLK_DISABLE();
        }
        else if (p_usartx == UART8)
        {
            UART8_PCLK_DISABLE();
        }
        else if (p_usartx == USART1)
        {
            USART1_PCLK_DISABLE();
        }
        else if (p_usartx == USART6)
        {
            USART6_PCLK_DISABLE();
        }
    }
}

void usart_init(usart_handle_t *p_usart_handle)
{
    // Temporary register
    uint32_t temp_reg = 0;

    // TODO Enable clock for USART peripheral
    usart_pclk_control(p_usart_handle->p_usartx, ENABLE);

    /*************** Configuration of USART_CR1 register **********************/

    // Enable USART Tx and Rx according to usart_mode configuration item
    if (p_usart_handle->usart_config.usart_mode == USART_MODE_ONLY_RX)
    {
        temp_reg &= ~(1 << 3); // Disable Tx
        temp_reg |= 1 << 2;    // Enable Rx
    }
    else if (p_usart_handle->usart_config.usart_mode == USART_MODE_ONLY_TX)
    {
        temp_reg &= ~(1 << 2); // Disable Rx
        temp_reg |= 1 << 3;    // Enable Tx
    }
    else if (p_usart_handle->usart_config.usart_mode == USART_MODE_TXRX)
    {
        temp_reg |= 2 << 2;
    }

    // Configure USART word length
    temp_reg |= p_usart_handle->usart_config.usart_word_length << 12;

    // Configure USART parity control
    if (p_usart_handle->usart_config.usart_parity_control == USART_PARITY_EN_EVEN)
    {
        // Enable parity control
        temp_reg |= 1 << 10;

        // Configure even parity
        temp_reg &= ~(1 << 9);
    }
    else if (p_usart_handle->usart_config.usart_parity_control == USART_PARITY_EN_ODD)
    {
        // Enable parity control
        temp_reg |= 1 << 10;

        // Configure odd parity
        temp_reg |= 1 << 9;
    }
    else if (p_usart_handle->usart_config.usart_parity_control == USART_PARITY_DISABLE)
    {
        temp_reg &= ~(1 << 10);
    }

    // Write configurations into USART_CR1 register
    p_usart_handle->p_usartx->CR1 = temp_reg;

    /*************** Configuration of USART_CR2 register **********************/
    temp_reg = 0;

    // Configure USART number of stop bits
    temp_reg |= p_usart_handle->usart_config.usart_no_stop_bits << 12;

    // Write configuration into USART_CR2 register
    p_usart_handle->p_usartx->CR2 = temp_reg;

    /*************** Configuration of USART_CR3 register **********************/
    temp_reg = 0;

    // Configure USART hardware flow control
    if (p_usart_handle->usart_config.usart_hw_flow_control == USART_HW_FLOW_CONTROL_CTS)
    {
        // Disable RTS
        temp_reg &= ~(1 << 8);

        // Enable CTS
        temp_reg |= 1 << 9;
    }
    else if (p_usart_handle->usart_config.usart_hw_flow_control == USART_HW_FLOW_CONTROL_RTS)
    {
        // Disable CTS
        temp_reg &= ~(1 << 9);

        // Enable RTS
        temp_reg |= 1 << 8;
    }
    else if (p_usart_handle->usart_config.usart_hw_flow_control == USART_HW_FLOW_CONTROL_CTS_RTS)
    {
        // Enable both CTS and RTS
        temp_reg |= 1 << 9;
        temp_reg |= 1 << 8;
    }
    else if (p_usart_handle->usart_config.usart_hw_flow_control == USART_HW_FLOW_CONTROL_NONE)
    {
        // Disable both CTS and RTS
        temp_reg &= ~(1 << 9);
        temp_reg &= ~(1 << 8);
    }

    // Write configuration into USART_CR3 register
    p_usart_handle->p_usartx->CR3 = temp_reg;

    /*************** Configuration of USART_BRR register **********************/
    // TODO Configure USART_BRR register

}

void usart_uninit(usart_regdef_t *p_usartx)
{
    // Reset all register of specified USARTx peripheral
    if (p_usartx == USART2)
    {
        USART2_REG_RESET();
    }
    else if (p_usartx == USART3)
    {
        USART3_REG_RESET();
    }
    else if (p_usartx == UART4)
    {
        UART4_REG_RESET();
    }
    else if (p_usartx == UART5)
    {
        UART5_REG_RESET();
    }
    else if (p_usartx == UART7)
    {
        UART7_REG_RESET();
    }
    else if (p_usartx == UART8)
    {
        UART8_REG_RESET();
    }
    else if (p_usartx == USART1)
    {
        USART1_REG_RESET();
    }
    else if (p_usartx == USART6)
    {
        USART6_REG_RESET();
    }
}

void usart_send_data(usart_regdef_t *p_usartx, uint8_t *p_tx_buffer, uint32_t length)
{

}


uint8_t usart_send_data_interrupt(usart_handle_t *p_usart_handle, uint8_t *p_tx_buffer, uint32_t length)
{

}

uint8_t usart_receive_data_interrupt(usart_handle_t *p_usart_handle, uint8_t *p_rx_buffer, uint32_t length)
{

}

void usart_irq_interrupt_config(uint8_t irq_number, uint8_t enable)
{

}

void usart_irq_priority_config(uint8_t irq_number, uint32_t irq_priority)
{

}

void usart_irq_handling(usart_handle_t *p_usart_handle)
{

}

void usart_peripheral_control(usart_regdef_t *p_usartx, uint8_t enable)
{

}

uint8_t usart_get_flag_status(usart_regdef_t *p_usartx, uint32_t flag_name)
{

}

void usart_clear_flag(usart_regdef_t *p_usartx, uint16_t status_flag_name)
{

}

void usart_app_event_callback(usart_handle_t *p_usart_handle, uint8_t app_event)
{

}
