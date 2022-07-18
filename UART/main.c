/******************************************************************************
* File Name: main.c
*
* Description: This example demonstrates the UART transmit and receive
*              operation using HAL APIs
*
* Related Document: See Readme.md 
*
*******************************************************************************
* (c) 2019-2021, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* identify Cypress against all liability.
*******************************************************************************/

#include "main.h"

int main(void)
{
	board_init();
    led_init();
    kitprog_uart_init();
    __enable_irq();
    sprintf(tx_buf, "\x1b[2J\x1b[;H"
    		"PSoC 6 MCU UART Transmit and Receive\r\n");
    cyhal_uart_write_async(&uart_obj, tx_buf, sizeof(tx_buf));

    for (;;)
    {
    	CyDelay(50);
    }
}

void handle_error(void)
{
     /* Disable all interrupts. */
    __disable_irq();

    CY_ASSERT(0);
}

//Error handler function
void board_init(void)
{
    result = cybsp_init(); /* Initialize the device and board peripherals */
    if (result != CY_RSLT_SUCCESS)
        handle_error();
}

void led_init(void)
{
    /*LED Initialization*/
    result = cyhal_gpio_init( LED1, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_ON);
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}
    result = cyhal_gpio_init( LED2, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}
}

//KITPROG UART init funtion
void kitprog_uart_init(void)
{
	/* Initialize the UART configuration structure */
	const cyhal_uart_cfg_t uart_config =
	{
	    .data_bits = DATA_BITS_8,
	    .stop_bits = STOP_BITS_1,
	    .parity = CYHAL_UART_PARITY_NONE,
	    .rx_buffer = rx_buf,
	    .rx_buffer_size = RX_BUF_SIZE
	};

	/* Initialize the UART Block */
	result = cyhal_uart_init(&uart_obj, KITPROG_TX, KITPROG_RX, NULL, &uart_config);
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}
	/* Set the baud rate */
	result = cyhal_uart_set_baud(&uart_obj, BAUD_RATE, &actualbaud);
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}

	/* The UART callback handler registration */
	cyhal_uart_register_callback(&uart_obj, uart_event_handler, NULL);
	/* Enable required UART events */
	cyhal_uart_enable_event(&uart_obj, (cyhal_uart_event_t)(CYHAL_UART_IRQ_RX_NOT_EMPTY), INT_PRIORITY, true);
}

/* Event handler callback function */
void uart_event_handler(void *handler_arg, cyhal_uart_event_t event)
{
    (void) handler_arg;
    if ((event & CYHAL_UART_IRQ_TX_ERROR) == CYHAL_UART_IRQ_TX_ERROR)
    {
    	handle_error();
    }
    else if ((event & CYHAL_UART_IRQ_RX_NOT_EMPTY) == CYHAL_UART_IRQ_RX_NOT_EMPTY)
    {
    	uart_flag=true;
    }
}
/* [] END OF FILE */
