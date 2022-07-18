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

//test code
//implement TELIT interface

int main(void)
{
	board_init();
    led_init();
    kitprog_uart_init();
    mce_uart_init();
    __enable_irq();

    sprintf(tx_buf, "\x1b[2J\x1b[;H"
    		"PSoC 6 MCU UART Transmit and Receive + MCE\r\n");
    cyhal_uart_write_async(&uart_obj, tx_buf, sizeof(tx_buf));
    CyDelay(UART_DELAY);

    mce_init();

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

void mce_uart_init(void)
{
	/* Initialize the UART configuration structure */
	const cyhal_uart_cfg_t uart2_config =
	{
	    .data_bits = DATA_BITS_8,
	    .stop_bits = STOP_BITS_1,
	    .parity = CYHAL_UART_PARITY_NONE,
	    .rx_buffer = rx_buf2,
	    .rx_buffer_size = RX_BUF_SIZE
	};

	/* Initialize the UART Block */
	result = cyhal_uart_init(&uart2_obj, MCE_TX, MCE_RX, NULL, &uart2_config);
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}
	/* Set the baud rate */
	result = cyhal_uart_set_baud(&uart2_obj, BAUD_RATE, &actualbaud2);
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}

	/* The UART callback handler registration */
	cyhal_uart_register_callback(&uart2_obj, uart2_event_handler, NULL);
	/* Enable required UART events */
	cyhal_uart_enable_event(&uart2_obj, (cyhal_uart_event_t)(CYHAL_UART_IRQ_RX_DONE), INT_PRIORITY, true);
}

/* Event handler callback function */
void uart2_event_handler(void *handler_arg, cyhal_uart_event_t event)
{
    (void) handler_arg;
    if ((event & CYHAL_UART_IRQ_TX_ERROR) == CYHAL_UART_IRQ_TX_ERROR)
    {
    	handle_error();
    }
    else if ((event & CYHAL_UART_IRQ_RX_DONE) == CYHAL_UART_IRQ_RX_DONE)
    {
    		uart2_flag=true;
    }
}

////////////////////////////////////////////MCE FUNCTIONS////////////////////////////////////////////////////////////////////////
void mce_checksum(uint8_t* mce_buf)
{
	uint16_t check_sum=0;
	check_sum = ( ((((uint16_t)mce_buf[1])<<8) | (uint16_t)mce_buf[0]) + ((((uint16_t)mce_buf[3])<<8) | (uint16_t)mce_buf[2]) +
			((((uint16_t)mce_buf[5])<<8) | (uint16_t)mce_buf[4]) ) * (-1);
	mce_buf[6] = (uint8_t)check_sum;
	mce_buf[7] = (uint8_t)(check_sum>>8);
}

//_Bool checksum_verify(uint8_t* mce_buf)
//{
//	uint16_t check_sum=0;
//	check_sum = ( ((((uint16_t)mce_buf[1])<<8) | (uint16_t)mce_buf[0]) + ((((uint16_t)mce_buf[3])<<8) | (uint16_t)mce_buf[2]) +
//			((((uint16_t)mce_buf[5])<<8) | (uint16_t)mce_buf[4]) ) * (-1);
//	if( check_sum == ((((uint16_t)mce_buf[7])<<8) | (uint16_t)mce_buf[6]))
//		return true;
//	else
//		return false;
//}

int16_t mce_read(uint8_t status_code)
{
	int16_t mce_reply=0;
    mce_tx_buf[0] = MCE_ADDRESS0;
    mce_tx_buf[1] = MCE_READ;
    mce_tx_buf[2] = status_code;
    mce_tx_buf[3] = 0x00;
    mce_tx_buf[4] = 0x00;
    mce_tx_buf[5] = 0x00;
    mce_checksum(mce_tx_buf);
	cyhal_uart_write_async(&uart2_obj, mce_tx_buf, sizeof(mce_tx_buf));
	cyhal_uart_read_async(&uart2_obj, mce_rx_buf, sizeof(mce_rx_buf));
	while(!uart2_flag)
		CyDelay(UART_DELAY);
	uart2_flag=false;
	return mce_reply = (((int16_t)mce_rx_buf[5])<<8) | (int16_t)mce_rx_buf[4]; //reikia pasitikrint
}

// INPUT: command = MCE_SPEED - set motor speed (positive/negative changes motor direction)
//					MCE_WRITE_REG - write value to MCE register
//data - data to be written
void mce_set_speed(int16_t speed)
{
    mce_tx_buf[0] = MCE_ADDRESS0;
    mce_tx_buf[1] = MCE_SPEED;
    mce_tx_buf[2] = 0x00;
    mce_tx_buf[3] = 0x00;
    mce_tx_buf[4] = (uint8_t)speed;
    mce_tx_buf[5] = (uint8_t)(speed>>8);
    mce_checksum(mce_tx_buf);
	cyhal_uart_write_async(&uart2_obj, mce_tx_buf, sizeof(mce_tx_buf));
	cyhal_uart_read_async(&uart2_obj, mce_rx_buf, sizeof(mce_rx_buf));
	while(!uart2_flag)
		CyDelay(UART_DELAY);
	uart2_flag=false;
}

void mce_clear_faults(void)
{
    mce_tx_buf[0] = MCE_ADDRESS0;
    mce_tx_buf[1] = MCE_CLEAR;
    mce_tx_buf[2] = 0x00;
    mce_tx_buf[3] = 0x00;
    mce_tx_buf[4] = 0x00;
    mce_tx_buf[5] = 0x00;
    mce_checksum(mce_tx_buf);
	cyhal_uart_write_async(&uart2_obj, mce_tx_buf, sizeof(mce_tx_buf));
	cyhal_uart_read_async(&uart2_obj, mce_rx_buf, sizeof(mce_rx_buf));
	while(!uart2_flag)
		CyDelay(UART_DELAY);
	uart2_flag=false;
}

void mce_init(void)
{
	mce_read(FAULT_FLAGS);
	mce_clear_faults();
	mce_set_speed(0);
}
/* [] END OF FILE */
