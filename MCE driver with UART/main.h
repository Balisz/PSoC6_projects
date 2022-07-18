#include "cybsp.h"
#include "cyhal.h"
#include "stdio.h"

//#include "cy_pdl.h"

#define DATA_BITS_8     8
#define STOP_BITS_1     1
#define BAUD_RATE       115200
#define UART_DELAY      10u
#define RX_BUF_SIZE     8
#define TX_BUF_SIZE     256
#define INT_PRIORITY    3

void handle_error(void);
void board_init(void);
void led_init(void);
void kitprog_uart_init(void);
void mce_uart_init(void);
void uart_event_handler(void *handler_arg, cyhal_uart_event_t event); //kitprog uart interrupt
void uart2_event_handler(void *handler_arg, cyhal_uart_event_t event); // mce uart interrupt

cy_rslt_t result;
cyhal_uart_t uart_obj;
cyhal_uart_t uart2_obj;
uint32_t actualbaud; // kitprog uart
uint32_t actualbaud2; // mce uart
uint8_t rx_buf[RX_BUF_SIZE]; // kitprog uart
uint8_t rx_buf2[RX_BUF_SIZE]; // mce uart
char tx_buf[TX_BUF_SIZE];
_Bool uart_flag=false; //kitprog uart
_Bool uart2_flag=false; // mce uart
uint8_t rx_index=0;



////////////////////////////////////////////// MCE interface driver definitions///////////////////////////////////////

#define MCE_ADDRESS0	0x00		//address for no response
#define MCE_ADDRESS		0xFF		//address with response
//Command definition
#define MCE_READ 		0x00
#define MCE_CLEAR 		0x01
#define MCE_SPEED		0x03
#define MCE_READ_REG	0x05
#define MCE_WRITE_REG	0x06
#define MCE_SAVE_PARAM	0x20
//status code to READ
#define FAULT_FLAGS		0x00
#define	MOTOR_SPEED		0x01
#define MOTOR_STATE		0x02
#define NODE_ID			0x03

#define MCE_FAULT		-1

//checksum and puts it into tx buffer for communication
void mce_checksum(uint8_t* mce_buf);
//verifies that the received data matches received checksum
_Bool checksum_verify(uint8_t* mce_buf);
//returns value from the mce based on the sent status code
int16_t mce_read(uint8_t status_code);
//sends speed value in RPM to the MCE +-changes motor spinning direction
void mce_set_speed(int16_t data);
//clears faults
void mce_clear_faults(void);
//initializes the MCE and motor
void mce_init(void);

uint8_t mce_tx_buf[8];
uint8_t mce_rx_buf[8];
