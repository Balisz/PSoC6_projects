#include "cybsp.h"
#include "cyhal.h"
#include "stdio.h"

//#include "cy_pdl.h"

#define DATA_BITS_8     8
#define STOP_BITS_1     1
#define BAUD_RATE       115200
#define UART_DELAY      10u
#define RX_BUF_SIZE     4
#define TX_BUF_SIZE     256
#define INT_PRIORITY    3

void handle_error(void);
void board_init(void);
void led_init(void);
void kitprog_uart_init(void);
void uart_event_handler(void *handler_arg, cyhal_uart_event_t event);

cy_rslt_t result;
cyhal_uart_t uart_obj;
uint32_t actualbaud;
uint8_t rx_buf[RX_BUF_SIZE];
char tx_buf[TX_BUF_SIZE];
_Bool uart_flag=false;
