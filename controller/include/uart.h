/*****************************************************************************
* uart.h
*
* Driver header for NEC V25 Serial-0 interface
* This is driver is interrupt driven.
*
* Created: April 11, 2020
*
*****************************************************************************/

#ifndef __UART_H__
#define __UART_H__

#include    <stdint.h>

typedef enum
    {
        BAUD_9600,
        BAUD_19200,
        BAUD_38400,
        BAUD_57600,
        BAUD_115200
    } baud_t;

int     uart_init(baud_t);                  // Initialization
void    uart_close(void);                   // Close serial-0 (revert Flashlite system normal)

uint8_t uart_rx_data(uint8_t *, uint8_t);   // Receive data from Rx buffer
void    uart_tx_data(uint8_t *, uint8_t);   // Transmit data through Tx buffer
void    uart_putchr(uint8_t);               // Output a character
uint8_t uart_getchr(void);                  // Get a character from receive buffer
int     uart_ischar(void);                  // Check if a character is present if the receive buffer

#endif /* __UART_H__ */
