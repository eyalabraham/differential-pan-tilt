/*****************************************************************************
* uart.c
*
* Driver for NEC V25 Serial-0 interface
* This is driver is interrupt driven.
*
* Created: April 11, 2020
*
*****************************************************************************/

#include    <stdio.h>
#include    <stdlib.h>
#include    <stdint.h>
#include    <limits.h>
#include    <i86.h>

#include    "v25.h"
#include    "uart.h"

/****************************************************************************
  Module definitions
****************************************************************************/
#define     __VERSION__     "1.0"

// Serial-0 UART
#define     SER0_MODE       0xc9    // Enable TX and Rx, 8bit, no parity, one stop bit
#define     SER0_RX_INT     0x07    // Serial interrupt control
#define     SER0_CTRL1      2       // Fclk/8
#define     SER0_BAUD1      130     // 9600 BAUD
#define     SER0_CTRL2      2       // Fclk/8
#define     SER0_BAUD2      65      // 19200 BAUD
#define     SER0_CTRL3      1       // Fclk/4
#define     SER0_BAUD3      65      // 38400 BAUD
#define     SER0_CTRL4      0       // Fclk/2
#define     SER0_BAUD4      87      // 57600 BAUD
#define     SER0_CTRL5      0       // Fclk/2
#define     SER0_BAUD5      43      // 115200 BAUD (** 0.9% error)

#define     SER0_TXB_EMPTY  0x20    // Tx buffer empty

#define     SER0_RX_IRQ     13

#define     UART_BUFF_LEN   32      // Data byte buffer

/****************************************************************************
  Types
****************************************************************************/

/****************************************************************************
  Function prototypes
****************************************************************************/
static void __interrupt uart_isr(void);

/****************************************************************************
  External function prototypes
****************************************************************************/

/****************************************************************************
  Globals
****************************************************************************/
static struct SFR __far *pSfr;

static uint8_t     uart_buffer[UART_BUFF_LEN];

volatile static int inIndex;
volatile static int outIndex;
volatile static int bytesInBuffer;

/* ---------------------------------------------------------------------------
 * uart_init()
 *
 * Hard coded UART setup: 8 data bits, 1 stop, no parity, 19,200 BAUD
 *
 * param:   Baud rate selection
 * return:  '0' initialization ok, '-1' initialization failed
 *
 */
int uart_init(baud_t baud_rate)
{
    uint16_t __far *wpVector;

    _disable();

    /* Global pointer to SFR
     */
    pSfr = MK_FP(0xf000, 0xff00);

    /* Setup receive circular buffer
     */
    inIndex = 0;
    outIndex = 0;
    bytesInBuffer = 0;

    /* UART re-initialization
     */
    pSfr->scm0 = SER0_MODE;

    if ( baud_rate == BAUD_9600 )
    {
        pSfr->scc0 = SER0_CTRL1;
        pSfr->brg0 = SER0_BAUD1;
    }
    else if ( baud_rate == BAUD_19200 )
    {
        pSfr->scc0 = SER0_CTRL2;
        pSfr->brg0 = SER0_BAUD2;
    }
    else if ( baud_rate == BAUD_38400 )
    {
        pSfr->scc0 = SER0_CTRL3;
        pSfr->brg0 = SER0_BAUD3;
    }
    else if ( baud_rate == BAUD_57600 )
    {
        pSfr->scc0 = SER0_CTRL4;
        pSfr->brg0 = SER0_BAUD4;
    }
    else if ( baud_rate == BAUD_115200 )
    {
        pSfr->scc0 = SER0_CTRL5;
        pSfr->brg0 = SER0_BAUD5;
    }
    else
        return -1;

    /* Setup receive interrupt vectors
     */
    wpVector      = MK_FP(0, (SER0_RX_IRQ * 4));
    *wpVector++   = FP_OFF(uart_isr);
    *wpVector     = FP_SEG(uart_isr);

    pSfr->sric0 = SER0_RX_INT;

    _enable();

    return 0;
}

/* ---------------------------------------------------------------------------
 * uart_rx_data()
 *
 * Attempt to read 'byteCount' data bytes from the UART input buffer.
 * Function read as much data as possible and returns number of data bytes
 * actually read.
 *
 * param:   Buffer for read bytes and buffer size
 * return:  Bytes read and returned
 *
 */
uint8_t uart_rx_data(uint8_t *data, uint8_t byteCount)
{
    uint8_t rxCount;
    uint8_t i;

    if ( bytesInBuffer == 0 )
        return 0;

    // figure out how many bytes to transfer
    rxCount = (byteCount < bytesInBuffer) ? byteCount : bytesInBuffer;

    // transfer data to caller's buffer
    for ( i = 0; i < rxCount; i++ )
    {
        data[i] = uart_buffer[outIndex++];  // get byte from input buffer to caller's buffer

        if ( outIndex == UART_BUFF_LEN )    // update circular buffer out index
            outIndex = 0;

        bytesInBuffer--;                    // decrement byte count
    }

    return rxCount;
}

/* ---------------------------------------------------------------------------
 * uart_tx_Data()
 *
 * Function write 'byteCount' data bytes to the UART transmitter.
 * Function blocks until all bytes have been written
 *
 * param:   Buffer with data bytes and byte count to send
 * return:  nothing
 *
 */
void uart_tx_data(uint8_t *data, uint8_t byteCount)
{
    int i;

    for (i = 0; i < byteCount; i++)
    {
        uart_putchr(data[i]);
    }
}

/* ----------------------------------------------------------------------------
 * uart_putchr()
 *
 * Send character c down the UART Tx, wait until tx holding register is empty
 *
 * param:   Character to transmit
 * return:  nothing
 *
 */
void uart_putchr(uint8_t c)
{
    if ( c == 0 )
        return;

    while ( (pSfr->scs0 & SER0_TXB_EMPTY) == 0 )
    { }

    pSfr->txb0 = c;
}

/* ----------------------------------------------------------------------------
 * uart_getchr()
 *
 * Get character from UART buffer
 *
 * param:   nothing
 * return:  Character/byte read from UART (circular) buffer
 *
 */
uint8_t uart_getchr(void)
{
    uint8_t byte = 0;

    if ( bytesInBuffer > 0 )
    {
        byte = uart_buffer[outIndex++];     // if buffer has data, get byte from input buffer

        if ( outIndex == UART_BUFF_LEN )    // update circular buffer out index
            outIndex = 0;

        bytesInBuffer--;                    // decrement byte count
    }

    return byte;
}

/* ----------------------------------------------------------------------------
 * uart_ischar()
 *
 * Test UART buffer for unread characters, return available characters to read
 *
 * param:   nothing
 * return:  '0' if no data in buffer, otherwise byte count
 *
 */
int uart_ischar(void)
{
    return bytesInBuffer;
}

/* ----------------------------------------------------------------------------
 * uart_isr()
 *
 * This ISR will trigger when the serial-0 UART receives a data byte.
 * Data is read and stored in a circular buffer.
 * Data can be polled and read from the circular buffer using
 * uart_getchr() or uart_rx_data()
 *
 */
static void __interrupt uart_isr(void)
{
    static uint8_t byte;

    byte = byte = pSfr->rxb0;

    if ( bytesInBuffer < UART_BUFF_LEN )
    {
        uart_buffer[inIndex++] = byte;  // If buffer has free space, store byte in buffer

        if ( inIndex == UART_BUFF_LEN ) // Update circular buffer index
            inIndex = 0;

        bytesInBuffer++;                // Increment byte count
    }

    /* end of interrupt epilogue for NEC V25
     */
    __asm { db  0x0f
            db  0x92
          }
}
