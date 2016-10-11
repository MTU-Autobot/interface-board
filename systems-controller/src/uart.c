/*
 * uart.c
 * UART initialization and related functions
 */

#include "kinetis.h"
#include "uart.h"

// uart initialization
void uartInit(uint32_t baudrate){
    // get divisor
    uint32_t divisor = BAUD2DIV(baudrate);

    SIM_SCGC4 |= SIM_SCGC4_UART0;

    // PORTB16 = UART0_RX, PORTB17 = UART0_TX
    PORTB_PCR16 |= PORT_PCR_MUX(3);
    PORTB_PCR17 |= PORT_PCR_MUX(3);
    // TX as output, RX as input
    GPIOB_PDDR |= 0x20000;
    GPIOB_PDDR &= ~(0x10000);

    // set baud rate registers
    UART0_BDH = (divisor >> 13) & 0x1F;
    UART0_BDL = (divisor >> 5) & 0xFF;
    UART0_C4 = divisor & 0x1F;

    // some fifo stuff?
    //UART0_C1 = UART_C1_ILT;
    //UART0_TWFIFO = 0;
    //UART0_RWFIFO = 4;
    //UART0_PFIFO = UART_PFIFO_TXFE | UART_PFIFO_RXFE;

    // activate tx and rx
    UART0_C2 = 0x0C;
}

// function to send a character
void uartSendChar(char data){
    while(!(UART0_S1 & UART_S1_TDRE));
    UART0_D = data;
}

// fucntion to get a char
char uartGetChar(void){
    char data = 0;
    while(!(UART0_S1 & UART_S1_RDRF));
    data = UART0_D;
    return data;
}

// function to send a string
void uartPrint(const char * data){
    uint16_t i = 0;
    while(data[i] != '\0'){
        uartSendChar(data[i]);
        i++;
    }
}
