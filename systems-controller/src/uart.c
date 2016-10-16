/*
 * uart.c
 * UART initialization and related functions
 */

#include <string.h>
#include <usb_serial.h>
#include "uart.h"

// print out a string
void serialPrint(const char * str){
    usb_serial_write((const uint8_t *)str, strlen(str));
}

// read data until we hit the stop character, function blocks until complete,
// returns the number of bytes read into the buffer
uint16_t serialRead(char * buffer, uint16_t size, char stopChar){
    if(usb_serial_available() > 0){
        // clear buffer
        memset(buffer, 0, size);
        uint16_t index = 0;

        uint8_t c = usb_serial_getchar();

        // block until we get a newline
        while(c != stopChar){
            while(usb_serial_available() > 0){
                buffer[index] = c;

                // make sure we dont overrun the buffer
                if(index < size - 1){
                    index++;
                }else{
                    return index;
                }

                c = usb_serial_getchar();
            }
        }
        return index;
    }
    return 0;
}
