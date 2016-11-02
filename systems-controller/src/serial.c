/*
 * serial.c
 * UART initialization and related functions
 */

#include <string.h>
#include <usb_serial.h>
#include "serial.h"

// print out a string
void serialPrint(const char * str){
    usb_serial_write((const uint8_t *)str, strlen(str));
}

// read data until we hit the stop character, function blocks until complete,
// returns the number of bytes read into the buffer
uint16_t serialRead(char * str, uint16_t size, char stopChar){
    // read buffer, just set at 128 bytes for now
    static char buffer[128] = {'\0'};

    if(usb_serial_available() > 0){
        static uint16_t index = 0;

        while(usb_serial_available() > 0){
            // get char
            char c = usb_serial_getchar();

            // check for stop character
            if(c == stopChar || index >= size - 1){
                // set end of input string, copy to output, and clear buffer
                buffer[index] = '\0';
                strcpy(str, buffer);
                memset(buffer, 0, 128);

                uint16_t readBytes = index;
                index = 0;
                return readBytes;
            }

            // copy new char into buffer
            buffer[index++] = c;
            return 0;
        }
    }
    return 0;
}
