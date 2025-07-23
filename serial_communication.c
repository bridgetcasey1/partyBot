/*
 * Serial_Communication.c
 *
 * Created: 20/11/2021 12:06:12 PM
 * Author : gatorBots
 */

#include "macros.h"
#include "serial_communication.h"
#include <avr/io.h>
#include <stdio.h>

// Initiate serial communication with a baud rate of 9600 and receiving and 
// transmitting data enabled. 
void init_serial_communication(void)  {	
   
    CLKPR = (1 << CLKPCE); // Enable a change to CLKPR
    CLKPR = 0; // Set the CLKDIV to 0 (default is division by 8)
    unsigned ubrr = BAUD_PRESCALER;

    // Set Baud rate
    UBRR0H = (ubrr >> 8);
    UBRR0L = (ubrr);

    // Set frame format to 8N1 (8 data bits, 1 stop bit)
    UCSR0C = 0x06;

    // Enable transmitter and receiver
    UCSR0B = (1 << TXEN0) | (1 << RXEN0);
	
    // Enable receive interrupt
    UCSR0B |= (1 << RXCIE0);
}

// Send given data via serial communication.
void send_serial_data(char data[])  {
    int i = 0;
    while(data[i]) {
        send_serial_char(data[i]);
        i++;
    }
}

// Send given character via serial communication. 
void send_serial_char(char c) {
	while (!( UCSR0A & (1<<UDRE0))); // Wait for empty transmit buffer
	UDR0 = c; // Put data into buffer - sends the data
}