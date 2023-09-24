/*
 * Serial_Communication.c
 *
 * Created: 20/11/2021 12:06:12 PM
 * Author : bridgetCasey
 */

#include "serial_communication.h"
#include <avr/io.h>
#include <stdio.h>

// CONSTANTS
//#define F_CPU 8000000 // Clock speed
//#define BAUD_RATE 9600 // 9600bps
//#define BAUD_PRESCALER F_CPU / 16 / BAUD_RATE - 1

// Initiate serial communication with a baud rate of 9600 and receiving and 
// transmitting data enabled. 
void initialise_serial_communication(void)  {	
   
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
        while (!( UCSR0A & (1<<UDRE0))); // Wait for empty transmit buffer
        UDR0 = data[i]; // Put data into buffer - sends the data
        i++;
    }
}