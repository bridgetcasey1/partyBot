/*
 * serial_communication.h
 *
 * Created: 20/11/2021 12:04:42 PM
 * Author : bridgetCasey
 */

#pragma once

// CONSTANTS
#define F_CPU 8000000 // Clock speed
#define BAUD_RATE 9600 // 9600bps
#define BAUD_PRESCALER F_CPU / 16 / BAUD_RATE - 1
//#define BUFFER_SIZE 100

// Initiate serial communication with a baud rate of 9600 and receiving and
// transmitting data enabled.
void initialise_serial_communication(void);

// Send given data via serial communication.
void send_serial_data(char data[]);