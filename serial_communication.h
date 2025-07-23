 /*
 * serial_communication.h
 *
 * Created: 20/11/2021 12:04:42 PM
 * Author : bridgetCasey
 */

// Initiate serial communication with a baud rate of 9600 and receiving and transmitting data enabled.
void init_serial_communication(void);

// Send given data via serial communication.
void send_serial_data(char data[]);

// Send given character via serial communication.
void send_serial_char(char c);