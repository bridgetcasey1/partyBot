/*
 * i2c_communication.c
 *
 * Created: 25/11/2021 04:44:27 PM
 * Author : bridgetCasey
 */

#include "i2c_communication.h"

// Initiate TWI communication with specified SCL frequency in kHz and with
// pull-ups either enabled or disabled depending on input.
void initialise_i2c_communication(int SCL_frequency, bool pullups_enabled) {
	
	// Set I2C pins to be outputs 
	DDRC |= (1 << SDA_PIN) | (1 << SCL_PIN);
	
	if (pullups_enabled) {
		PORTC |= (1 << SDA_PIN) | (1 << SCL_PIN);
	} else {
		PORTC &= ~((1 << SDA_PIN) | (1 << SCL_PIN));
		DDRC &= ~((1 << SDA_PIN) | (1 << SCL_PIN));
	}
	
	// Set TWBR for specified SCL frequency
	TWBR = F_CPU/(2 * SCL_frequency * 1000) - 8;
	TWSR = 0;  // No prescalar
	TWCR = (1<<TWEN);  // Enable I2C
	// TWBR = ((F_CPU/SCL_CLOCK)-16)/2;  /* must be > 10 for stable operation */
}  

// Send start condition 
void i2c_start(void) {
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTA);
	while(!(TWCR & (1 << TWINT))); // Wait for TWINT flag to set
	
	// Write device address with WRITE FLAG
	TWDR = address | TW_WRITE;
	TWCR = (1 << TWINT) | (1 << TWEN);
	while(!(TWCR & (1 << TWINT)));
}

void i2c_write(char c) {
	TWDR = c;						//Move value to I2C
	TWCR = (1<<TWINT) | (1<<TWEN);	//Enable I2C and clear interrupt
	while  (!(TWCR &(1<<TWINT)));
}

// Send stop condition 
void i2c_stop(void) {
	
	//Send STOP command
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
	
	// Wait until stop condition is executed and bus released
	while(TWCR & (1<<TWSTO));
}

// Write given data to specified address via I2C.
void i2c_write(uint8_t address, uint8_t data) {
	
	// Send START condition
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTA);
	while(!(TWCR & (1 << TWINT))); // Wait for TWINT flag to set

	// Write device address with WRITE FLAG
	TWDR = address | TW_WRITE;
	TWCR = (1 << TWINT) | (1 << TWEN);
	while(!(TWCR & (1 << TWINT)));

	// Send data to device
	TWDR = data;
	TWCR = (1 << (TWINT)) | (1 << TWEN);
	while(!(TWCR & (1 << TWINT)));	
	
	//Send STOP command
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
	  
	// Wait until stop condition is executed and bus released
	while(TWCR & (1<<TWSTO));
}

// Write given data to specified register on specified device.
void i2c_write_register(uint8_t device_address, uint8_t reg, uint8_t data)
{
	// Send START condition
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTA);
	while (!(TWCR & (1 << TWINT))); // Wait for TWINT flag to set

	// Write chip address with WRITE FLAG
	TWDR = device_address | TW_WRITE;
	TWCR = (1 << TWINT) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));
	
	// Set register to write to
	TWDR = reg;
	TWCR = (1 << TWINT) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));

	// Write to register
	TWDR = data;
	TWCR = (1 << (TWINT)) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));
	
	//Send STOP command
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
}