/*
 * i2c_communication.h
 *
 * Created: 25/11/2021 04:48:08 PM
 * Author : bridgetCasey
 */

#pragma once

#include <avr/io.h>
#include <stdbool.h>
#include <util/twi.h>
#include <util/delay.h>

// Pins used for SCL
#define SCL_PIN PORTC5
#define SDA_PIN PORTC4

#define F_CPU 8000000 // Clock speed

// Initiate I2C communication with specified SCL frequency in kHz and with
// pull-ups either enabled or disabled depending on input.
void initialise_i2c_communication(int SCL_frequency, bool pullups_enabled);

// Write given data to specified address via I2C.
void i2c_write(uint8_t address, uint8_t data);

// Write given data to specified register on specified device.
void i2c_write_register(uint8_t device_address, uint8_t reg, uint8_t data);