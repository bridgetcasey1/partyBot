/*
 * LCD.h
 *
 * Created: 28/11/2021 03:09:21 PM
 * Author : bridgetCasey
 */

#pragma once

#include <stdint.h>
#include "i2c_communication.h"

// I2C Addresses
#define LCD_ADDRESS	(0x7C >> 1)
#define RGB_ADDRESS	(0xC0 >> 1)

// Dimension Constants
#define LCD_ROWS	2
#define LCD_COLS	16

// Colours
#define WHITE           0
#define RED             1
#define GREEN           2
#define BLUE            3

#define REG_RED         0x04  // PWM2
#define REG_GREEN       0x03  // PWM1
#define REG_BLUE        0x02  // PWM0

#define REG_MODE1       0x00
#define REG_MODE2       0x01
#define REG_OUTPUT      0x08

// Commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// Flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// Flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// Flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// Flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x8DOTS 0x00

// LCD structure
/*
struct LCD {
	uint8_t show_function;
	uint8_t show_control;
	uint8_t show_mode;
	uint8_t initialised;
	uint8_t num_lines;
	uint8_t current_line;
	uint8_t lcd_addr;
	uint8_t RGB_addr;
	uint8_t cols;
	uint8_t rows;
	uint8_t backlight_val;
};
*/

// Functions
void initialise_LCD(void);
//void initialise_LCD(struct LCD *lcd);