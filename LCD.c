/*
 * LCD.c
 *
 * Created: 28/11/2021 03:12:08 PM
 * Author : bridgetCasey
 *
 */

#include "LCD.h"

#define I2C_FREQUENCY 400

// Send command to LCD
void send_command(uint8_t value) {
	i2c_write(LCD_ADDRESS, 0x80);
	_delay_ms(5);
	i2c_write(LCD_ADDRESS, value);
	_delay_ms(5);
}

// Set RGB 
void set_RGB(uint8_t r, uint8_t g, uint8_t b){
	i2c_write_register(RGB_ADDRESS, REG_RED, r);
	i2c_write_register(RGB_ADDRESS, REG_GREEN, g);
	i2c_write_register(RGB_ADDRESS, REG_BLUE, b);
}

// Send character to LCD
void write_char(uint8_t value) {

	i2c_write(LCD_ADDRESS, 0x40);
	i2c_write(LCD_ADDRESS, value);
}

// Send string to LCD
void send_string(const char *str) {
	uint8_t i;
	for(i = 0; str[i] != '\0';i++)
	write_char(str[i]);
}

// Initialise LCD
void initialise_LCD(void) {
	
	// Initialize I2C with SCL freq of 400 KHz with internal pull-ups enabled
	initialise_i2c_communication(I2C_FREQUENCY, true);
	
	uint8_t show_function = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;
	show_function |= LCD_2LINE;
	_delay_ms(50);  // Need to wait at least 40ms after power up
	
	// Send function set command sequence
	send_command(LCD_FUNCTIONSET | show_function);
	_delay_ms(5);  // Wait more than 4.1ms
	
	// Second try
	send_command(LCD_FUNCTIONSET | show_function);
	_delay_ms(5);

	// Third go
	send_command(LCD_FUNCTIONSET | show_function);
	
	// Turn the display on with no cursor or blinking default
	uint8_t show_control = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
	
	// Display on LCD
	show_control |= LCD_DISPLAYON;
	send_command(LCD_DISPLAYCONTROL | show_control);

	// Clear screen
	send_command(LCD_CLEARDISPLAY);
	_delay_us(2000);

	// Initialize to default text direction (for romance languages)
	uint8_t show_mode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
	
	// Set the entry mode
	send_command(LCD_ENTRYMODESET | show_mode);
	
	// Initialise back light 
	i2c_write_register(RGB_ADDRESS, REG_MODE1, 0);
	// Set LEDs controllable by both PWM and GRPPWM registers
	i2c_write_register(RGB_ADDRESS, REG_OUTPUT, 0xFF);
	// Set MODE2 values
	// 0010 0000 -> 0x20  (DMBLNK to 1, i.e. blinky mode)
	i2c_write_register(RGB_ADDRESS, REG_MODE2, 0x20);
	
	// Set colour white 
	set_RGB(255, 255, 255);
	
	// Set curser to (0,0)
	uint8_t to_send = (0 == 0 ? 0|0x80 : 0|0xC0);
	i2c_write(LCD_ADDRESS, to_send);
	
	// Display PartyBot on LCD
	send_string("PartyBot");
}

/*
// Send command to LCD
void send_command(uint8_t value) {
	i2c_write(LCD_ADDRESS,0x80);
	i2c_write(LCD_ADDRESS,value);
}

// Display on LCD
void display(struct LCD *lcd) {
	lcd->show_control |= LCD_DISPLAYON;
	send_command(LCD_DISPLAYCONTROL | lcd->show_control);
}

// Clear screen 
void clear() {
	send_command(LCD_CLEARDISPLAY); // Clear display and set cursor position to zero
	_delay_us(2000);        
}

// Write given data to given address
void set_reg(uint8_t addr, uint8_t data) {
	i2c_write(RGB_ADDRESS,addr);
	i2c_write(RGB_ADDRESS,data);
}

//
void set_RGB(uint8_t r, uint8_t g, uint8_t b) {
	set_reg(REG_RED, r);
	set_reg(REG_GREEN, g);
	set_reg(REG_BLUE, b);
}

// Set LCD to the colour white
void set_color_white(void) {
	set_RGB(255, 255, 255);
}

// Set cursor of LCD
void set_cursor(uint8_t col, uint8_t row){
	col = (row == 0 ? col|0x80 : col|0xc0);		
	i2c_write(LCD_ADDRESS,0x80);
	i2c_write(LCD_ADDRESS,col);
}

void write_char(uint8_t value) {
	i2c_write(LCD_ADDRESS,0x40);
	i2c_write(LCD_ADDRESS,value);
}

void send_string(const char *str){
	uint8_t i;
	for(i = 0; str[i] != '\0';i++) {
		write_char(str[i]);
	}
}

// Initialise LCD1602 LCD module 
void initialise_LCD(struct LCD *lcd) {
		
	lcd->show_function = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;
	
	lcd->rows = LCD_ROWS;
	lcd->cols = LCD_COLS;
	
	if (lcd->rows > 1) {
		lcd->show_function |= LCD_2LINE;
	}
	lcd->current_line = 0; 
	
	// Wait 40ms after power on to send commands
	_delay_ms(50);
	
	// Send function set command sequence
	send_command(LCD_FUNCTIONSET | lcd->show_function);
	_delay_ms(5);  // wait more than 4.1ms
	
	// Second try
	send_command(LCD_FUNCTIONSET | lcd->show_function);
	_delay_ms(5);

	// Third go
	send_command(LCD_FUNCTIONSET | lcd->show_function);
	
	// Turn the display on with no cursor or blinking default
	lcd->show_control = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
	display(lcd);
	
	// Clear screen 
	clear();
	
	// Initialise to default text direction (for romance languages)
	lcd->show_mode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
	
	// Set the entry mode
	send_command(LCD_ENTRYMODESET | lcd->show_mode);
	
	// Initialise back light
	set_reg(REG_MODE1, 0);
	// Set LEDs controllable by both PWM and GRPPWM registers
	set_reg(REG_OUTPUT, 0xFF);
	// Set MODE2 values
	// 0010 0000 -> 0x20  (DMBLNK to 1, i.e. blinky mode)
	set_reg(REG_MODE2, 0x20);
	
	set_color_white();
	
	set_cursor(0,0);
	
	// Print PartyBot
	send_string("PartyBot");
}*/