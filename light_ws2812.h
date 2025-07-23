/*
 * light weight WS2812 lib include
 *
 * Version 2.3  - Nev 29th 2015
 * Author: Tim (cpldcpu@gmail.com) 
 *
 * Please do not change this file! All configuration is handled in 
 * "ws2812_config.h"
 *
 * License: GNU GPL v2+ (see License.txt)
 +
 */ 

#pragma once 

#include <avr/io.h>
#include <avr/interrupt.h>

///////////////////////////////////////////////////////////////////////
// Define Reset time in 탎.
//
// This is the time the library spends waiting after writing the data.
//
// WS2813 needs 300 탎 reset time.
// WS2812 and clones only need 50 탎.
//
///////////////////////////////////////////////////////////////////////
#if !defined(ws2812ResetTime)
#define ws2812ResetTime 300
#endif

///////////////////////////////////////////////////////////////////////
// Define I/O pin
///////////////////////////////////////////////////////////////////////
#if !defined(ws2812Port)
#define ws2812Port B // Set data port for LED output. 
#endif

#if !defined(ws2812Pin)
#define ws2812Pin  1 // Set data out pin for LED strip
#define ws2812Pin2 2 // Set data out pin for LED rings
#endif

/*
 *  Structure of the LED array
 *
 * cRGB:     RGB  for WS2812S/B/C/D, SK6812, SK6812Mini, SK6812WWA, APA104, 
 * APA106
 * cRGBW:    RGBW for SK6812RGBW
 */

struct cRGB  { uint8_t g; uint8_t r; uint8_t b; };
struct cRGBW { uint8_t g; uint8_t r; uint8_t b; uint8_t w;};
	
/* User Interface
 * 
 * Input:
 *         ledarray:           An array of GRB data describing the LED colors
 *         numberOfLeds:     The number of LEDs to write
 *         pinmask (optional): Bitmask describing the output bin. e.g. _BV(PB0)
 *
 * The functions will perform the following actions:
 *         - Set the data-out pin as output
 *         - Send out the LED data 
 *         - Wait 50탎 to reset the LEDs
 */

void ws2812SetLeds(struct cRGB *ledArray, uint16_t numberOfLeds);
void ws2812SetLedsPin(struct cRGB *ledArray, uint16_t numberOfLeds,
        uint8_t pinMask);
void ws2812SetLedsRgbw(struct cRGBW *ledArray, uint16_t numberOfLeds);

/* 
 * Old interface / Internal functions
 *
 * The functions take a byte-array and send to the data output as WS2812 
 * bitstream.
 * The length is the number of bytes to send - three per LED.
 */

void ws2812SendArray(uint8_t *array, uint16_t length);
void ws2812SendArrayMask(uint8_t *array, uint16_t length, uint8_t pinMask);

/*
 * Internal defines
 */
#if !defined(CONCAT)
#define CONCAT(a, b)            a ## b
#endif

#if !defined(CONCAT_EXP)
#define CONCAT_EXP(a, b)   CONCAT(a, b)
#endif

#define ws2812PORTREG  CONCAT_EXP(PORT, ws2812Port)
#define ws2812DDRREG   CONCAT_EXP(DDR, ws2812Port)