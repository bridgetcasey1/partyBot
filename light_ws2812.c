/*
* light weight WS2812 lib V2.0b
*
* Controls WS2811/WS2812/WS2812B RGB-LEDs
* Author: Tim (cpldcpu@gmail.com)
*
* Jan 18th, 2014  v2.0b Initial Version
* Nov 29th, 2015  v2.3  Added SK6812RGBW support
*
* License: GNU GPL v2+ (see License.txt)
*/

#include "light_ws2812.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

// Normally ws2812SendArrayMask() runs under disabled-interrupt condition,
// undefine if you want to accept interrupts in that function.
#define interruptIsDisabled
 
// Set leds for standard RGB.
void inline ws2812SetLeds(struct cRGB *ledArray, uint16_t leds) 
{
    ws2812SetLedsPin(ledArray, leds, _BV(ws2812Pin));
}

void inline ws2812SetLedsPin(struct cRGB *ledArray, uint16_t leds, 
		uint8_t pinMask) 
{
    ws2812SendArrayMask((uint8_t*)ledArray,leds + leds + leds, pinMask);
    _delay_us(ws2812ResetTime);
}

// Set leds for SK6812RGBW.
void inline ws2812SetLedsRgbw(struct cRGBW *ledArray, uint16_t leds)
{
    ws2812SendArrayMask((uint8_t*)ledArray, leds << 2, _BV(ws2812Pin));
    _delay_us(ws2812ResetTime);
}

void ws2812SendArray(uint8_t *data, uint16_t dataLength)
{
    ws2812SendArrayMask(data, dataLength, _BV(ws2812Pin));
}

// This routine writes an array of bytes with RGB values to the Dataout pin
// using the fast 800kHz clockless WS2811/2812 protocol.

// Timing in ns:
#define wZeroPulse 350
#define wOnePulse 900
#define wTotalPeriod 1250

// Fixed cycles used by the inner loop.
#define wFixedLow 2
#define wFixedHigh 4
#define wFixedTotal 8   

// Insert NOPs to match the timing, if possible.
#define wZeroCycles (((F_CPU / 1000) * wZeroPulse) / 1000000)
#define wOneCycles (((F_CPU / 1000) * wOnePulse + 500000) / 1000000)
#define wTotalCycles (((F_CPU / 1000) * wTotalPeriod + 500000) / 1000000)

// w1 - nops between rising edge and falling edge - low.
#define w1 (wZeroCycles-wFixedLow)
// w2 - nops between fe low and fe high.
#define w2 (wOneCycles-wFixedHigh - w1)
// w3 - nops to complete loop.
#define w3 (wTotalCycles-wFixedTotal - w1 - w2)

#if w1 > 0
    #define w1Nops w1
#else
    #define w1Nops 0
#endif

// The only critical timing parameter is the minimum pulse length of the "0"
// Warn or throw error if this timing can not be met with current F_CPU 
// settings.
#define wLowTime ((w1Nops + wFixedLow) * 1000000) / (F_CPU / 1000)
#if wLowTime > 550
    #error "LightWs2812: Sorry, the clock speed is too low. Did you set F_CPU correctly?"
#elif wLowTime > 450
    #warning "LightWs2812: The timing is critical and may only work on WS2812B, not on WS2812(S)."
    #warning "Please consider a higher clockspeed, if possible"
#endif   

#if w2 > 0
#define w2Nops w2
#else
#define w2Nops 0
#endif

#if w3 > 0
#define w3Nops w3
#else
#define w3Nops 0
#endif

#define wNop1  "nop      \n\t"
#ifdef interruptIsDisabled
#define wNop2  "brid .+0 \n\t"
#else
#define wNop2  "brtc .+0 \n\t"
#endif
#define wNop4  wNop2 wNop2
#define wNop8  wNop4 wNop4
#define wNop16 wNop8 wNop8

void inline ws2812SendArrayMask(uint8_t *data, uint16_t dataLength, uint8_t maskHigh)
{
    uint8_t currentByte, ctr, maskLow;
    uint8_t sregPrev;
  
    ws2812DDRREG |= maskHigh; // Enable output
  
    maskLow =~ maskHigh & ws2812PORTREG;
    maskHigh |= ws2812PORTREG;
  
    sregPrev=SREG;
	
#ifdef interruptIsDisabled
    cli();  
#endif  

    while (dataLength--) {
		
        currentByte = *data++;
    
        asm volatile(
        "       ldi   %0,8  \n\t"
        #ifndef interruptIsDisabled
        "       clt         \n\t"
        #endif
        "loop%=:            \n\t"
        "       out   %2,%3 \n\t" //  '1' [01] '0' [01] - re
        #if (w1Nops & 1)
        wNop1
        #endif
        #if (w1Nops & 2)
        wNop2
        #endif
        #if (w1Nops & 4)
        wNop4
        #endif
        #if (w1Nops & 8)
        wNop8
        #endif
        #if (w1Nops & 16)
        wNop16
        #endif
        "       sbrs  %1,7  \n\t" // '1' [03] '0' [02]
	    "       out   %2,%4 \n\t" // '1' [--] '0' [03] - fe-low
        "       lsl   %1    \n\t" // '1' [04] '0' [04]
        #if (w2Nops & 1)
        wNop1
        #endif
        #if (w2Nops & 2)
        wNop2
        #endif
        #if (w2Nops & 4)
        wNop4
        #endif
        #if (w2Nops & 8)
        wNop8
        #endif
        #if (w2Nops & 16)
        wNop16 
        #endif
        "       out   %2,%4 \n\t" // '1' [+1] '0' [+1] - fe-high
        #if (w3Nops & 1)
        wNop1
        #endif
        #if (w3Nops & 2)
        wNop2
        #endif
        #if (w3Nops & 4)
        wNop4
        #endif
        #if (w3Nops & 8)
        wNop8
        #endif
        #if (w3Nops & 16)
        wNop16
        #endif

        "       dec   %0    \n\t" // '1' [+2] '0' [+2]
        "       brne  loop%=\n\t" // '1' [+3] '0' [+4]
        :	"=&d" (ctr)
        :	"r" (currentByte), "I" (_SFR_IO_ADDR(ws2812PORTREG)), "r" (maskHigh), "r" (maskLow)
        );
    }
  
    SREG=sregPrev;
}