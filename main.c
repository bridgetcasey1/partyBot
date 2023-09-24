/*
 * partyBot.c
 *
 * Created: 17/09/2021 11:58:37 AM
 * Author : s4319233
 */ 

#include "light_ws2812.h"
#include "serial_communication.h"
#include <avr/io.h>
#include <stdbool.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// Constants
#define F_CPU 8000000 // Clock speed
#define TIMER_1_PERIOD 0.05 // in seconds
#define TIMER_2_PERIOD 0.05 // in seconds
#define AUTO_MODE 0
#define MANUAL_MODE 1
#define THRESHOLD_VALUE 500
#define SIDE_THRESHOLD_VALUE 510
#define MAX_TURN_COUNT 5
#define NUMBER_LEDS 32

#define SLOWEST_SPEED 1
#define SLOW_SPEED 2
#define MID_SPEED 3
#define FAST_SPEED 4
#define FASTEST_SPEED 5

#define RIGHT_SLOWEST_SPEED_PWM 210
#define LEFT_SLOWEST_SPEED_PWM 170
#define RIGHT_SLOW_SPEED_PWM 219
#define LEFT_SLOW_SPEED_PWM 177
#define RIGHT_MID_SPEED_PWM 225
#define LEFT_MID_SPEED_PWM 207
#define RIGHT_FAST_SPEED_PWM 240
#define LEFT_FAST_SPEED_PWM 194
#define RIGHT_FASTEST_SPEED_PWM 255
#define LEFT_FASTEST_SPEED_PWM 207

#define WHITE 0 
#define RED 1
#define ORANGE 2
#define YELLOW 3
#define GREEN 4
#define LIGHTBLUE 5
#define BLUE 6
#define VIOLET 7 
#define RAINBOW 8 

// Global variables 
int current_mode = AUTO_MODE;
int current_speed = MID_SPEED;
bool motors_on = false; 
int solenoid_counter = 0; 
bool fire = false; 
struct cRGB leds[NUMBER_LEDS]; // Array for storing LED colours
int previous_colours[NUMBER_LEDS]; 
bool slow_flash_leds = false; 
bool flash_leds = false; 
bool fast_flash_leds = false; 
bool leds_off = true; 
bool leds_just_flashed = true; 
bool rainbow_boogie = false; 
int led_counter = 0; 
int rainbow_tracker = 0; 
int current_colour = WHITE; 
bool receivingMessage = false; 
char message[50];
int messageLength = 0; 
bool switchOn = false; 

// Auto mode global variables 
bool reversing = false;  
bool turning_left = false;
bool turning_right = false;
bool moving_forward = false;
bool stopped = false;
bool obstacle_detected = false;
bool time_to_reverse = false;
bool time_to_turn = false;
int reversing_counter = 0;
int turning_left_counter = 0;
int turning_right_counter = 0;
int stopped_counter = 0; 

// Manual mode global variables 
bool man_moving_forward = false;
bool man_turning_left = false; 
bool man_turning_right = false; 
bool man_reversing = false; 
bool partyBot_dancing = false; 

// Set up timer 1 with 1s period and interrupt on compare match A.
void initialise_timer_1(void) {
	
	// Set prescalar to 64
	TCCR1B |= (1 << CS11) | (1 << CS10);
	int prescalar = 64;
	
	// Set period to 0.05s (set compare value A)
	OCR1A = TIMER_1_PERIOD * (F_CPU/prescalar) - 1;
	
	// Set counter to 0
	TCNT1 = 0;
	
	// Set interrupt on compare match A
	TIMSK1 |= (1 << OCIE1A);
}

// Set up timer 2 with 500ms period and interrupt on compare match A.
void initialise_timer_2(void) {
	
	// Set the timer mode to CTC
	TCCR2A |= (1 << WGM21);
	
	// Set prescalar to 1024
	TCCR2B |= (1 << CS20) |(1 << CS21) | (1 << CS22);
	int prescalar = 1024;
	
	// Set period to 500ms (set compare value A)
	OCR2A = TIMER_2_PERIOD * (F_CPU/prescalar) - 1;
	
	// Set counter to 0
	TCNT2 = 0;
	
	// Set interrupt on compare match A
	TIMSK2 |= (1 << OCIE2A);
}

// Set up distance sensors 
void setup_distance_sensors(void) {
	
	// Configure sensor pins as inputs
	DDRC &= ~((1 << DDC0) | (1 << DDC1) | (1 << DDC2) | (1 << DDC3));
	
	// ADC prescalar of 128 
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	
	// Configure ADC to use AVCC as reference
	ADMUX |= (1 << REFS0);
	
	// Enable ADC
	ADCSRA |= (1 << ADEN);

	// Enable interrupts (for processing IR sensor data) 
	sei(); 
}

// Set up motor control pins and PWM using timer 0 for speed control.
void setup_motor_control(void) {
	
	// Configure motor A pins as outputs 
	DDRB |= (1 << DDB2);
	DDRD |= (1 << DDD7);
	DDRD |= (1 << DDD6);

	// Configure motor B pins as outputs 
	DDRB |= (1 << DDB0);
	DDRD |= (1 << DDD4);
	DDRD |= (1 << DDD5);

	// Set up non-inverting PWM mode using timer 0 
	TCCR0A |= (1<<COM0A1) | (1<<COM0B1) | (1<<WGM01) | (1<<WGM00);
	TCNT0 = 0;  // Start counter at 0 	
	TCCR0B |= (1<<CS01);  // Turn timer on with prescalar of 8
	
	// Initialise motor speeds at 0  
	OCR0A = 0;  
	OCR0B = 0; 
}

// Read value from rear sensor
uint16_t get_rear_sensor_value(void) {
	
	// Zero ADMUX bits
	ADMUX &= 0xF0;
	
	// Select ADC2 as ADC input
	ADMUX |= (0 << MUX3) | (0 << MUX2) |(1 << MUX1) | (0 << MUX0);
	
	// Start an ADC conversion by setting ADSC bit (bit 6)
	ADCSRA |= (1 << ADSC);
		
	// Wait until the conversion is complete
	while(ADCSRA & (1 << ADSC));
	
	return ADC;
}

// Read value from front sensor
uint16_t get_front_sensor_value(void) {
	
	// Zero ADMUX bits
	ADMUX &= 0xF0;
	
	// select ADC3 as ADC input
	ADMUX |= (0 << MUX3) | (0 << MUX2) |(1 << MUX1) | (1 << MUX0);
	
	// Start an ADC conversion by setting ADSC bit (bit 6)
	ADCSRA |= (1 << ADSC);
	
	// Wait until the conversion is complete
	while(ADCSRA & (1 << ADSC));

	return ADC;
}

// Read value from right sensor
uint16_t get_right_sensor_value(void) {
	
	// Zero ADMUX bits
	ADMUX &= 0xF0;
	
	// select ADC0 as ADC input
	ADMUX |= (0 << MUX3) | (0 << MUX2) |(0 << MUX1) | (0 << MUX0);
	
	// Start an ADC conversion by setting ADSC bit (bit 6)
	ADCSRA |= (1 << ADSC);
	
	// Wait until the conversion is complete
	while(ADCSRA & (1 << ADSC));

	return ADC;
}

// Read value from left sensor
uint16_t get_left_sensor_value(void) {
	
	// Zero ADMUX bits
	ADMUX &= 0xF0;
	
	// select ADC1 as ADC input
	ADMUX |= (0 << MUX3) | (0 << MUX2) |(0 << MUX1) | (1 << MUX0);
	
	// Start an ADC conversion by setting ADSC bit (bit 6)
	ADCSRA |= (1 << ADSC);
	
	// Wait until the conversion is complete
	while(ADCSRA & (1 << ADSC));
	
	return ADC;
}

// Turn motors on by setting enable pins to high 
void turn_motors_on(void) {

	if (current_speed == SLOWEST_SPEED) {
		OCR0A = RIGHT_SLOWEST_SPEED_PWM; 
		OCR0B = LEFT_SLOWEST_SPEED_PWM;
	} else if (current_speed == SLOW_SPEED) {
		OCR0A = RIGHT_SLOW_SPEED_PWM;
		OCR0B = LEFT_SLOW_SPEED_PWM;
	} else if (current_speed == MID_SPEED) {
		OCR0A = RIGHT_MID_SPEED_PWM;
		OCR0B = LEFT_MID_SPEED_PWM;
	} else if (current_speed == FAST_SPEED) {
		OCR0A = RIGHT_FAST_SPEED_PWM;
		OCR0B = LEFT_FAST_SPEED_PWM;
	} else if (current_speed == FASTEST_SPEED) {
		OCR0A = RIGHT_FASTEST_SPEED_PWM;
		OCR0B = LEFT_FASTEST_SPEED_PWM;
	}
	motors_on = true; 
}

// Turn motors off by setting enable pins to low
void turn_motors_off(void) {
	
	// Set motor speeds to 0 
	OCR0A = 0;
	OCR0B = 0;
	
	// Set motor pins to low 
	PORTD &= ~(1 << PIND7); // Set backward motor 1 pin to low
	PORTD &= ~(1 << PIND6); // Set enable motor 1 pin to low
	PORTB &= ~(1 << PINB0); // Set forward motor 2 pin to low
	PORTB &= ~(1 << PINB2); // Set forward motor 1 pin to low
	PORTD &= ~(1 << PIND4); // Set backward motor 2 pin to low
	PORTD &= ~(1 << PIND5); // Set enable motor 2 pin to low

	motors_on = false;
}

// Move robot forward
void move_forward() {
	PORTB |= (1 << PINB2); // Set forward motor 1 pin to high
	PORTD |= (1 << PIND4); // Set forward motor 2 pin to high 
	PORTD &= ~(1 << PIND7); // Set backward motor 1 pin to low
	PORTB &= ~(1 << PINB0); // Set backward motor 2 pin to low
}

// Move robot backwards
void reverse(void) {
	PORTD |= (1 << PIND7); // Set backward motor 1 pin to high
	PORTB |= (1 << PINB0); // Set backward motor 2 pin to high 
	PORTB &= ~(1 << PINB2); // Set forward motor 1 pin to low
	PORTD &= ~(1 << PIND4); // Set forward motor 2 pin to low
} 

// Turn robot left
void turn_left(void) {
	PORTB |= (1 << PINB2); // Set forward motor 1 pin to high
	PORTD &= ~(1 << PIND4); // Set forward motor 2 pin to low
	PORTD &= ~(1 << PIND7); // Set backward motor 1 pin to low
	PORTB |= (1 << PINB0); // Set backward motor 2 pin to high
}

// Turn robot right
void turn_right(void) {
	PORTB &= ~(1 << PINB2); // Set forward motor 1 pin to low
	PORTD |= (1 << PIND4); // Set forward motor 2 pin to high
	PORTD |= (1 << PIND7); // Set backward motor 1 pin to high
	PORTB &= ~(1 << PINB0); // Set backward motor 2 pin to low
}

// Resets auto mode variables when manual mode activated
void reset_auto_variables(void) {
	obstacle_detected = false; 
	moving_forward = false;
	stopped = false; 
	time_to_reverse = false; 
	reversing = false;
	time_to_turn = false; 
	turning_left = false; 
	turning_right = false; 
	stopped_counter = 0;
	reversing_counter = 0;
	turning_left_counter = 0; 
	turning_right_counter = 0; 
}

// Reset manual steering variables 
void reset_man_variables(void) {
	man_moving_forward = false; 
	man_reversing = false; 
	man_turning_left = false; 
	man_turning_right = false; 
	partyBot_dancing = false; 
}

// Set LEDs to red
void set_leds_red(void) {
	const struct cRGB red = {.r = 15, .g = 0, .b = 0};
	for (int i = 0; i < NUMBER_LEDS; i++) {
		leds[i] = red;
	}
	ws2812SetLeds(leds, NUMBER_LEDS);
	leds_off = false;
	current_colour = RED;
}

// Set LEDs to orange
void set_leds_orange(void) {
	const struct cRGB orange = {.r = 20, .g = 4, .b = 0};
	for (int i = 0; i < NUMBER_LEDS; i++) {
		leds[i] = orange;
	}
	ws2812SetLeds(leds, NUMBER_LEDS);
	leds_off = false;
	current_colour = ORANGE;
}

// Set LEDs to yellow
void set_leds_yellow(void) {
	const struct cRGB yellow = {.r = 14, .g = 7, .b = 0};
	for (int i = 0; i < NUMBER_LEDS; i++) {
		leds[i] = yellow;
	}
	ws2812SetLeds(leds, NUMBER_LEDS);
	leds_off = false;
	current_colour = YELLOW;
}

// Set LEDs to green
void set_leds_green(void) {
	const struct cRGB green = {.r = 0, .g = 15, .b = 0};
	for (int i = 0; i < NUMBER_LEDS; i++) {
		leds[i] = green;
	}
	ws2812SetLeds(leds, NUMBER_LEDS);
	leds_off = false;
	current_colour = GREEN;
}

// Set LEDs light blue 
void set_leds_light_blue(void) {
	const struct cRGB light_blue = {.r = 0, .g = 6, .b = 15};
	for (int i = 0; i < NUMBER_LEDS; i++) {
		leds[i] = light_blue;
	}
	ws2812SetLeds(leds, NUMBER_LEDS);
	leds_off = false;
	current_colour = LIGHTBLUE;
}

// Set LEDs to blue
void set_leds_blue(void) {
	const struct cRGB blue = {.r = 0, .g = 0, .b = 15};
	for (int i = 0; i < NUMBER_LEDS; i++) {
		leds[i] = blue;
	}
	ws2812SetLeds(leds, NUMBER_LEDS);
	leds_off = false;
	current_colour = BLUE;
}

// Set LEDs to violet
void set_leds_violet(void) {
	const struct cRGB violet = {.r = 6, .g = 0, .b = 15};
	for (int i = 0; i < NUMBER_LEDS; i++) {
		leds[i] = violet;
	}
	ws2812SetLeds(leds, NUMBER_LEDS);
	leds_off = false;
	current_colour = VIOLET;
}

// Set LEDs to white
void set_leds_white(void) {
	const struct cRGB white = {.r = 15, .g = 15, .b = 15};
	for (int i = 0; i < NUMBER_LEDS; i++) {
		leds[i] = white;
	}
	ws2812SetLeds(leds, NUMBER_LEDS);
	leds_off = false;
	current_colour = WHITE;	
}

// Set LEDs to rainbow
void set_leds_rainbow(void) {
	
	for (int i = 0; i < NUMBER_LEDS; i++) {
		if ((i % 7) == 0) {
			const struct cRGB red = {.r = 15, .g = 0, .b = 0};
			leds[i] = red;
		} else if ((i % 7) == 1) {
			const struct cRGB orange = {.r = 20, .g = 4, .b = 0};
			leds[i] = orange;
		} else if ((i % 7) == 2) {
			const struct cRGB yellow = {.r = 14, .g = 7, .b = 0};
			leds[i] = yellow;
		} else if ((i % 7) == 3) {
			const struct cRGB green = {.r = 0, .g = 15, .b = 0};
			leds[i] = green;
		} else if ((i % 7) == 4) {
			const struct cRGB light_blue = {.r = 0, .g = 6, .b = 15};
			leds[i] = light_blue;
		} else if ((i % 7) == 5) {
			const struct cRGB blue = {.r = 0, .g = 0, .b = 15};
			leds[i] = blue;
		} else if ((i % 7) == 6) {
			const struct cRGB violet = {.r = 6, .g = 0, .b = 15};
			leds[i] = violet;
		}
	}
	ws2812SetLeds(leds, NUMBER_LEDS);
}

// Set LEDs to move along one spot
void set_leds_rainbow_boogie(void) {
	
	for (int i = 0; i < NUMBER_LEDS; i++) {
		if (previous_colours[i] == RED) {
			const struct cRGB orange = {.r = 20, .g = 4, .b = 0};
			leds[i] = orange;
			previous_colours[i] = ORANGE; 
		} else if (previous_colours[i] == ORANGE) {
			const struct cRGB yellow = {.r = 14, .g = 7, .b = 0};
			leds[i] = yellow;
			previous_colours[i] = YELLOW; 
		} else if (previous_colours[i] == YELLOW) {
			const struct cRGB green = {.r = 0, .g = 15, .b = 0};
			leds[i] = green;
			previous_colours[i] = GREEN;
		} else if (previous_colours[i] == GREEN) {
			const struct cRGB light_blue = {.r = 0, .g = 6, .b = 15};
			leds[i] = light_blue;
			previous_colours[i] = LIGHTBLUE;
		} else if (previous_colours[i] == LIGHTBLUE) {
			const struct cRGB blue = {.r = 0, .g = 0, .b = 15};
			leds[i] = blue;
			previous_colours[i] = BLUE; 
		} else if (previous_colours[i] == BLUE) {
			const struct cRGB violet = {.r = 6, .g = 0, .b = 15};
			leds[i] = violet;
			previous_colours[i] = VIOLET;
		} else if (previous_colours[i] == VIOLET) {
			const struct cRGB red = {.r = 15, .g = 0, .b = 0};
			leds[i] = red;
			previous_colours[i] = RED; 
		}
	}
	ws2812SetLeds(leds, NUMBER_LEDS);	
}

// Reset rainbow boogie colour array 
void reset_rainbow_boogie_array(void) {
	
	for (int i = 0; i < NUMBER_LEDS; i++) {
		if ((i % 7) == 0) {
			const struct cRGB red = {.r = 15, .g = 0, .b = 0};
			previous_colours[i] = RED;
			leds[i] = red;
		} else if ((i % 7) == 1) {
			const struct cRGB orange = {.r = 20, .g = 4, .b = 0};
			previous_colours[i] = ORANGE;
			leds[i] = orange; 
		} else if ((i % 7) == 2) {
			const struct cRGB yellow = {.r = 14, .g = 7, .b = 0};
			previous_colours[i] = YELLOW;
			leds[i] = yellow; 
		} else if ((i % 7) == 3) {
			const struct cRGB green = {.r = 0, .g = 15, .b = 0};
			previous_colours[i] = GREEN;
			leds[i] = green; 
		} else if ((i % 7) == 4) {
			const struct cRGB light_blue = {.r = 0, .g = 6, .b = 15};
			previous_colours[i] = LIGHTBLUE;
			leds[i] = light_blue; 
		} else if ((i % 7) == 5) {
			const struct cRGB blue = {.r = 0, .g = 0, .b = 15};
			previous_colours[i] = BLUE;
			leds[i] = blue; 
		} else if ((i % 7) == 6) {
			const struct cRGB violet = {.r = 6, .g = 0, .b = 15};
			previous_colours[i] = VIOLET;
			leds[i] = violet;
		}
	}
	ws2812SetLeds(leds, NUMBER_LEDS);
	rainbow_boogie = true; 
	current_colour = RAINBOW; 
}

// Set LEDs to current colour
void set_leds(void) {
	if (current_colour == RED) {
		set_leds_red();
	} else if (current_colour == ORANGE) {
		set_leds_orange();
	} else if (current_colour == YELLOW) {
		set_leds_yellow();
	} else if (current_colour == GREEN) {
		set_leds_green();
	} else if (current_colour == LIGHTBLUE) {
		set_leds_light_blue();
	} else if (current_colour == BLUE) {
		set_leds_blue();
	} else if (current_colour == VIOLET) {
		set_leds_violet();
	} else if (current_colour == WHITE) {
		set_leds_white();
	} else if (current_colour == RAINBOW) {
		if (rainbow_boogie) {
			set_leds_rainbow_boogie(); 
		} else {
			set_leds_rainbow();
		}
	}
}

// Turn LEDs off
void turnLedsOff(void) {
	
	const struct cRGB off = {.r = 0, .g = 0, .b = 0};
	for (int i = 0; i < NUMBER_LEDS; i++) {
		leds[i] = off;
	}
	ws2812SetLeds(leds, NUMBER_LEDS);
	leds_off = true;
	slow_flash_leds = false;
	flash_leds = false;
	fast_flash_leds = false;
	rainbow_boogie = false;
	leds_just_flashed = true;
	led_counter = 0;
	rainbow_tracker = 0;
	current_colour = WHITE;
}

// Reset partyBot 
void resetPartyBot(void) {
	
	// Reset general variables 
	current_mode = AUTO_MODE;
	current_speed = MID_SPEED;
	motors_on = false;
	solenoid_counter = 0;
	fire = false;
	reset_auto_variables();
	reset_man_variables(); 
	turnLedsOff();	
}

// Interrupt used for firing solenoid and LED control 
ISR(TIMER1_COMPA_vect) {
	if (fire) {
		solenoid_counter++;
	PORTC |= (1 << PINC4);
	//PORTC |= (1 << PINC5); // Set solenoid pin to high
		if (solenoid_counter == 2) {
			PORTC &= ~(1 << PINC4);
			//PORTC &= ~(1 << PINC5); // Set solenoid pin to low
			solenoid_counter = 0;
			fire = false;
		}
	}
	led_counter++; 
	if (slow_flash_leds) {
		if (led_counter == 4) {
			led_counter = 0; 
			if (rainbow_boogie) {
				set_leds(); 
			} else if (leds_just_flashed) {
				leds_just_flashed = false; 
				const struct cRGB off = {.r = 0, .g = 0, .b = 0};
				for (int i = 0; i < NUMBER_LEDS; i++) {
					leds[i] = off;
				}
				ws2812SetLeds(leds, NUMBER_LEDS);
			} else {
				set_leds(); 
				leds_just_flashed = true; 
			}
		}
	} else if (flash_leds) {
		if (led_counter == 2) {
			led_counter = 0;
			if (rainbow_boogie) {
				set_leds();
			} else if (leds_just_flashed) {
				leds_just_flashed = false;
				const struct cRGB off = {.r = 0, .g = 0, .b = 0};
				for (int i = 0; i < NUMBER_LEDS; i++) {
					leds[i] = off;
				}
				ws2812SetLeds(leds, NUMBER_LEDS);
			} else {
				set_leds();
				leds_just_flashed = true; 
			}
		}
	} else if (fast_flash_leds) {
		if (led_counter == 1) {
			led_counter = 0;
			if (rainbow_boogie) {
				set_leds();
			} else if (leds_just_flashed) {
				leds_just_flashed = false;
				const struct cRGB off = {.r = 0, .g = 0, .b = 0};
				for (int i = 0; i < NUMBER_LEDS; i++) {
					leds[i] = off;
				}
				ws2812SetLeds(leds, NUMBER_LEDS);
			} else {
				set_leds();
				leds_just_flashed = true; 
			}
		}
	} 
	
	// Polling switch pin
	if (PIND & (1 << PIND2)) {
		// Pin D2 is high
		if (!switchOn) {
			// Sliding of switch to on position yet to be processed
			switchOn = true;
			resetPartyBot(); 
		}
	} else {
		// Pin D2 is low
		if (switchOn) {
			switchOn = false;
			resetPartyBot();
		}
	}
}

// Interrupt used for processing IR sensor data. 
ISR(TIMER2_COMPA_vect) {
	
	if (current_mode == AUTO_MODE) {
		if (!obstacle_detected) {
			uint16_t front_distance_value = get_front_sensor_value();
			if (front_distance_value < THRESHOLD_VALUE) {
				// Turn motors on if off and move forward
				if (!motors_on) {
					turn_motors_on();
				} 
				if (!moving_forward) {
					move_forward();
					moving_forward = true;
				}
			} else {
				obstacle_detected = true;
				turn_motors_off();
				stopped = true;
				moving_forward = false;
			}
		} else {
			if (stopped) {
				stopped_counter++;
				if (stopped_counter == 7) {
					stopped = false;
					stopped_counter = 0;
					time_to_reverse = true;
				}
			} else if (time_to_reverse) {
				uint16_t rear_distance_value = get_rear_sensor_value();
				if (rear_distance_value < THRESHOLD_VALUE) {
					// Back up
					turn_motors_on();
					reverse();
					reversing = true;
				} else {
					time_to_turn = true;
				}
				time_to_reverse = false;
			} else if (reversing) {
				uint16_t rear_distance_value = get_rear_sensor_value();
				reversing_counter++;
				if (rear_distance_value < THRESHOLD_VALUE) {
					if (reversing_counter == 7) {
						reversing = false;
						reversing_counter = 0;
						time_to_turn = true;
					}
				} else {
					time_to_turn = true;
					reversing = false;
					reversing_counter = 0;
				}
			} else if (time_to_turn) {
				uint16_t left_distance_value = get_left_sensor_value();
				uint16_t right_distance_value = get_right_sensor_value();
				if (left_distance_value <= right_distance_value && left_distance_value < SIDE_THRESHOLD_VALUE) {
					turn_left();
					turning_left = true;
				} else if (right_distance_value < SIDE_THRESHOLD_VALUE) {
					turn_right();
					turning_right = true;
				} else {
					obstacle_detected = false;
				}
				time_to_turn = false;
			} else if (turning_left) {
				turning_left_counter++;
				if (turning_left_counter == MAX_TURN_COUNT) {
					turning_left = false;
					turning_left_counter = 0;
					obstacle_detected = false;
				}
			} else if (turning_right) {
				turning_right_counter++;
				if (turning_right_counter == MAX_TURN_COUNT) {
					turning_right = false;
					turning_right_counter = 0;
					obstacle_detected = false;
				}
			}
		}
	} else {
		if (man_moving_forward) {
			uint16_t front_distance_value = get_front_sensor_value();
			if (front_distance_value < THRESHOLD_VALUE) {
				// Turn motors on if off and move forward
				if (!motors_on) {
					turn_motors_on();
				}
				move_forward();
			}
		} else if (man_reversing) {
			uint16_t rear_distance_value = get_rear_sensor_value();
			if (rear_distance_value < THRESHOLD_VALUE) {
				if (!motors_on) {
					turn_motors_on();
				}
				reverse();
			}
		} else if (man_turning_left) {
			uint16_t left_distance_value = get_left_sensor_value();
			if (left_distance_value < SIDE_THRESHOLD_VALUE) {
				if (!motors_on) {
					turn_motors_on();
				}
				turn_left();
			}
		} else if (man_turning_right) {
			uint16_t right_distance_value = get_right_sensor_value();
			if (right_distance_value < SIDE_THRESHOLD_VALUE) {
				if (!motors_on) {
					turn_motors_on();
				}
				turn_right();
			}
		} else if (partyBot_dancing) {
			
		}
	}
}

// Interrupt handler for receiving UART data.
ISR(USART_RX_vect) {
	
	// Read character
	char receivedCharacter;
	receivedCharacter = UDR0;
	if (receivedCharacter == 85) {
		// 85 == U
		partyBot_dancing = false; 
		if (!man_moving_forward) {
			man_moving_forward = true;
		} else {
			turn_motors_off();
			man_moving_forward = false;
		}
	} else if (receivedCharacter == 76) {
		// 76 == L
		partyBot_dancing = false; 
		if (!man_turning_left) {
			man_turning_left = true; 
		} else {
			turn_motors_off();
			man_turning_left = false;
		}
	} else if (receivedCharacter == 82) {
		// 82 == R
		partyBot_dancing = false; 
		if (!man_turning_right) {
			man_turning_right = true;
		} else {
			turn_motors_off(); 
			man_turning_right = false; 
		}
	} else if (receivedCharacter == 68) {
		// 68 == D
		partyBot_dancing = false; 
		if (!man_reversing) {
			man_reversing = true; 
		} else {
			turn_motors_off();
			man_reversing = false; 
		}
	} else if (receivedCharacter == 65) {
		// 65 == A
		if (current_mode == MANUAL_MODE) {
			reset_man_variables();
			current_mode = AUTO_MODE;
		}
	} else if (receivedCharacter == 77) {
		// 77 = M
		if (current_mode == AUTO_MODE) {
			turn_motors_off();
			reset_auto_variables(); 
			current_mode = MANUAL_MODE;
		}
	} else if (receivedCharacter == 64) {
		// 64 = @ (reset steering)
		reset_man_variables(); 
	} else if (receivedCharacter == 83) {
		// 83 = S
		current_speed = SLOWEST_SPEED; 
		turn_motors_on();
	} else if (receivedCharacter == 84) {
		// 84 = T
		current_speed = SLOW_SPEED;
		turn_motors_on();
	} else if (receivedCharacter == 88) {
		// 88 = X
		current_speed = MID_SPEED;
		turn_motors_on(); 
	} else if (receivedCharacter == 90) {
		// Z = 90 
		current_speed = FAST_SPEED; 
		turn_motors_on(); 
	} else if (receivedCharacter == 33) {
		// ! = 33
		current_speed = FASTEST_SPEED; 
		turn_motors_on(); 
	} else if (receivedCharacter == 70) {
		// F = 70
		receivingMessage = true;
		// Initialise message array 
		for(int i = 0; i < sizeof(message); i++) {
			message[i] = 0; // Initialising each element separately
		}
		messageLength = 0;
	} else if (receivedCharacter == 71) {
		// G = 71
		fire = true; 
		solenoid_counter = 0;
	} else if (receivedCharacter == 66) {
		// B = 66
		set_leds_blue();
		if (rainbow_boogie) {
			rainbow_boogie = false; 
			slow_flash_leds = false;
			flash_leds = false;
			fast_flash_leds = false;
		} 
	} else if (receivedCharacter == 67) {
		// C = 67
		// Set LEDs to rainbow 
		set_leds_rainbow(); 
		leds_off = false;
		current_colour = RAINBOW; 
		if (rainbow_boogie) {
			rainbow_boogie = false;
			slow_flash_leds = false;
			flash_leds = false;
			fast_flash_leds = false;
		}
	} else if (receivedCharacter == 72) {
		// H = 72
		set_leds_green(); 
		if (rainbow_boogie) {
			rainbow_boogie = false;
			slow_flash_leds = false;
			flash_leds = false;
			fast_flash_leds = false;
		}
	} else if (receivedCharacter == 78) {
		// N = 78
		set_leds_light_blue();  
		if (rainbow_boogie) {
			rainbow_boogie = false;
			slow_flash_leds = false;
			flash_leds = false;
			fast_flash_leds = false;
		}
	} else if (receivedCharacter == 79) {
		// O = 79
		set_leds_orange(); 
		if (rainbow_boogie) {
			rainbow_boogie = false;
			slow_flash_leds = false;
			flash_leds = false;
			fast_flash_leds = false;
		}
	} else if (receivedCharacter == 81) {
		// Q = 81
		set_leds_red();
		if (rainbow_boogie) {
			rainbow_boogie = false;
			slow_flash_leds = false;
			flash_leds = false;
			fast_flash_leds = false;
		}
	} else if (receivedCharacter == 86) {
		// V = 86
		set_leds_violet(); 
		if (rainbow_boogie) {
			rainbow_boogie = false;
			slow_flash_leds = false;
			flash_leds = false;
			fast_flash_leds = false;
		}
	} else if (receivedCharacter == 87) {
		// W = 87
		set_leds_white(); 
		if (rainbow_boogie) {
			rainbow_boogie = false;
			slow_flash_leds = false;
			flash_leds = false;
			fast_flash_leds = false;
		}
	} else if (receivedCharacter == 89) {
		// Y = 89
		set_leds_yellow(); 
		if (rainbow_boogie) {
			rainbow_boogie = false;
			slow_flash_leds = false;
			flash_leds = false;
			fast_flash_leds = false;
		}
	} else if (receivedCharacter == 69) {
		// E = 69
		// Turn LEDs off 
		const struct cRGB off = {.r = 0, .g = 0, .b = 0};
		for (int i = 0; i < NUMBER_LEDS; i++) {
			leds[i] = off;
		}
		ws2812SetLeds(leds, NUMBER_LEDS);
		leds_off = true; 
		slow_flash_leds = false; 
		flash_leds = false; 
		fast_flash_leds = false; 
		rainbow_boogie = false; 
	} else if (receivedCharacter == 73) {
		// I = 73
		set_leds_white();
		if (rainbow_boogie) {
			rainbow_boogie = false;
			slow_flash_leds = false;
			flash_leds = false;
			fast_flash_leds = false;
		}
	} else if (receivedCharacter == 74) {
		// J = 74
		if (rainbow_boogie) {
			rainbow_boogie = false;
			slow_flash_leds = false;
			flash_leds = false;
			fast_flash_leds = false;
			set_leds_rainbow();
			leds_off = false;
			current_colour = RAINBOW;
		}
		if (!leds_off) {
			slow_flash_leds = true;
			leds_just_flashed = true; 
		}
		if (flash_leds) {
			flash_leds = false; 
		}
		if (fast_flash_leds) {
			fast_flash_leds = false; 
		}
		led_counter = 0; 
	} else if (receivedCharacter == 75) {
		// K = 75
		if (rainbow_boogie) {
			rainbow_boogie = false;
			slow_flash_leds = false;
			flash_leds = false;
			fast_flash_leds = false;
			set_leds_rainbow();
			leds_off = false;
			current_colour = RAINBOW;
		}
		if (!leds_off) {
			flash_leds = true;
			leds_just_flashed = true; 
		}
		if (slow_flash_leds) {
			slow_flash_leds = false;
		}
		if (fast_flash_leds) {
			fast_flash_leds = false;
		}
		led_counter = 0; 
	} else if (receivedCharacter == 80) {
		// P = 80
		if (rainbow_boogie) {
			rainbow_boogie = false;
			slow_flash_leds = false;
			flash_leds = false;
			fast_flash_leds = false;
			set_leds_rainbow();
			leds_off = false;
			current_colour = RAINBOW;
		}
		if (!leds_off) {
			fast_flash_leds = true;
			leds_just_flashed = true; 
		}
		if (slow_flash_leds) {
			slow_flash_leds = false;
		}
		if (flash_leds) {
			flash_leds = false;
		}
		led_counter = 0; 
	} else if (receivedCharacter == 60) {
		// 60 = < (slow rainbow boogie) 
		slow_flash_leds = true;
		flash_leds = false;
		fast_flash_leds = false;
		rainbow_boogie = true;
		reset_rainbow_boogie_array(); 
		led_counter = 0; 
	} else if (receivedCharacter == 63) {
		// 63 = ? (rainbow boogie)
		slow_flash_leds = false;
		flash_leds = true;
		fast_flash_leds = false;
		rainbow_boogie = true;
		reset_rainbow_boogie_array(); 
		led_counter = 0; 
	} else if (receivedCharacter == 62) {
		// 62 = > (fast rainbow boogie)
		slow_flash_leds = false;
		flash_leds = false;
		fast_flash_leds = true;
		rainbow_boogie = true;
		reset_rainbow_boogie_array(); 
		led_counter = 0; 
	} else if (receivedCharacter == 43) {
		// 43 = + (stop flash)
		slow_flash_leds = false;
		flash_leds = false;
		fast_flash_leds = false;
		rainbow_boogie = false;
		set_leds(); 
	} else if (receivedCharacter == 46) {
		// 46 = .
		reset_man_variables();
		turnLedsOff();
		if (partyBot_dancing) {
			partyBot_dancing = false; 
		} else {
			partyBot_dancing = true; 
		}
	}
}

int main(void) {
	
	// Set-up distance sensors, timer 1, timer 2 and motors
	setup_distance_sensors(); 
	setup_motor_control(); 
	initialise_timer_1(); 
	initialise_timer_2(); 
	
	// Initialise serial communication
	initialise_serial_communication();
	
	// TEST
	DDRD |= (1 << DDD3); 
		
	// Set pin B1 to be an output for LEDs
	DDRB |= (1 << DDB1);
	
	// Set pin C4 to be an output for solenoid control
	//DDRC |= (1 << DDC5); 
	DDRC |= (1 << DDC4); 
	
	// Set pin D2 to be an input for switch 
	DDRD &= ~(1 << DDD2); 
	
    while (1) {		
    }
}