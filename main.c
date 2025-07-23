/*
 * partyBot.c
 *
 * Created: 06/01/2025 03:56:27 PM
 * Author : gatorBots
 */ 

#include "light_ws2812.h"
#include "serial_communication.h"
#include "macros.h"
#include <avr/io.h>
#include <stdbool.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define F_CPU 8000000 // clock speed

// Movement variables
int current_mode = AUTO_MODE;
int current_speed = MID_SPEED;
int movement_mode = STOPPED;
int reversing_counter = 0;
int turning_counter = 0;
int stopped_counter = 0;
bool forward_mvmt_disabled = false;
bool obstacle_just_detected = false; 
bool partyBot_switched_off = false;

// Solenoid variables
int solenoid_counter = 0;
bool fire_solenoid = false;

// LED variables
struct cRGB leds[NUMBER_LEDS]; // Array for storing LED colours
struct cRGB ring_leds[NUM_RING_LEDS];
int previous_led_colours[NUMBER_LEDS]; // For rainbow boogie
int previous_ring_led_colours[NUM_RING_LEDS];
int led_mode = OFF;
int current_led_colour = NO_COLOUR;
int pre_obstacle_led_mode = OFF;
int pre_obstacle_led_colour = NO_COLOUR;
bool leds_just_flashed = false;
int led_counter = 0;
bool lcd_in_sync = true; 

const struct cRGB off = {.r = 0, .g = 0, .b = 0};
const struct cRGB white = {.r = 15, .g = 15, .b = 15};
const struct cRGB red = {.r = 15, .g = 0, .b = 0};
const struct cRGB orange = {.r = 20, .g = 4, .b = 0};
const struct cRGB yellow = {.r = 14, .g = 7, .b = 0};
const struct cRGB green = {.r = 0, .g = 15, .b = 0};
const struct cRGB light_blue = {.r = 0, .g = 6, .b = 15};
const struct cRGB blue = {.r = 0, .g = 0, .b = 15};
const struct cRGB violet = {.r = 6, .g = 0, .b = 15};
const struct cRGB pink = {.r = 15, .g = 5, .b = 3};

// LCD message variables
bool receiving_message = false;
char message[50];
int message_length = 0;

// Set up timer 1 with interrupt on compare match A
void init_timer_1(void) {

	// Set the timer mode to CTC
	TCCR1B |= (1 << WGM12);
	
	// Set prescalar to 64
	TCCR1B |= (1 << CS11) | (1 << CS10);
	int prescalar = 64;
	
	// Set period (compare value A)
	OCR1A = TIMER_1_PERIOD * (F_CPU/prescalar) - 1;
	
	// Set counter to 0
	TCNT1 = 0;
	
	// Set interrupt on compare match A
	TIMSK1 |= (1 << OCIE1A);

	// Enable interrupts
	sei(); 
}

// Set up timer 2 with interrupt on compare match A
void init_timer_2(void) {
	
	// Set the timer mode to CTC
	TCCR2A |= (1 << WGM21);
	
	// Set prescalar to 1024
	TCCR2B |= (1 << CS20) |(1 << CS21) | (1 << CS22);
	int prescalar = 1024;
	
	// Set period (compare value A)
	OCR2A = TIMER_2_PERIOD * F_CPU / (prescalar * 1000) - 1;
	
	// Set counter to 0
	TCNT2 = 0;
	
	// Set interrupt on compare match A
	TIMSK2 |= (1 << OCIE2A);
}

// Set up distance sensors 
void init_distance_sensors(void) {
	
	// Configure ADC to use AVCC as reference
	ADMUX = (1 << REFS0);
	
	// Configure for 8 bit results
	ADMUX |= (1 << ADLAR);
	
	// Set ADC prescalar to 128 
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	
	// Enable ADC
	ADCSRA |= (1 << ADEN);
}

// Set up motor control pins and PWM using timer 0 for speed control
void init_motor_control(void) {
	
	// Configure motor 1 pins as outputs 
	M1_DDR |= (1 << M1_EN_PIN);
	M1_DDR |= (1 << M1_FWD_PIN);
	M1_DDR |= (1 << M1_BWD_PIN);

	// Configure motor 2 pins as outputs 
	M2_DDR |= (1 << M2_EN_PIN);
	M2_DDR |= (1 << M2_FWD_PIN);
	M2_BWD_DDR |= (1 << M2_BWD_PIN);

	// Set up non-inverting PWM mode using timer 0 
	TCCR0A |= (1<<COM0A1) | (1<<COM0B1) | (1<<WGM01) | (1<<WGM00);
	TCNT0 = 0;  // Start counter at 0 	
	TCCR0B |= (1<<CS01);  // Turn timer on with prescalar of 8
	
	// Initialise motor speeds at 0  
	OCR0A = 0;  
	OCR0B = 0; 
}

// Set LEDs to rainbow
void set_leds_rainbow(void) {
	
	// Setting LED strip
	for (int led = 0; led < NUMBER_LEDS; led++) {
		if ((led % NUM_COLOURS) == RED) {
			leds[led] = red;
		} else if ((led % NUM_COLOURS) == ORANGE) {
			leds[led] = orange;
		} else if ((led % NUM_COLOURS) == YELLOW) {
			leds[led] = yellow;
		} else if ((led % NUM_COLOURS) == GREEN) {
			leds[led] = green;
		} else if ((led % NUM_COLOURS) == LIGHTBLUE) {
			leds[led] = light_blue;
		} else if ((led % NUM_COLOURS) == BLUE) {
			leds[led] = blue;
		} else if ((led % NUM_COLOURS) == VIOLET) {
			leds[led] = violet;
		} else if ((led % NUM_COLOURS) == PINK) {
			leds[led] = pink;
		}
	}
	ws2812SetLeds(leds, NUMBER_LEDS);
	
	// Setting LED rings
	for (int led = 0; led < NUM_RING_LEDS; led++) {
		if ((led % NUM_COLOURS) == RED) {
			ring_leds[led] = red;
		} else if ((led % NUM_COLOURS) == ORANGE) {
			ring_leds[led] = orange;
		} else if ((led % NUM_COLOURS) == YELLOW) {
			ring_leds[led] = yellow;
		} else if ((led % NUM_COLOURS) == GREEN) {
			ring_leds[led] = green;
		} else if ((led % NUM_COLOURS) == LIGHTBLUE) {
			ring_leds[led] = light_blue;
		} else if ((led % NUM_COLOURS) == BLUE) {
			ring_leds[led] = blue;
		} else if ((led % NUM_COLOURS) == VIOLET) {
			ring_leds[led] = violet;
		} else if ((led % NUM_COLOURS) == PINK) {
			ring_leds[led] = pink;
		}
	}
	ws2812SetLedsPin(ring_leds, NUM_RING_LEDS, _BV(ws2812Pin2));
}

// Set LEDs to move along one spot
void set_leds_rainbow_boogie(void) {
	
	// Setting LED strip
	for (int led = 0; led < NUMBER_LEDS; led++) {
		if (previous_led_colours[led] == RED) {
			leds[led] = orange;
			previous_led_colours[led] = ORANGE;
		} else if (previous_led_colours[led] == ORANGE) {
			leds[led] = yellow;
			previous_led_colours[led] = YELLOW;
		} else if (previous_led_colours[led] == YELLOW) {
			leds[led] = green;
			previous_led_colours[led] = GREEN;
		} else if (previous_led_colours[led] == GREEN) {
			leds[led] = light_blue;
			previous_led_colours[led] = LIGHTBLUE;
		} else if (previous_led_colours[led] == LIGHTBLUE) {
			leds[led] = blue;
			previous_led_colours[led] = BLUE;
		} else if (previous_led_colours[led] == BLUE) {
			leds[led] = violet;
			previous_led_colours[led] = VIOLET;
		} else if (previous_led_colours[led] == VIOLET) {
			leds[led] = pink;
			previous_led_colours[led] = PINK;
		} else if (previous_led_colours[led] == PINK) {
			leds[led] = red;
			previous_led_colours[led] = RED;
		}
	}
	ws2812SetLeds(leds, NUMBER_LEDS);
	
	// Setting LED rings
	for (int led = 0; led < NUM_RING_LEDS; led++) {
		if (previous_ring_led_colours[led] == RED) {
			ring_leds[led] = orange;
			previous_ring_led_colours[led] = ORANGE;
		} else if (previous_ring_led_colours[led] == ORANGE) {
			ring_leds[led] = yellow;
			previous_ring_led_colours[led] = YELLOW;
		} else if (previous_ring_led_colours[led] == YELLOW) {
			ring_leds[led] = green;
			previous_ring_led_colours[led] = GREEN;
		} else if (previous_ring_led_colours[led] == GREEN) {
			ring_leds[led] = light_blue;
			previous_ring_led_colours[led] = LIGHTBLUE;
		} else if (previous_ring_led_colours[led] == LIGHTBLUE) {
			ring_leds[led] = blue;
			previous_ring_led_colours[led] = BLUE;
		} else if (previous_ring_led_colours[led] == BLUE) {
			ring_leds[led] = violet;
			previous_ring_led_colours[led] = VIOLET;
		} else if (previous_ring_led_colours[led] == VIOLET) {
			ring_leds[led] = pink;
			previous_ring_led_colours[led] = PINK;
		} else if (previous_ring_led_colours[led] == PINK) {
			ring_leds[led] = red;
			previous_ring_led_colours[led] = RED;
		}
	}
	ws2812SetLedsPin(ring_leds, NUM_RING_LEDS, _BV(ws2812Pin2));
}

// Reset rainbow boogie colour array
void reset_rainbow_boogie_array(void) {
	
	// Resetting LED strip
	for (int led = 0; led < NUMBER_LEDS; led++) {
		if ((led % NUM_COLOURS) == RED) {
			previous_led_colours[led] = RED;
			leds[led] = red;
		} else if ((led % NUM_COLOURS) == ORANGE) {
			previous_led_colours[led] = ORANGE;
			leds[led] = orange;
		} else if ((led % NUM_COLOURS) == YELLOW) {
			previous_led_colours[led] = YELLOW;
			leds[led] = yellow;
		} else if ((led % NUM_COLOURS) == GREEN) {
			previous_led_colours[led] = GREEN;
			leds[led] = green;
		} else if ((led % NUM_COLOURS) == LIGHTBLUE) {
			previous_led_colours[led] = LIGHTBLUE;
			leds[led] = light_blue;
		} else if ((led % NUM_COLOURS) == BLUE) {
			previous_led_colours[led] = BLUE;
			leds[led] = blue;
		} else if ((led % NUM_COLOURS) == VIOLET) {
			previous_led_colours[led] = VIOLET;
			leds[led] = violet;
		} else if ((led % NUM_COLOURS) == PINK) {
			previous_led_colours[led] = PINK;
			leds[led] = pink;
		}
	}
	ws2812SetLeds(leds, NUMBER_LEDS);
	
	// Resetting LED rings
	for (int led = 0; led < NUM_RING_LEDS; led++) {
		if ((led % NUM_COLOURS) == RED) {
			previous_ring_led_colours[led] = RED;
			ring_leds[led] = red;
		} else if ((led % NUM_COLOURS) == ORANGE) {
			previous_ring_led_colours[led] = ORANGE;
			ring_leds[led] = orange;
		} else if ((led % NUM_COLOURS) == YELLOW) {
			previous_ring_led_colours[led] = YELLOW;
			ring_leds[led] = yellow;
		} else if ((led % NUM_COLOURS) == GREEN) {
			previous_ring_led_colours[led] = GREEN;
			ring_leds[led] = green;
		} else if ((led % NUM_COLOURS) == LIGHTBLUE) {
			previous_ring_led_colours[led] = LIGHTBLUE;
			ring_leds[led] = light_blue;
		} else if ((led % NUM_COLOURS) == BLUE) {
			previous_ring_led_colours[led] = BLUE;
			ring_leds[led] = blue;
		} else if ((led % NUM_COLOURS) == VIOLET) {
			previous_ring_led_colours[led] = VIOLET;
			ring_leds[led] = violet;
		} else if ((led % NUM_COLOURS) == PINK) {
			previous_ring_led_colours[led] = PINK;
			ring_leds[led] = pink;
		}
	}
	ws2812SetLedsPin(ring_leds, NUM_RING_LEDS, _BV(ws2812Pin2));
	
	led_counter = 0;
	current_led_colour = RAINBOW;
}

// Turn LEDs off
void turn_leds_off(void) {
	
	// Turning strip LEDs off
	for (int led = 0; led < NUMBER_LEDS; led++) {
		leds[led] = off;
	}
	ws2812SetLeds(leds, NUMBER_LEDS);
	
	// Turning ring LEDs off
	for (int led = 0; led < NUM_RING_LEDS; led++) {
		ring_leds[led] = off;
	}
	ws2812SetLedsPin(ring_leds, NUM_RING_LEDS, _BV(ws2812Pin2));

	led_mode = OFF;
	current_led_colour = NO_COLOUR;
}

// Set LEDs to the current colour
void set_leds(void) {
	
	if (led_mode == BOOGIE_SLOW || led_mode == BOOGIE || led_mode == BOOGIE_SLOW) {
		set_leds_rainbow_boogie();
	} else {
		struct cRGB colour_to_be_set = red;
		switch (current_led_colour) {
			
			case ORANGE:
			colour_to_be_set = orange;
			break;
			
			case YELLOW:
			colour_to_be_set = yellow;
			break;
			
			case GREEN:
			colour_to_be_set = green;
			break;
			
			case LIGHTBLUE:
			colour_to_be_set = light_blue;
			break;
			
			case BLUE:
			colour_to_be_set = blue;
			break;
			
			case VIOLET:
			colour_to_be_set = violet;
			break;
			
			case PINK:
			colour_to_be_set = pink;
			break;
			
			case WHITE:
			colour_to_be_set = white;
			break;
			
			case RAINBOW:
			set_leds_rainbow();
			break;
		}
		
		// Setting strip LEDs
		if (current_led_colour != RAINBOW) {
			for (int led = 0; led < NUMBER_LEDS; led++) {
				leds[led] = colour_to_be_set;
			}
			ws2812SetLeds(leds, NUMBER_LEDS);
		}
		
		// Setting ring LEDs
		if (current_led_colour != RAINBOW) {
			for (int led = 0; led < NUM_RING_LEDS; led++) {
				ring_leds[led] = colour_to_be_set;
			}
			ws2812SetLedsPin(ring_leds, NUM_RING_LEDS, _BV(ws2812Pin2));
		}
	}
}

// Initialise partyBot
void init_partyBot(void) {

	// Initialise timers
	init_timer_1();
	init_timer_2();

	// Initialise distance sensors
	init_distance_sensors();

	// Initliase motors
	init_motor_control();

	// Initialise serial communication
	init_serial_communication();

	// Set test LED pin as an output
	TEST_LED_DDR |= (1 << TEST_LED_PIN);
	
	// Set LED strip pin as an output
	LED_DDR |= (1 << LED_PIN);

	// Set ring LED pin as an output
	RING_LED_DDR |= (1 << RING_LED_PIN);

	// Set solenoid pin as an output
	SOLENOID_DDR |= (1 << SOLENOID_PIN);
	
	// Turn LEDs green
	current_led_colour = GREEN;
	set_leds();
	
	// Set up touch sensor interrupts (PCINT6 and PCINT7)
	PCMSK0 |= (1 << PCINT7) | (1 << PCINT6);
	PCICR |= (1 << PCIE0);
	
	// Start in auto mode and at mid speed
	//current_mode = AUTO_MODE;
	current_mode = MANUAL_MODE;
	send_serial_char(CHANGE_AUTO_MODE);
	current_speed = MID_SPEED;
}

// Read value of given distance sensor
uint8_t get_distance_sensor_value(int sensor) {

	// Zero ADMUX (lower four) bits
	ADMUX &= 0xF0;

	switch (sensor) {
		case FRONT_SENSOR:
			// Select ADC2 as ADC input
			ADMUX |= (1 << MUX1);
			break;
		case REAR_SENSOR: 
			// Select ADC1 as ADC input
			ADMUX |= (1 << MUX0);
			break; 
		case LEFT_SENSOR: 
			// Select ADC0 as ADC input - no change required to ADMUX register
			break;
		default: 
			// Select ADC3 as ADC input
			ADMUX |= (1 << MUX1) | (1 << MUX0);
			break;
	}

	// Start an ADC conversion by setting ADSC bit
	ADCSRA |= (1 << ADSC);
	
	// Wait until the conversion is complete
	while(ADCSRA & (1 << ADSC)) {}
		
	return ADCH;
}

// Turn motors on according to current speed
void turn_motors_on(void) {

	if (current_speed == SLOWEST_SPEED) {
		OCR0A = M1_SLOWEST_PWM; 
		OCR0B = M2_SLOWEST_PWM;
	} else if (current_speed == SLOW_SPEED) {
		OCR0A = M1_SLOW_PWM;
		OCR0B = M2_SLOW_PWM;
	} else if (current_speed == MID_SPEED) {
		OCR0A = M1_MID_PWM;
		OCR0B = M2_MID_PWM;
	} else if (current_speed == FAST_SPEED) {
		OCR0A = M1_FAST_PWM;
		OCR0B = M2_FAST_PWM;
	} else if (current_speed == FASTEST_SPEED) {
		OCR0A = M1_FASTEST_PWM;
		OCR0B = M2_FASTEST_PWM;
	}
}

// Turn motors off by setting motor pins to low
void turn_motors_off(void) {
	
	// Set motor speeds to 0 
	OCR0A = 0;
	OCR0B = 0;
	
	// Set motor pins to low 
	M1_PORT &= ~(1 << M1_EN_PIN);
	M1_PORT &= ~(1 << M1_FWD_PIN); 
	M1_PORT &= ~(1 << M1_BWD_PIN);
	M2_PORT &= ~(1 << M2_EN_PIN);
	M2_PORT &= ~(1 << M2_FWD_PIN);  
	M2_BWD_PORT &= ~(1 << M2_BWD_PIN);

	movement_mode = STOPPED;
}

// Move robot forward
void move_forward() {
	M1_PORT |= (1 << M1_FWD_PIN);  // Set forward motor 1 pin to high
	M2_PORT |= (1 << M2_FWD_PIN);  // Set forward motor 2 pin to high 
	M1_PORT &= ~(1 << M1_BWD_PIN);  // Set backward motor 1 pin to low
	M2_BWD_PORT &= ~(1 << M2_BWD_PIN);  // Set backward motor 2 pin to low
	
	movement_mode = FORWARD;
}

// Move robot backwards
void reverse(void) {
	M1_PORT |= (1 << M1_BWD_PIN); // Set backward motor 1 pin to high
	M2_BWD_PORT |= (1 << M2_BWD_PIN); // Set backward motor 2 pin to high 
	M1_PORT &= ~(1 << M1_FWD_PIN); // Set forward motor 1 pin to low
	M2_PORT &= ~(1 << M2_FWD_PIN); // Set forward motor 2 pin to low
	
	movement_mode = REVERSING;
} 

// Turn robot left
void turn_left(void) {
	M1_PORT |= (1 << M1_FWD_PIN); // Set forward motor 1 pin to high
	M2_BWD_PORT |= (1 << M2_BWD_PIN); // Set backward motor 2 pin to high
	M1_PORT &= ~(1 << M1_BWD_PIN); // Set backward motor 1 pin to low
	M2_PORT &= ~(1 << M2_FWD_PIN); // Set forward motor 2 pin to low
	
	movement_mode = TURNING_L;
}

// Turn robot right
void turn_right(void) {
	M1_PORT |= (1 << M1_BWD_PIN); // Set backward motor 1 pin to high
	M2_PORT |= (1 << M2_FWD_PIN); // Set forward motor 2 pin to high
	M1_PORT &= ~(1 << M1_FWD_PIN); // Set forward motor 1 pin to low
	M2_BWD_PORT &= ~(1 << M2_BWD_PIN); // Set backward motor 2 pin to low
	
	movement_mode = TURNING_R;
}

// Reset auto mode variables when manual mode activated
void reset_auto_variables(void) {
	movement_mode = STOPPED; 
	reversing_counter = 0;
	turning_counter = 0;
	stopped_counter = 0;
	forward_mvmt_disabled = false;
	obstacle_just_detected = false;
}

// Reset manual steering variables 
void reset_man_variables(void) {
	movement_mode = STOPPED;
}

// Interrupt used for firing solenoid, LED control and polling switch pin
ISR(TIMER1_COMPA_vect) {
	
	// Solenoid control
	if (fire_solenoid) {
		SOLENOID_PORT |= (1 << SOLENOID_PIN);
		solenoid_counter++;

		// After set period, reset solenoid
		if (solenoid_counter >= (SOLENOID_FIRING_PERIOD / TIMER_1_PERIOD)) {
			SOLENOID_PORT &= ~(1 << SOLENOID_PIN);
			solenoid_counter = 0;
			fire_solenoid = false;
		}
	}
	
	// LED control
	bool update_leds_flash_mode = false; 
	bool update_leds_boogie = false; 
	if ((led_mode == SLOW_FLASH) && (led_counter >= (SLOW_FLASH_PERIOD / TIMER_1_PERIOD))) {
		update_leds_flash_mode = true; 
	} else if ((led_mode == FLASH) && (led_counter >= (FLASH_PERIOD / TIMER_1_PERIOD))) {
		update_leds_flash_mode = true;
	} else if ((led_mode == FAST_FLASH) && (led_counter >= (FAST_FLASH_PERIOD / TIMER_1_PERIOD))) {
		update_leds_flash_mode = true;
	} else if ((led_mode == BOOGIE_SLOW) && (led_counter >= (SLOW_FLASH_PERIOD / TIMER_1_PERIOD))) {
		update_leds_boogie = true; 
	} else if ((led_mode == BOOGIE) && (led_counter >= (FLASH_PERIOD / TIMER_1_PERIOD))) {
		update_leds_boogie = true;
	} else if ((led_mode == BOOGIE_FAST) && (led_counter >= (FAST_FLASH_PERIOD / TIMER_1_PERIOD))) {
		update_leds_boogie = true;
	}
	
	if (update_leds_flash_mode) {
		if (leds_just_flashed) {
			turn_leds_off();
			leds_just_flashed = false;
		} else {
			set_leds();
			leds_just_flashed = true;
		}
		led_counter = 0;
		update_leds_flash_mode = false;
	} else if (update_leds_boogie) {
		set_leds();
		led_counter = 0; 
		update_leds_boogie = false; 
	} else {
		led_counter++;
	}
	
	// Polling switch pin
	if (PINC & (1 << SWITCH_PIN)) {
		// Switch pin is high / in OFF position
		partyBot_switched_off = true;
		cli(); // disable interrupts
		turn_leds_off(); 
		turn_motors_off();
	} else {
		// Switch pin is low / in ON position - actioned in main loop
	}
}

// Interrupt for touch sensor activation
ISR(PCINT0_vect) {
	
	// Check for TS1 press
	if (PINB & (1 << TS_1_PIN)) {
		// TS pin is high / TS pressed --> change LED colour
		if (current_led_colour == NO_COLOUR) {
			current_led_colour = RED;
		} else {
			current_led_colour++;
		}
		set_leds();
		
		// Set LCD colour to match if in sync
		if (lcd_in_sync) {
			send_serial_char(current_led_colour + SET_LCD_WHITE);
		}
	} else {
		// TS pin is low
	}
	
	// Check for TS2 press
	if (PINB & (1 << TS_2_PIN)) {
		// TS pin is high / TS pressed --> fire solenoid
		fire_solenoid = true;
	} else {
		// TS pin is low
	}
}


// Check for obstacle in front of partyBot. Returns false if obstacle detected and stops partyBot in this case.
bool coast_is_clear(void) {
	uint8_t front_distance_value = get_distance_sensor_value(FRONT_SENSOR);
	if (front_distance_value < FRONT_SENSOR_THRESHOLD) {
		return true; 
	} else {
		if (movement_mode == FORWARD) {
			send_serial_char(OBSTACLE_DETECTED);
			obstacle_just_detected = true; 
			pre_obstacle_led_mode = led_mode;
			pre_obstacle_led_colour = current_led_colour;
			current_led_colour = ORANGE; 
			set_leds();
			led_mode = FLASH; 
			leds_just_flashed = false;
			led_counter = 0;
		}
		turn_motors_off();
		return false;
	}
}

// Interrupt used for processing IR sensor data. 
ISR(TIMER2_COMPA_vect) {
	
	if (current_mode == AUTO_MODE) {
		if (forward_mvmt_disabled == false && coast_is_clear()) {	
			
			// Turn motors on if off
			if (movement_mode == STOPPED  || movement_mode == TIME_TO_REVERSE) {
				turn_motors_on();
			} 
			
			// Move partyBot forward if not already moving in this direction
			if (!(movement_mode == FORWARD)) {
				move_forward();
			}
		} else {
			if (obstacle_just_detected) {
				forward_mvmt_disabled = true;  // Only in auto mode
				obstacle_just_detected = false; 
			} else if (movement_mode == STOPPED) {
				stopped_counter++;
				if (stopped_counter >= (STOP_PERIOD / TIMER_2_PERIOD)) {
					movement_mode = TIME_TO_REVERSE;
					stopped_counter = 0;
					led_mode = pre_obstacle_led_mode;
					leds_just_flashed = false;
					led_counter = 0;
					current_led_colour = pre_obstacle_led_colour;
					set_leds(); 
				}
			} else if (movement_mode == TIME_TO_REVERSE) {
				// Reverse partyBot if coast is clear
				uint8_t rear_distance_value = get_distance_sensor_value(REAR_SENSOR);
				if (rear_distance_value < REAR_SENSOR_THRESHOLD) {
					// Back up
					turn_motors_on();
					reverse();
				} else {
					movement_mode = TIME_TO_TURN;
				}
			} else if (movement_mode == REVERSING) {
				uint8_t rear_distance_value = get_distance_sensor_value(REAR_SENSOR);
				if (rear_distance_value < REAR_SENSOR_THRESHOLD) {
					reversing_counter++;
					if (reversing_counter >= (REVERSE_PERIOD / TIMER_2_PERIOD)) {
						turn_motors_off();
						movement_mode = TIME_TO_TURN;
						reversing_counter = 0;
					}
				} else {
					turn_motors_off();
					movement_mode = TIME_TO_TURN;
					reversing_counter = 0;
				}
			} else if (movement_mode == TIME_TO_TURN) {
				// Turn towards the side with the most space (if enough space)
				uint8_t left_distance_value = get_distance_sensor_value(LEFT_SENSOR);
				uint8_t right_distance_value = get_distance_sensor_value(RIGHT_SENSOR);
				if (left_distance_value <= right_distance_value && left_distance_value < SIDE_SENSOR_THRESHOLD) {
					turn_motors_on();
					turn_left();
				} else if (right_distance_value < SIDE_SENSOR_THRESHOLD) {
					turn_motors_on();
					turn_right();
				} else {
					turn_motors_off();
					forward_mvmt_disabled = false; 
					movement_mode = TIME_TO_REVERSE;
				}
			} else if (movement_mode == TURNING_L) {
				turning_counter++;
				if (turning_counter >= (TURNING_PERIOD / TIMER_2_PERIOD)) {
					turn_motors_off();
					forward_mvmt_disabled = false;
					movement_mode = TIME_TO_REVERSE;
					turning_counter = 0;
				}
			} else if (movement_mode == TURNING_R) {
				turning_counter++;
				if (turning_counter >= (TURNING_PERIOD / TIMER_2_PERIOD)) {
					turn_motors_off();
					forward_mvmt_disabled = false;
					movement_mode = TIME_TO_REVERSE;
					turning_counter = 0;
				}
			}
		}
	} else {
		bool stop_partyBot = true; 
		if (movement_mode == FORWARD) {
			uint8_t front_distance_value = get_distance_sensor_value(FRONT_SENSOR);
			if (front_distance_value < FRONT_SENSOR_THRESHOLD) {
				// Turn motors on if off and move forward
				if (movement_mode == STOPPED) {
					turn_motors_on();
				}
				move_forward();
				stop_partyBot = false;
			}
		} else if (movement_mode == REVERSING) {
			uint8_t rear_distance_value = get_distance_sensor_value(REAR_SENSOR);
			if (rear_distance_value < REAR_SENSOR_THRESHOLD) {
				if (movement_mode == STOPPED) {
					turn_motors_on();
				}
				reverse();
				stop_partyBot = false;
			}
		} else if (movement_mode == TURNING_L) {
			uint8_t left_distance_value = get_distance_sensor_value(LEFT_SENSOR);
			if (left_distance_value < SIDE_SENSOR_THRESHOLD) {
				if (movement_mode == STOPPED) {
					turn_motors_on();
				}
				turn_left();
				stop_partyBot = false;
			}
		} else if (movement_mode == TURNING_R) {
			uint8_t right_distance_value = get_distance_sensor_value(RIGHT_SENSOR);
			if (right_distance_value < SIDE_SENSOR_THRESHOLD) {
				if (movement_mode == STOPPED) {
					turn_motors_on();
				}
				turn_right();
				stop_partyBot = false;
			}
		} 

		if (stop_partyBot) {
			turn_motors_off();
		}
	}
}

void partyBot_dance_pls() {
	
	turn_motors_off(); 
	
	// Store current state to restore post dance
	int previous_led_colour = current_led_colour;
	turn_leds_off();
	int previous_speed = current_speed;
	cli();
	_delay_ms(2000);
	
	// Dance !
	// Forward + skittle + back + skittle
	current_led_colour = RED; 
	set_leds(); 
	move_forward(); 
	_delay_ms(500);
	for (int i = 0; i < 3; i++) {
		int led_colour = current_led_colour;
		turn_leds_off();
		_delay_ms(100);
		current_led_colour = led_colour++;
		set_leds();
		_delay_ms(400);
	}
	turn_motors_off();
	SOLENOID_PORT |= (1 << SOLENOID_PIN);
	_delay_ms(SOLENOID_FIRING_PERIOD);
	SOLENOID_PORT &= ~(1 << SOLENOID_PIN);
	reverse(); 
	for (int i = 0; i < 4; i++) {
		int led_colour = current_led_colour;
		turn_leds_off();
		_delay_ms(100);
		current_led_colour++;
		current_led_colour = led_colour++;
		set_leds();
		_delay_ms(400);
	}
	turn_motors_off();
	SOLENOID_PORT |= (1 << SOLENOID_PIN);
	_delay_ms(SOLENOID_FIRING_PERIOD);
	SOLENOID_PORT &= ~(1 << SOLENOID_PIN);
	
	// Turn with lights
	current_speed = FAST_SPEED;
	current_led_colour = RED;
	set_leds();
	turn_left(); 
	_delay_ms(500);
	turn_leds_off();
	_delay_ms(500);
	current_led_colour = RED;
	set_leds();
	_delay_ms(500);
	turn_leds_off();
	
	// Then forward
	move_forward();
	current_led_colour = RED;
	set_leds();
	_delay_ms(500);
	turn_leds_off();
	_delay_ms(500);
	current_led_colour = RED;
	set_leds();
	_delay_ms(500);
	
	// Then back 
	reverse();
	turn_leds_off();
	_delay_ms(500);
	current_led_colour = RED;
	set_leds();
	_delay_ms(500);
	turn_leds_off();
	current_led_colour = RED;
	set_leds();
	_delay_ms(500);
	
	// Then turn the other way
	turn_leds_off();
	_delay_ms(200);
	current_led_colour = GREEN;
	set_leds();
	turn_right(); 
	_delay_ms(500);
	turn_leds_off();
	_delay_ms(500);
	current_led_colour = GREEN;
	set_leds();
	_delay_ms(500);
	turn_leds_off();
	_delay_ms(500);
	current_led_colour = GREEN;
	set_leds();
	_delay_ms(500);
	turn_leds_off();
	_delay_ms(500);
	current_led_colour = GREEN;
	set_leds();
	
	// Then forward
	move_forward();
	turn_leds_off();
	_delay_ms(500);
	current_led_colour = GREEN;
	set_leds();
	_delay_ms(500);
	turn_leds_off();
	_delay_ms(500);
	
	reverse();
	current_led_colour = GREEN;
	set_leds();
	_delay_ms(500);
	turn_leds_off();
	_delay_ms(500);
	current_led_colour = GREEN;
	set_leds();
	_delay_ms(500);
	
	// Then spin !!
	turn_leds_off();
	_delay_ms(200);
	current_speed = FASTEST_SPEED;
	turn_left(); 
	reset_rainbow_boogie_array();
	set_leds_rainbow_boogie();
	_delay_ms(300);
	set_leds_rainbow_boogie();
	_delay_ms(300);
	set_leds_rainbow_boogie();
	_delay_ms(300);
	set_leds_rainbow_boogie();
	_delay_ms(300);
	turn_right(); 
	set_leds_rainbow_boogie();
	_delay_ms(200);
	SOLENOID_PORT |= (1 << SOLENOID_PIN);
	set_leds_rainbow_boogie();
	_delay_ms(200);
	SOLENOID_PORT &= ~(1 << SOLENOID_PIN);
	set_leds_rainbow_boogie();
	_delay_ms(200);
	SOLENOID_PORT |= (1 << SOLENOID_PIN);
	set_leds_rainbow_boogie();
	_delay_ms(200);
	SOLENOID_PORT &= ~(1 << SOLENOID_PIN);
	set_leds_rainbow_boogie();
	_delay_ms(200);
	SOLENOID_PORT |= (1 << SOLENOID_PIN);
	_delay_ms(200);
	turn_motors_off();
	SOLENOID_PORT &= ~(1 << SOLENOID_PIN);
	
	// Restore partyBot post dance
	current_led_colour = previous_led_colour;
	current_speed = previous_speed;
	reversing_counter = 0;
	turning_counter = 0;
	stopped_counter = 0;
	forward_mvmt_disabled = false;
	obstacle_just_detected = false;
	solenoid_counter = 0;
	fire_solenoid = false;
	leds_just_flashed = true;
	led_counter = 0;
	reset_rainbow_boogie_array();
	sei();
}

// Interrupt handler for receiving UART data.
ISR(USART_RX_vect) {
	
	// Read character
	char receivedChar;
	receivedChar = UDR0;
	
	// Process character
	// Turn off in sync LED and LCD colour changes with colour commands
	if ((receivedChar >= TURN_LEDS_OFF && receivedChar <= SET_LEDS_RAINBOW) || 
			(receivedChar >= SET_LCD_WHITE && receivedChar <= SET_LCD_PURPLE)) {
		lcd_in_sync = false; 
	}
	
	// Action character
	if (receivedChar == MOVE_FORWARD && current_mode == MANUAL_MODE) {
		movement_mode = FORWARD;
	} else if (receivedChar == REVERSE && current_mode == MANUAL_MODE) {
		movement_mode = REVERSING;
	} else if (receivedChar == TURN_LEFT && current_mode == MANUAL_MODE) {
		movement_mode = TURNING_L;
	} else if (receivedChar == TURN_RIGHT && current_mode == MANUAL_MODE) {
		movement_mode = TURNING_R;
	} else if (receivedChar == CHANGE_AUTO_MODE && current_mode == MANUAL_MODE) {
		turn_motors_off();
		reset_auto_variables();
		current_mode = AUTO_MODE;
		send_serial_char(CHANGE_AUTO_MODE);
	} else if (receivedChar == CHANGE_MAN_MODE && current_mode == AUTO_MODE) {
		turn_motors_off();
		reset_man_variables(); 
		current_mode = MANUAL_MODE;
		send_serial_char(CHANGE_MAN_MODE);
	} else if (receivedChar == SET_SLOWEST_SPEED) {
		current_speed = SLOWEST_SPEED; 
		if (movement_mode != STOPPED) {
			turn_motors_on();
		}
	} else if (receivedChar == SET_SLOW_SPEED) {
		current_speed = SLOW_SPEED;
		if (movement_mode != STOPPED) {
			turn_motors_on();
		}
	} else if (receivedChar == SET_MID_SPEED) {
		current_speed = MID_SPEED;
		if (movement_mode != STOPPED) {
			turn_motors_on();
		}
	} else if (receivedChar == SET_FAST_SPEED) {
		current_speed = FAST_SPEED; 
		if (movement_mode != STOPPED) {
			turn_motors_on();
		}
	} else if (receivedChar == SET_FASTEST_SPEED) {
		current_speed = FASTEST_SPEED; 
		if (movement_mode != STOPPED) {
			turn_motors_on();
		}
	} else if (receivedChar == LCD_MSG_INCOMING) {
		receiving_message = true;
		// Initialise message array 
		for(int char_pos = 0; char_pos < sizeof(message); char_pos++) {
			message[char_pos] = 0;
		}
		message_length = 0;
	} else if (receivedChar == FIRE_SOLENOID) {
		fire_solenoid = true; 
		solenoid_counter = 0;
	} else if (receivedChar == SET_LEDS_RED) {
		current_led_colour = RED;
		set_leds();
	} else if (receivedChar == SET_LEDS_ORANGE) {
		current_led_colour = ORANGE;
		set_leds();
	} else if (receivedChar == SET_LEDS_YELLOW) {
		current_led_colour = YELLOW;
		set_leds();
	} else if (receivedChar == SET_LEDS_GREEN) {
		current_led_colour = GREEN;
		set_leds();
	} else if (receivedChar == SET_LEDS_BLUE) {
		current_led_colour = BLUE;
		set_leds();
	} else if (receivedChar == SET_LEDS_LIGHTBLUE) {
		current_led_colour = LIGHTBLUE;
		set_leds(); 
	}  else if (receivedChar == SET_LEDS_VIOLET) {
		current_led_colour = VIOLET;
		set_leds();
	} else if (receivedChar == SET_LEDS_PINK) {
		current_led_colour = PINK;
		set_leds();
	} else if (receivedChar == SET_LEDS_WHITE) {
		current_led_colour = WHITE;
		set_leds();
	} else if (receivedChar == SET_LEDS_RAINBOW) {
		current_led_colour = RAINBOW;
		set_leds();
	} else if (receivedChar == TURN_LEDS_OFF) {
		turn_leds_off();
	} else if (receivedChar == SET_SLOW_FLASH || receivedChar == SET_FLASH || receivedChar == SET_FAST_FLASH) {
		if (led_mode == BOOGIE_SLOW || led_mode == BOOGIE || led_mode == BOOGIE_FAST) {
			set_leds_rainbow();
		}
		if (led_mode != OFF) {
			switch (receivedChar) {
				case SET_SLOW_FLASH:
					led_mode = SLOW_FLASH;
					leds_just_flashed = false;
					led_counter = 0;
					break; 
				case SET_FLASH: 
					led_mode = FLASH;
					leds_just_flashed = false;
					led_counter = 0;
					break;
				case SET_FAST_FLASH: 
					led_mode = FAST_FLASH;
					leds_just_flashed = false;
					led_counter = 0;
					break;
			}
			leds_just_flashed = true; 
			led_counter = 0;
		}
	} else if (receivedChar == SET_SLOW_BOOGIE) {
		reset_rainbow_boogie_array(); 
		led_mode = BOOGIE_SLOW;
	} else if (receivedChar == SET_BOOGIE) {
		reset_rainbow_boogie_array();
		led_mode = BOOGIE;
	} else if (receivedChar == SET_FAST_BOOGIE) {
		reset_rainbow_boogie_array(); 
		led_mode = BOOGIE_FAST;
	} else if (receivedChar == STOP_FLASH) {
		if (current_mode != OFF) {
			led_mode = NO_FLASH;
			set_leds();
		}
	} else if (receivedChar == PARTYBOT_DANCE) {
		partyBot_dance_pls();
	} else if (receivedChar >= SET_LCD_WHITE && receivedChar <= SET_LCD_PURPLE) {
		send_serial_char(receivedChar);	
	} else if (receivedChar == PARTYBOT_STOP) {
		turn_motors_off();
	} else if (receivedChar == PARTYBOT_OFF_CODE) {
		turn_leds_off();
		turn_motors_off();
		reset_man_variables();
		current_mode = MANUAL_MODE;
		send_serial_char(CHANGE_MAN_MODE);
	} else if (receivedChar == PARTYBOT_ON_CODE) {
		reset_auto_variables();
		reset_man_variables();
		init_partyBot();
	} else if (receivedChar == BLUETOOTH_TEST) {
		send_serial_char(BLUETOOTH_TEST);
		if (current_mode == AUTO_MODE) {
			send_serial_char(CHANGE_AUTO_MODE);
		} else {
			send_serial_char(CHANGE_MAN_MODE);
		}
	} 
}

int main(void) {
	
	// Initialise partyBot
	init_partyBot(); 
	
    while (1) {
		// Check if partyBot switched back on (if off) 
		if (partyBot_switched_off && ((PINC & (1 << SWITCH_PIN)) == 0)) {
			_delay_ms(250);
			init_partyBot();
			reset_auto_variables();
			reset_man_variables();
			partyBot_switched_off = false;
		}
    }
}