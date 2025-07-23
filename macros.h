/*
 * macros.h
 *
 * Created: 06/01/2025 04:17:57 PM
 * Author : gatorBots
 */ 

/* Timer Macros */
#define TIMER_1_PERIOD 30 // in ms
#define TIMER_2_PERIOD 30 // in ms

/* Movement Macros */
#define AUTO_MODE   0
#define MANUAL_MODE 1

#define FRONT_SENSOR_THRESHOLD 100
#define REAR_SENSOR_THRESHOLD  100
#define SIDE_SENSOR_THRESHOLD  130
/* Note from datasheet: voltage ~0.4V when no object detected (>=150cm away) & ~2.5V (half of reference voltage) when object ~20cm away */

// Movement Modes
#define STOPPED			0
#define FORWARD		    1
#define REVERSING	    2
#define TURNING_L	    3
#define TURNING_R	    4
#define TIME_TO_REVERSE 5
#define TIME_TO_TURN    6

// Auto Movement Periods
#define STOP_PERIOD    500  // in ms
#define REVERSE_PERIOD 1000  // in ms
#define TURNING_PERIOD 1000  // in ms

/* LED Macros */
#define NUMBER_LEDS   32
#define NUM_RING_LEDS 12
#define NUM_COLOURS    8

// LED Colours
#define WHITE     0
#define RED		  1
#define ORANGE    2
#define YELLOW	  3
#define GREEN	  4
#define LIGHTBLUE 5
#define BLUE	  6
#define VIOLET    7
#define PINK	  8
#define RAINBOW   9
#define NO_COLOUR 10

// LED Modes
#define OFF		    0
#define NO_FLASH    1
#define SLOW_FLASH  2
#define FLASH       3
#define FAST_FLASH  4
#define BOOGIE_SLOW 5
#define BOOGIE      6
#define BOOGIE_FAST 7

// LED Flash Periods
#define SLOW_FLASH_PERIOD 1500  // in ms
#define FLASH_PERIOD      1000  // in ms
#define FAST_FLASH_PERIOD 500  // in ms

// LED Strip Pin 
#define LED_DDR  DDRB
#define LED_PORT PORTB
#define LED_PIN  PINB1

// Ring LED Pin 
#define RING_LED_DDR  DDRB
#define RING_LED_PORT PORTB
#define RING_LED_PIN  PINB2

// Test LED Pin
#define TEST_LED_DDR  DDRC
#define TEST_LED_PORT PORTC
#define TEST_LED_PIN  PINC4

// Touch Sensor Pins
#define TS_DDR   DDRB
#define TS_PORT  PORTB
#define TS_1_PIN PINB7
#define TS_2_PIN PINB6 

/* Motor Macros */
// Motor Pins
#define M1_DDR		DDRD
#define M1_PORT		PORTD
#define M2_DDR		DDRD
#define M2_PORT		PORTD
#define M2_BWD_DDR  DDRB
#define M2_BWD_PORT PORTB
#define M1_EN_PIN	PIND6
#define M1_FWD_PIN	PIND7
#define M1_BWD_PIN	PIND2
#define M2_EN_PIN	PIND5
#define M2_FWD_PIN	PIND4
#define M2_BWD_PIN	PINB0

// Motor Speeds
#define SLOWEST_SPEED 1
#define SLOW_SPEED	  2
#define MID_SPEED	  3
#define FAST_SPEED    4
#define FASTEST_SPEED 5

// Motor Speed PWM Values
#define M1_SLOWEST_PWM 233
#define M2_SLOWEST_PWM 190
#define M1_SLOW_PWM	   243
#define M2_SLOW_PWM	   200
#define M1_MID_PWM	   248  // M1 = RIGHT
#define M2_MID_PWM	   210  // M2 = LEFT
#define M1_FAST_PWM	   251
#define M2_FAST_PWM	   220
#define M1_FASTEST_PWM 255
#define M2_FASTEST_PWM 230

/* Distance Sensor Macros */
// Distance Sensor Pins
#define IR_SENSOR_DDR	 DDRC
#define IR_SENSOR_PORT   PORTC
#define FRONT_SENSOR_PIN PINC2
#define REAR_SENSOR_PIN  PINC1
#define LEFT_SENSOR_PIN  PINC0
#define RIGHT_SENSOR_PIN PINC3

// Distance Sensor IDs
#define FRONT_SENSOR 0
#define REAR_SENSOR  1
#define LEFT_SENSOR  2
#define RIGHT_SENSOR 3

/* Solenoid Macros */
#define SOLENOID_FIRING_PERIOD 300  // in ms

// Solenoid Pin
#define SOLENOID_DDR  DDRD
#define SOLENOID_PORT PORTD
#define SOLENOID_PIN  PIND3

// Switch Pin
#define SWITCH_PORT PORTC
#define SWITCH_PIN  PINC5

/* Serial Communication Macros */
#define BAUD_RATE      9600 // in bps
#define BAUD_PRESCALER F_CPU / 16 / BAUD_RATE - 1

// Remote Message Codes
#define TURN_LEDS_OFF      0
#define SET_LEDS_WHITE     1 
#define SET_LEDS_PINK      2
#define SET_LEDS_RED       3
#define SET_LEDS_ORANGE    4
#define SET_LEDS_YELLOW    5
#define SET_LEDS_GREEN     6
#define SET_LEDS_LIGHTBLUE 7
#define SET_LEDS_BLUE      8
#define SET_LEDS_VIOLET    9
#define SET_LEDS_RAINBOW   10 

#define SET_FLASH          11
#define SET_SLOW_FLASH     12
#define SET_FAST_FLASH     13
#define SET_BOOGIE         14
#define SET_SLOW_BOOGIE    15
#define SET_FAST_BOOGIE    16
#define STOP_FLASH         17

#define SET_LCD_WHITE      18
#define SET_LCD_RED        19
#define SET_LCD_ORANGE     20
#define SET_LCD_YELLOW     21
#define SET_LCD_GREEN      22
#define SET_LCD_LIGHTBLUE  23
#define SET_LCD_BLUE       24
#define SET_LCD_PURPLE     25
#define SET_LCD_PINK       26

#define SET_SLOWEST_SPEED  27
#define SET_SLOW_SPEED     28
#define SET_MID_SPEED      29
#define SET_FAST_SPEED     30
#define SET_FASTEST_SPEED  31

#define MOVE_FORWARD       32 
#define REVERSE            33 
#define TURN_LEFT          34 
#define TURN_RIGHT         35 
#define PARTYBOT_STOP      36

#define PARTYBOT_OFF_CODE  37
#define PARTYBOT_ON_CODE   38
#define CHANGE_AUTO_MODE   39 
#define CHANGE_MAN_MODE    40 

#define PARTYBOT_DANCE     41 
#define FIRE_SOLENOID      42 
#define OBSTACLE_DETECTED  43
#define BLUETOOTH_TEST     44
#define LCD_MSG_INCOMING   45


