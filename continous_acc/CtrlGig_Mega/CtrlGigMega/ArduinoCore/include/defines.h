#ifndef DEFINES
#define DEFINES

#include <Arduino.h>
#define F_CPU 16000000UL
#define _DEBUG

//#define _DEBUG_COM_WHILE_SPEED_CTRL

//#define max(a,b) \
//({ __typeof__ (a) _a = (a); \
//  __typeof__ (b) _b = (b); \
//_a > _b ? _a : _b; })
//
//#define min(a,b) \
//({ __typeof__ (a) _a = (a); \
//  __typeof__ (b) _b = (b); \
//_a < _b ? _a : _b; })

//Casting and masks for 8 bits for different variables sizes and bytes positions.
#define lo8(x) ((uint8_t)((x) & 0xff))
#define hi8(x) ((uint8_t)((x & 0xff00)>>8))
#define hi16(x) ((uint8_t)((x & 0xff0000)>>16))
#define hi24(x) ((uint8_t)((x & 0xff000000)>>24))

#define UART_HEADER_1		'S'
#define UART_HEADER_COM     'c'
#define UART_HEADER_START   't'
#define REC_VECTOR_SIZE      2000

const uint8_t END_MSG[] = {0x00, 0x80, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00};

#define MBASE_DIR		22
#define MBASE_STEP		24
#define MBASE_RESET     26
#define MBASE_MS3		2
#define MBASE_MS2		3
#define MBASE_MS1		4
#define MBASE_ENABLE    5


#define MTOP_DIR		6
#define MTOP_STEP		7
#define MTOP_RESET      8
#define MTOP_MS3		9
#define MTOP_MS2		10
#define MTOP_MS1		11
#define MTOP_ENABLE     12


#define PIN_LED_NATIVE    13
#define PIN_LED_BASE_LOCK 25
#define PIN_MONING_LED	  23

#define SERIAL_BAUD     57600

#define DEFAULT_MS                 16                                   //the microsteping that is used here
#define FS_REVOLUTION              200                                  //full steps per revolution
#define MAX_STEPS_SINGLE_DIR_BASE ((int32_t)(DEFAULT_MS*FS_REVOLUTION*2))               //this is 16*200*2, two turns of the top motor 6400 steps 
#define INITIAL_SPEED             10                                    //Initial speed in degrees per second
#define INITIAL_SPEED_2			  100									//INITIAL_SPEED*INITIAL_SPEED
#define MAX_SPEED                 720                                   //Max angular speed (2 turns per second)
#define INITIAL_SPEED_PERIOD      ((float)((1E6*360)/(INITIAL_SPEED*DEFAULT_MS*FS_REVOLUTION)))   //Maximum period which corresponds to the minimum speed
#define MAX_SPEED_PERIOD          ((float)((1E6*360)/(MAX_SPEED*DEFAULT_MS*FS_REVOLUTION)))     //Minimum period which corresponds to the maximum speed
#define ACC_SCALER				  1                                   // a constant that will multiply the acceleration module
#define INV_ACC_SCALER			  1
#define BREAKING_SCALER			  1
#define INV_BREAKING_SCALER		  1
#define RATIO_ACC_BREAKING		  0.5		//this is the % of total steps that will be dedicated to acceleration. if the acceleration is twice slower than breaking, than this ratio should be .666 for example

#define SPEED_UPDATE_TIMER_TOP	  11119					//this will give approximately 720Hz - period of the timer that will update the speed (will 
#define ACC_MODULE				  (720*ACC_SCALER)		//degrees per second square
#define BREAKING_MODULE			  (720*BREAKING_SCALER)
#define D_THETA					  (360/(DEFAULT_MS*FS_REVOLUTION))
#define DEN_CALC_ACC_STEPS		  (float)(162)					//360/(2*ACC_MODULE*D_THETA));

typedef enum{
  FULL_STEP,
  HALF_STEP,
  QUARTER_STEP,
  EIGHTH_STEP,
  SIXTEENTH_STEP,
}TYPE_MICRO_STEP;

typedef enum{
  IDLE,
  ROTATING,
  RECEIVING,
}TYPE_SYSTEM_STATE;

//direction definitions
typedef enum{
  DIR_NEGATIVE = 0,     //CW
  DIR_POSITIVE = 1,     //CCW
}TYPE_DIRECTION;


//PIN DEFINES: FOR ARDUINO MEGA -- those defines improve the performance of a key part of the code
#define CLEAR_NATIVE_LED		PORTB &= ~(1<<(PB7));
#define SET_NATIVE_LED			PORTB |= (1<<(PB7));
#define TOGGLE_NATIVE_LED		PINB |= (1<<(PB7));

#define CLEAR_MTOP_STEP_PIN		PORTH &= ~(1<<(PH4));
#define SET_MTOP_STEP_PIN		PORTH |= (1<<(PH4));

#define CLEAR_MBASE_STEP_PIN	PORTA &= ~(1<<(PA2));
#define SET_MBASE_STEP_PIN		PORTA |= (1<<(PA2));

#define TOGGLE_MOVING_LED_PIN	PINA |= (1<<(PA1));
#define CLEAR_MOVING_LED_PIN	PORTA &= ~(1<<(PA1));
#define SET_MOVING_LED_PIN		PORTA |= (1<<(PA1));

#endif

