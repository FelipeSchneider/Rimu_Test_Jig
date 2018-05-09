#ifndef VARIABLES
#define VARIABLES
#include <Arduino.h>
#include "defines.h"

typedef struct{
	boolean			f_rdy_4_command;   //if the motors are ready to start a new movement, these both flags will be set.
	boolean			f_stop;
	boolean			f_reset;
	boolean			f_enable;
	TYPE_DIRECTION  f_dir;				//see the direction definitions
	TYPE_MICRO_STEP micro_step_config;  //always 16 microstep
  
	uint16_t		prog_steps;			//number of steps of this specify command
	int16_t			prog_speed;			//cruse speed of specific command
	uint16_t		n_to_cruse_speed;   //number of steps to cruse speed, at this point we will stop incrementing the speed
	uint16_t		n_to_start_breaking;//number of steps that we must count to start breaking
	uint16_t		steps_breaking;		//number of steps that will be spend breaking
	uint16_t		c_steps_made;		//number of steps already completed for this command
  
	float		speed;					//actual motor speed
  
}s_motor_ctrl;

typedef struct{
	TYPE_SYSTEM_STATE state;
	boolean       positive_limit;     //base cannot rotate anymore in the positive direction
	boolean       negative_limit; 
	boolean       rdy_to_rotate;      //tells me if we have received the commands from uart and are ready to rotate... waiting for the St command 

}s_global_flags;

typedef struct{
	int32_t base_limit;       //this is a safety counter that will not allow more than MAX_STEPS_SINGLE_DIR_BASE steps in the same direction for the base motor.
							//This is necessary for protect the wirings of the top motor. This is a sign variable - must be compared with -MAX_STEPS_SINGLE_DIR_BASE also.
// 	int32_t base_partial;     //counts how many steps we have performance for the base motor for a given set of commands
//   
// 	int32_t top_partial;      //counts how many steps we have performance for the top motor for a given set of commands
}s_global_counters;

extern s_motor_ctrl gs_top_ctrl, gs_base_ctrl;
extern s_global_flags gfs;
extern s_global_counters gcs;

extern uint8_t gv_ctrl_com[REC_VECTOR_SIZE];    //this will keep the bytes that come from the uart
extern uint16_t gc_ctrl_com;					//points where we are in the gv_ctrl_com vector
extern const PROGMEM uint16_t timerPeriod[721];
#endif


