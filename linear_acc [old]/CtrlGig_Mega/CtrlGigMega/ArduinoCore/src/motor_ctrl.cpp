#include "defines.h"
#include "variables.h"
//#include "TimerOne.h"

//the acceleration and break is predefined by the program and its goal is to reduce vibration. Its module cannot be changed by communication
//the possible speeds are predefined to improve code performance

//the timer period that should be apply to get a determined angular velocity of the motor
//the angular velocity is the position in the timerPeriod vector, for example, position 45 refers to 45 degrees per second.
//up to 15 degrees per second, the timer should be prescale by 8.
// the position 0 and 1 of this vector makes the timer count every 1 millisecond 
//positions 0 and 1 cannot be used

void configPinMotors(void){
	pinMode(MBASE_DIR, OUTPUT);
	pinMode(MBASE_STEP, OUTPUT);
	pinMode(MBASE_MS3, OUTPUT);
	pinMode(MBASE_MS2, OUTPUT);
	pinMode(MBASE_MS1, OUTPUT);
	pinMode(MBASE_ENABLE, OUTPUT);
	pinMode(MBASE_RESET, OUTPUT);

	pinMode(MTOP_DIR, OUTPUT);
	pinMode(MTOP_STEP, OUTPUT);
	pinMode(MTOP_MS3, OUTPUT);
	pinMode(MTOP_MS2, OUTPUT);
	pinMode(MTOP_MS1, OUTPUT);
	pinMode(MTOP_ENABLE, OUTPUT);
	pinMode(MTOP_RESET, OUTPUT);
}

void mBaseSetMicroStep(TYPE_MICRO_STEP step_type){
  switch(step_type){
    case FULL_STEP:
		digitalWrite(MBASE_MS3,LOW);
		digitalWrite(MBASE_MS2,LOW);
		digitalWrite(MBASE_MS1,LOW);
	break;
	
	case HALF_STEP:
		digitalWrite(MBASE_MS3,LOW);
		digitalWrite(MBASE_MS2,LOW);
		digitalWrite(MBASE_MS1,HIGH);
	break;
	
	case QUARTER_STEP:
		digitalWrite(MBASE_MS3,LOW);
		digitalWrite(MBASE_MS2,HIGH);
		digitalWrite(MBASE_MS1,LOW);
	break;
	
	case EIGHTH_STEP:
		digitalWrite(MBASE_MS3,LOW);
		digitalWrite(MBASE_MS2,HIGH);
		digitalWrite(MBASE_MS1,HIGH);
	break;
	
	case SIXTEENTH_STEP:
		digitalWrite(MBASE_MS3,HIGH);
		digitalWrite(MBASE_MS2,HIGH);
		digitalWrite(MBASE_MS1,HIGH);
	break;    
  }
}

void mTopSetMicroStep(TYPE_MICRO_STEP step_type){
  switch(step_type){
    case FULL_STEP:
		digitalWrite(MTOP_MS3,LOW);
		digitalWrite(MTOP_MS2,LOW);
		digitalWrite(MTOP_MS1,LOW);
	break;
	
	case HALF_STEP:
		digitalWrite(MTOP_MS3,LOW);
		digitalWrite(MTOP_MS2,LOW);
		digitalWrite(MTOP_MS1,HIGH);
	break;
	
	case QUARTER_STEP:
		digitalWrite(MTOP_MS3,LOW);
		digitalWrite(MTOP_MS2,HIGH);
		digitalWrite(MTOP_MS1,LOW);
	break;
	
	case EIGHTH_STEP:
		digitalWrite(MTOP_MS3,LOW);
		digitalWrite(MTOP_MS2,HIGH);
		digitalWrite(MTOP_MS1,HIGH);
	break;
	
	case SIXTEENTH_STEP:
		digitalWrite(MTOP_MS3,HIGH);
		digitalWrite(MTOP_MS2,HIGH);
		digitalWrite(MTOP_MS1,HIGH);
	break;    
  }
}

TYPE_MICRO_STEP nextStepConfig(TYPE_MICRO_STEP step_type){
	//full step does not work well at 2khz
	switch(step_type){
		case FULL_STEP:
			return(HALF_STEP);
		break;
		
		case HALF_STEP:
			return(QUARTER_STEP);
		break;
		
		case QUARTER_STEP:
			return(EIGHTH_STEP);
		break;
		
		case EIGHTH_STEP:
			return(SIXTEENTH_STEP);
		break;
		
		case SIXTEENTH_STEP:
			return(HALF_STEP);
		break;
	}
}

uint8_t checkBaseLimit(s_motor_ctrl *motor,int16_t *c_limit){
	//if return zero we cannot increment.
	if(motor->f_dir == 0){
		if(*c_limit<MAX_STEPS_SINGLE_DIR_BASE){	//if we can still more steps in this direction
			switch(motor->micro_step_config){
				case FULL_STEP:
				*c_limit = *c_limit+16;
				break;
				
				case HALF_STEP:
					*c_limit = *c_limit+8;
				break;
				
				case QUARTER_STEP:
					*c_limit = *c_limit+4;
				break;
				
				case EIGHTH_STEP:
					*c_limit = *c_limit+2;
				break;
				
				case SIXTEENTH_STEP:
					*c_limit = *c_limit+1;
				break;
			}
			return(1);
		}
		else{	// if we cannot: if(*c_limit<MAX_STEPS_SINGLE_DIR_BASE)
			return(0);//cannot make more steps in this direction
		}
	}
	else{
		if(*c_limit>(-MAX_STEPS_SINGLE_DIR_BASE)){	//if we can still more steps in this direction
			switch(motor->micro_step_config){
				case FULL_STEP:
				*c_limit = *c_limit-16;
				break;
				
				case HALF_STEP:
					*c_limit = *c_limit-8;
				break;
				
				case QUARTER_STEP:
					*c_limit = *c_limit-4;
				break;
				
				case EIGHTH_STEP:
					*c_limit = *c_limit-2;
				break;
				
				case SIXTEENTH_STEP:
					*c_limit = *c_limit-1;
				break;
			}
			return(1);
		}
		else{	// if we cannot: if(*c_limit>(-MAX_STEPS_SINGLE_DIR_BASE))
			return(0);//cannot make more steps in this direction
		}
		
	}
}

void checkBaseBlock(void){
	// Turn on the limiting led
	if(gcs.base_limit >= MAX_STEPS_SINGLE_DIR_BASE){
		gfs.positive_limit = 1;
		digitalWrite(PIN_LED_BASE_LOCK, HIGH);
	}
	else if(gcs.base_limit <= -MAX_STEPS_SINGLE_DIR_BASE){
		gfs.negative_limit = 1;
		digitalWrite(PIN_LED_BASE_LOCK, HIGH);
	}
	else{
		gfs.positive_limit = gfs.negative_limit = 0;
		digitalWrite(PIN_LED_BASE_LOCK, LOW);
	}
}

void limitBaseSteps(void){
	//check if we can make one more step. Set the flags if we cannot, if we can, rise the step pin
	if (gs_base_ctrl.f_dir == DIR_NEGATIVE) {          //if we are turning in the CCW direction
		if(gcs.base_limit > -MAX_STEPS_SINGLE_DIR_BASE){    //if we still able to turn a bit more:
			gcs.base_limit--;
			SET_MBASE_STEP_PIN;
		}
	}
	else{                           //if we are turning in the CW direction
		if(gcs.base_limit < MAX_STEPS_SINGLE_DIR_BASE){   //if we still able to turn a bit more:
			gcs.base_limit++;
			SET_MBASE_STEP_PIN;
		}
	}
}




